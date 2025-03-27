#pragma once

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdUtils/stageCache.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/scenegraph/usd/usd/prim.h>
#include <usdrt/scenegraph/usd/usd/tokens.h>
#include <usdrt/scenegraph/usd/sdf/path.h>
#include <usdrt/scenegraph/base/gf/vec3f.h>
#include <usdrt/scenegraph/base/gf/quatf.h>
#include <usdrt/scenegraph/usd/usd/attribute.h>
#include <omni/fabric/usd/PathConversion.h>

#include <usdrt/scenegraph/usd/physxSchema/physxForceAPI.h>
#include <usdrt/scenegraph/usd/usdPhysics/rigidBodyAPI.h>
#include <usdrt/scenegraph/usd/physxSchema/physxDeformableAPI.h>
#include <usdrt/scenegraph/usd/usdPhysics/articulationRootAPI.h>
#include <usdrt/scenegraph/usd/usdPhysics/revoluteJoint.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <execution>
#include <random>
#include <regex>
#include <vector>

#include "xoshiro256ss.h"
#include <Px4MathUtils.hpp>
#include <usdrt/gf/vec.h>

namespace uosm {
namespace isaac {
namespace px4 {

static usdrt::UsdStageRefPtr getActiveStage(void)
{
    const std::vector<PXR_NS::UsdStageRefPtr> stages = PXR_NS::UsdUtilsStageCache::Get().GetAllStages();
    if (stages.size() != 1)
        return nullptr;

    auto stage_id = PXR_NS::UsdUtilsStageCache::Get().GetId(stages[0]).ToLongInt();
    return usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stage_id));
}

class Px4Multirotor
{
public:
    struct VehicleParameters
    {
        // https://openusd.org/release/wp_rigid_body_physics.html
        float mass = 0.0f;                         // Vehicle mass in kg.
        usdrt::GfVec3f diagonal_inertias{0, 0, 0}; // Diagonal inertias of the vehicle (Ixx, Iyy, Izz) in kg*m^2.
        usdrt::GfVec3f center_of_mass{0, 0, 0};    // Center of mass of the vehicle (x, y, z) in meters.

        // https://database.tytorobotics.com/tests/k9w/t-motor-2216-900kv-1045prop-4s).
        float moment_constants = 0.0000001738f; // Moment constants
        float force_constants = 0.00001211f;    // Force constants
        float max_rotor_velocity = 1032.0f;     // Maximum rotor velocity in rad/s.
        float params_std_dev = 0.05f;           // Standard deviation
        float rotor_time_constants = 0.05f;     // Time constants

        // servo specs
        float max_torque = 0.11f; // Maximum torque in kg*m^-1
        float max_angle = 180.0f; // Maximum angle in degrees
        float servo_time_constants = 0.2f;     // Time constants

        // https://ardupilot.org/copter/docs/airspeed-estimation.html
        usdrt::GfVec3f rotor_drag_coef{0, 0, 0}; // Rotor drag coefficients
        float airframe_drag = 0.20f;             // Airframe drag coefficient
    };

    struct VechicleMotion
    {
        // https://openusd.org/release/wp_rigid_body_physics.html
        // https://docs.omniverse.nvidia.com/isaacsim/latest/reference_conventions.htmls
        usdrt::GfVec3d translate{0, 0, 0}; // m
        usdrt::GfQuatd orient{1, 0, 0, 0};
        usdrt::GfVec3f velocity{0, 0, 0};         //  m/s
        usdrt::GfVec3f angular_velocity{0, 0, 0}; // deg/s
    };

    struct ActuatorControl
    {
        std::array<float, 16> actuator_cmd;
        std::array<float, 16> actuator_setpoint;
    };

    Px4Multirotor() : parameters()
    {
        rotor_joint_group.reserve(16);
        rotor_body_group.reserve(16);
        actuator_joint_group.reserve(6);
        actuator_body_group.reserve(6);
    }

    VehicleParameters &getParameters() { return parameters; }

    size_t getRotorCount() const { return rotor_joint_group.size(); }
    size_t getActuatorCount() const { return actuator_joint_group.size(); }

    const void reset(void)
    {
        setVehicleForceAndTorque(usdrt::GfVec3f(0, 0, 0), usdrt::GfVec3f(0, 0, 0));
        std::for_each(rotor_joint_group.begin(), rotor_joint_group.end(), [&](const auto &joint)
                        {
                    size_t idx = &joint - &rotor_joint_group[0];
                    setRotorVelocity(0, static_cast<int>(idx)); });
        std::for_each(rotor_body_group.begin(), rotor_body_group.end(), [&](const auto &body)
                        {
                    size_t idx = &body - &rotor_body_group[0];
                    setRotorForce(0, static_cast<int>(idx)); });
        vehicle_base_link = usdrt::UsdPrim{nullptr};
        vehicle_articulation_root = usdrt::UsdPrim{nullptr};
        rotor_joint_group.clear();
        rotor_body_group.clear();
        actuator_joint_group.clear();
        actuator_body_group.clear();
    }

    /**
     * @brief Load a vehicle from USD stage.
     *
     * It expects the vehicle to be represented as an articulation root with a base link
     * as a child. The base link is expected to be a RigidBody with a name matching the
     * pattern "base[_]?link.*".
     *
     * @param target The path to the vehicle prim in the USD stage.
     * @return true if the vehicle is successfully loaded, false otherwise.
     */
    bool loadVehicle(const omni::fabric::PathC &target)
    {
        auto stage = getActiveStage();
        if (!stage)
            return false;

        const std::string path = omni::fabric::toSdfPath(target).GetString();
        usdrt::UsdPrim prim = stage->GetPrimAtPath(target);
        if (!prim)
            return false;

        const usdrt::TfToken rigidBodyAPI = usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType();
        const usdrt::TfToken articulationRootAPI = usdrt::UsdPhysicsArticulationRootAPI::_GetStaticTfType();
        std::regex pattern("[bB]ase[_]?[lL]ink.*");
        // Case 1: Prim is articulation root
        if (prim.HasAPI(articulationRootAPI))
        {
            for (const auto &child : prim.GetAllChildren())
            {
                if (child.HasAPI(rigidBodyAPI) && std::regex_match(child.GetName().GetText(), pattern))
                {
                    vehicle_articulation_root = prim;
                    vehicle_base_link = child;
                    return initVehicle();
                }
            }
        }
        // Case 2: Prim is base link
        else if (prim.HasAPI(rigidBodyAPI) && std::regex_match(prim.GetName().GetText(), pattern))
        {
            auto parent = prim.GetParent();
            if (parent.HasAPI(articulationRootAPI))
            {
                vehicle_articulation_root = parent;
                vehicle_base_link = prim;
                return initVehicle();
            }
        }

        CARB_LOG_WARN("%s is not base_link or does not have RigidBody/ArticulationRoot API.", path.c_str());
        return false;
    }

    bool isVehiclePathEqual(omni::fabric::PathC &target)
    {
        if (!vehicle_articulation_root || !vehicle_base_link)
            return false;

        std::string target_path = omni::fabric::toSdfPath(target).GetString();
        std::string vehicle_root_path = vehicle_articulation_root.GetPrimPath().GetString();
        std::string vehicle_baselink_path = vehicle_base_link.GetPrimPath().GetString();
        return vehicle_root_path == target_path || vehicle_baselink_path == target_path;
    }

    const void getVehicleMotion(VechicleMotion &motions)
    {
        if (!vehicle_base_link.IsValid())
            return;

        vehicle_base_link.GetAttribute(usdrt::TfToken("xformOp:translate")).Get<usdrt::GfVec3d>(&motions.translate);
        vehicle_base_link.GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&motions.orient);
        vehicle_base_link.GetAttribute(usdrt::TfToken("physics:velocity")).Get<usdrt::GfVec3f>(&motions.velocity);
        vehicle_base_link.GetAttribute(usdrt::TfToken("physics:angularVelocity")).Get<usdrt::GfVec3f>(&motions.angular_velocity);
    }

    const void getRotorTorqueAxis(const float &moment, usdrt::GfVec3f &torque_axis, int index)
    {
        if (rotor_body_group.empty() || index < 0 || index >= static_cast<int>(rotor_body_group.size()))
            return;

        usdrt::GfQuatd orient(0, 0, 0, 0);
        rotor_body_group[index].GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&orient);
        torque_axis = moment * getQuatAxis(static_cast<usdrt::GfQuatf>(orient));
    }

    void computeDynamics(ActuatorControl &m_actuator_control, std::vector<int> &dir)
    {
        std::random_device rd;
        xoshiro256ss rng(rd());
        std::normal_distribution<float> vel_normal_dist(parameters.max_rotor_velocity, parameters.max_rotor_velocity * parameters.params_std_dev);
        std::normal_distribution<float> km_normal_dist(parameters.moment_constants, parameters.moment_constants * parameters.params_std_dev);
        std::normal_distribution<float> kf_normal_dist(parameters.force_constants, parameters.force_constants * parameters.params_std_dev);
        std::normal_distribution<float> tau_normal_dist(parameters.rotor_time_constants, parameters.rotor_time_constants * parameters.params_std_dev);

        const float omega = vel_normal_dist(rng);
        const float omega_squared = std::pow(omega, 2.0f);
        std::vector<usdrt::GfVec3f> rotor_torque(getRotorCount());
        const int unused_cmd = 16 - getRotorCount() - getActuatorCount();
        std::for_each(std::execution::par,
                        m_actuator_control.actuator_cmd.begin(), m_actuator_control.actuator_cmd.end() - unused_cmd,
                        [&](const float &cmd)
                        {
                            size_t idx = &cmd - &m_actuator_control.actuator_cmd[0];
                            float tau = std::clamp(tau_normal_dist(rng), 0.0f, 1.0f);
                            if (idx < getRotorCount())
                            {
                                // update for rotors (exponential smoothing)
                                // current_setpoint = prev_setpoint + tau * (sqrt(|cmd|) - prev_setpoint)
                                m_actuator_control.actuator_setpoint[idx] += tau * (std::sqrt(std::abs(cmd)) - m_actuator_control.actuator_setpoint[idx]);
                                float throttle = m_actuator_control.actuator_setpoint[idx];
                                float rotor_velocity = dir[idx] * omega * throttle;
                                if (throttle > 5 * parameters.params_std_dev)
                                {
                                    // ignore velocity update if throttle is too small
                                    setRotorVelocity(rotor_velocity, idx);
                                }

                                float throttle_squared = std::pow(throttle, 2.0f);
                                float rotor_thrust = throttle_squared * std::abs(kf_normal_dist(rng) * omega_squared);
                                if (throttle > 5 * parameters.params_std_dev)
                                {
                                    // ignore thrust update if throttle is too small
                                    setRotorForce(rotor_thrust, idx);
                                }

                                float rotor_moment = -dir[idx] * throttle_squared * std::abs(km_normal_dist(rng) * omega_squared);
                                usdrt::GfVec3f torque_axis(0.0f, 0.0f, 0.0f);
                                getRotorTorqueAxis(rotor_moment, torque_axis, idx);
                                rotor_torque[idx] = torque_axis;
                            }
                            else if (idx >= getRotorCount() && idx < getRotorCount() + getActuatorCount())
                            {
                                // update for generic actuators (s-curve)
                                // current_setpoint = lerp(prev_setpoint, target_setpoint, smoothstep)
                                float smoothstep = parameters.servo_time_constants * parameters.servo_time_constants * (3 - 2 * parameters.servo_time_constants);
                                m_actuator_control.actuator_setpoint[idx] = lerp(m_actuator_control.actuator_setpoint[idx], parameters.max_angle * cmd, smoothstep);
                                setActuatorPosition(m_actuator_control.actuator_setpoint[idx], idx - getRotorCount());
                                setActuatorForce(parameters.max_torque, idx - getRotorCount());
                            }
                        });

        usdrt::GfVec3f velocity(0.0f, 0.0f, 0.0f);
        getVehicleVelocity(velocity);
        usdrt::GfMatrix3f rotMatrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        getVehicleRotationMatrix(rotMatrix);

        usdrt::GfVec3f body_drag = -1.0f * parameters.airframe_drag * velocity.GetLength() * velocity;
        usdrt::GfVec3f rotor_drag = -1.0f * rotMatrix * parameters.rotor_drag_coef * rotMatrix.GetTranspose() * velocity.GetLength();
        /** @note
        / We are applying additional body drag assuming similar reference area in X,Y plane and first-order rotor drag to the vehicle base link.
        / The collective thrusts are applied to center of mass of the vehicle and the gravity is already acting on the rigid body.
        / see https://discuss.px4.io/t/wind-estimator-tuning-equation/28053 and https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf
        / Should consider https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/nodes/omni-physx-forcefields/omni-physx-forcefields-forcefieldwind-1.html?
        / Global downwash and equilibrium actuator action acting on main body is not taken into account, assuming negligible.
        */
        usdrt::GfVec3f force_ = body_drag + rotor_drag;
        usdrt::GfVec3f torque_ = std::reduce(rotor_torque.begin(), rotor_torque.end());

        setVehicleForceAndTorque(force_, torque_, true);
    }

    const std::vector<int> getAirframeConfig(const std::string &airframe)
    {
        // https://docs.px4.io/main/en/airframes/airframe_reference.html
        static const std::unordered_map<std::string, std::vector<int>> airframeConfigs = {
            // CCW = 1, CW = -1
            {"QuadX", {1, 1, -1, -1}},
            {"HexX", {-1, 1, -1, 1, 1, -1}},
            {"OctX", {-1, -1, 1, 1, 1, 1, -1, -1}}};

        auto it = airframeConfigs.find(airframe);
        if (it != airframeConfigs.end())
        {
            return it->second;
        }

        return {}; // Return empty vector if not found
    }

private:
    VehicleParameters parameters;
    usdrt::UsdPrim vehicle_base_link;
    usdrt::UsdPrim vehicle_articulation_root;
    std::vector<usdrt::UsdPrim> rotor_joint_group;
    std::vector<usdrt::UsdPrim> rotor_body_group;
    std::vector<usdrt::UsdPrim> actuator_joint_group;
    std::vector<usdrt::UsdPrim> actuator_body_group;

    /**
     * Initializes the vehicle by setting up force API, mass, inertia, and rotor groups.
     *
     * @return True if the rotor joint group is not empty and matches the
     *         size of the rotor body group, otherwise false.
     */
    bool initVehicle(void)
    {
        setForceAPI(vehicle_base_link, true);
        setMassAndInertia();
        setRotorActuatorGroup();

        return !rotor_joint_group.empty() &&
                rotor_joint_group.size() == rotor_body_group.size() &&
                actuator_joint_group.size() == actuator_body_group.size();
    }

    /**
     * Populates the rotor/actuator joint and body groups by iterating over the children
     * of both the vehicle base link and vehicle articulation root that matches the
     * regex pattern and the corresponding USD type (RevoluteJoint / RigidBody / Deformable)
     */
    void setRotorActuatorGroup(void)
    {
        if (!vehicle_base_link.IsValid() || !vehicle_articulation_root.IsValid())
            return;

        rotor_joint_group.clear();
        rotor_body_group.clear();
        actuator_joint_group.clear();
        actuator_body_group.clear();

        std::regex rotor_pattern("[rR]otor[_]?.*");
        std::regex actuator_pattern("[aA]ctuator[_]?.*");
        auto processChild = [&](const usdrt::UsdPrim &child)
        {
            if (std::regex_match(child.GetName().GetText(), rotor_pattern))
            {
                if (child.IsA(usdrt::UsdPhysicsRevoluteJoint::_GetStaticTfType()))
                {
                    rotor_joint_group.emplace_back(child);
                }
                if (child.HasAPI(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType()))
                {
                    rotor_body_group.emplace_back(child);
                    setForceAPI(child);
                }
            }
            if (std::regex_match(child.GetName().GetText(), actuator_pattern))
            {
                if (child.IsA(usdrt::UsdPhysicsRevoluteJoint::_GetStaticTfType()))
                {
                    actuator_joint_group.emplace_back(child);
                }
                if (child.HasAPI(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType()))
                {
                    actuator_body_group.emplace_back(child);
                    setForceAPI(child);
                }
                if (child.HasAPI(usdrt::PhysxSchemaPhysxDeformableAPI::_GetStaticTfType()))
                {
                    // soft body actuator
                    actuator_body_group.emplace_back(child);
                }
            }
        };

        for (const auto &child : vehicle_base_link.GetAllChildren())
        {
            processChild(child);
        }
        for (const auto &child : vehicle_articulation_root.GetAllChildren())
        {
            processChild(child);
        }
    }

    /**
     * Populates the vehicle's mass, center of mass, and diagonal inertia values (ixx, iyy, izz)
     * assuming that the MassAPI is already present alongside the RigidBodyAPI.
     */
    void setMassAndInertia(void)
    {
        if (!vehicle_base_link.IsValid())
            return;

        usdrt::TfToken mass = usdrt::TfToken("physics:mass");
        if (vehicle_base_link.HasAttribute(mass))
        {
            vehicle_base_link.GetAttribute(mass).Get<float>(&parameters.mass);
        }

        usdrt::TfToken com = usdrt::TfToken("physics:center_of_mass");
        if (vehicle_base_link.HasAttribute(com))
        {
            vehicle_base_link.GetAttribute(com).Get<usdrt::GfVec3f>(&parameters.center_of_mass);
        }

        usdrt::TfToken inertia = usdrt::TfToken("physics:diagonalInertia");
        if (vehicle_base_link.HasAttribute(inertia))
        {
            vehicle_base_link.GetAttribute(inertia).Get<usdrt::GfVec3f>(&parameters.diagonal_inertias);
        }
    }

    void setForceAPI(const usdrt::UsdPrim &prim, bool isGlobal = false)
    {
        if (!prim.HasAPI(usdrt::PhysxSchemaPhysxForceAPI::_GetStaticTfType()))
        {
            usdrt::PhysxSchemaPhysxForceAPI::Apply(prim);
        }

        usdrt::PhysxSchemaPhysxForceAPI forceSchema(prim);

        if (!prim.HasAttribute(usdrt::TfToken("physxForce:force")))
        {
            forceSchema.CreateForceAttr().Set(usdrt::GfVec3f(0, 0, 0));
        }
        if (!prim.HasAttribute(usdrt::TfToken("physxForce:torque")))
        {
            forceSchema.CreateTorqueAttr().Set(usdrt::GfVec3f(0, 0, 0));
        }
        if (!prim.HasAttribute(usdrt::TfToken("physxForce:forceEnabled")))
        {
            forceSchema.CreateForceEnabledAttr().Set(true);
        }
        if (!prim.HasAttribute(usdrt::TfToken("physxForce:worldFrameEnabled")))
        {
            forceSchema.CreateWorldFrameEnabledAttr().Set(isGlobal);
        }
        if (!prim.HasAttribute(usdrt::TfToken("physxForce:mode")))
        {
            forceSchema.CreateModeAttr().Set(usdrt::TfToken("force"));
        }
    }

    void getVehicleVelocity(usdrt::GfVec3f &velocity)
    {
        if (!vehicle_base_link.IsValid())
            return;

        vehicle_base_link.GetAttribute(usdrt::TfToken("physics:velocity")).Get<usdrt::GfVec3f>(&velocity);
    }

    void getVehicleRotationMatrix(usdrt::GfMatrix3f &rotMat)
    {
        if (!vehicle_base_link.IsValid())
            return;

        usdrt::GfQuatd orient(0, 0, 0, 0);
        vehicle_base_link.GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&orient);
        rotMat.SetRotate(orient);
    }

    void setRotorVelocity(const float velocity, const int idx = 0)
    {
        if (rotor_joint_group.empty() || idx < 0 || idx >= static_cast<int>(rotor_joint_group.size()))
            return;
        // rotor_joint_group[idx].GetAttribute(usdrt::TfToken("drive:angular:physics:targetVelocity")).Set(velocity * RAD2DEG);
        rotor_joint_group[idx].GetAttribute(usdrt::TfToken("state:angular:physics:velocity")).Set(velocity * RAD2DEG);
    }

    void setRotorForce(const float thrust, const int idx = 0)
    {
        if (rotor_body_group.empty() || idx < 0 || idx >= static_cast<int>(rotor_body_group.size()))
            return;

        usdrt::GfVec3f force(0, 0, thrust);
        rotor_body_group[idx].GetAttribute(usdrt::TfToken("physxForce:force")).Set(force);
    }

    void setActuatorPosition(const float position, const int idx = 0)
    {
        if (actuator_joint_group.empty() || idx < 0 || idx >= static_cast<int>(actuator_joint_group.size()))
            return;

        // actuator_joint_group[idx].GetAttribute(usdrt::TfToken("drive:angular:physics:targetPosition")).Set(position);
        actuator_joint_group[idx].GetAttribute(usdrt::TfToken("state:angular:physics:position")).Set(position);
    }

    void setActuatorForce(const float force, const int idx = 0)
    {
        if (actuator_body_group.empty() || idx < 0 || idx >= static_cast<int>(actuator_body_group.size()))
            return;

        usdrt::GfVec3f applied_force(force, 0, 0);
        actuator_body_group[idx].GetAttribute(usdrt::TfToken("physxForce:force")).Set(applied_force);
    }

    void setVehicleForceAndTorque(const usdrt::GfVec3f &force, const usdrt::GfVec3f &torque, const bool isGlobal = true)
    {
        if (!vehicle_base_link.IsValid())
            return;

        vehicle_base_link.GetAttribute(usdrt::TfToken("physxForce:worldFrameEnabled")).Set(isGlobal);
        vehicle_base_link.GetAttribute(usdrt::TfToken("physxForce:force")).Set(force);
        vehicle_base_link.GetAttribute(usdrt::TfToken("physxForce:torque")).Set(torque);
    }
};

}  // px4
}  // isaac
}  // uosm