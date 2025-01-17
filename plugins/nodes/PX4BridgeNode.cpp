/****************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2025 limshoonkit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ****************************************************************************/
#include <PX4BridgeNodeDatabase.h>
#include <MavlinkServerManager.hpp>
#include <Px4ProcessManager.hpp>
#include <Px4VehicleUtils.hpp>

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <carb/events/EventsUtils.h>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace uosm {
namespace isaac {
namespace px4 {

class PX4BridgeNode
{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj)
    {
        const omni::fabric::Token kDefaultSimModel("isaacsim_s500");

        const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
        auto &state = PX4BridgeNodeDatabase::sPerInstanceState<PX4BridgeNode>(nodeObj, graphInstanceIndex);

        state.m_vehicle = std::make_unique<Px4Multirotor>();
        state.m_vehicle->reset();
        // onValueChanged callback
        auto cb = [](const omni::graph::core::AttributeObj &attr, const void *value)
        {
            const NodeObj nodeObj = attr.iAttribute->getNode(attr);
            const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
            auto &state = PX4BridgeNodeDatabase::sPerInstanceState<PX4BridgeNode>(nodeObj, graphInstanceIndex);
            Px4Multirotor::VehicleParameters &vehicle_params = state.m_vehicle->getParameters();
            if (attr.iAttribute->getNameToken(attr) == state::km.token())
                vehicle_params.moment_constants = *static_cast<const float *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::kf.token())
                vehicle_params.force_constants = *static_cast<const float *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::maxVel.token())
                vehicle_params.max_rotor_velocity = *static_cast<const float *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::stdDev.token())
                vehicle_params.params_std_dev = *static_cast<const float *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::tau.token())
                vehicle_params.time_constants = *static_cast<const float *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::rotorDrag.token())
                vehicle_params.rotor_drag_coef = *static_cast<const usdrt::GfVec3f *>(value);
            if (attr.iAttribute->getNameToken(attr) == state::airframeDrag.token())
                vehicle_params.airframe_drag = *static_cast<const float *>(value);
        };

        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::simModel.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eToken, &kDefaultSimModel, 0);

        // Helper function to initialize attributes
        auto initializeAttribute = [&](const auto &token, omni::fabric::BaseDataType type, const void *defaultValue)
        {
            AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, token);
            attr.iAttribute->setDefaultValue(attr, type, defaultValue, 0);
            attr.iAttribute->registerValueChangedCallback(attr, cb, true);
        };
        Px4Multirotor::VehicleParameters &vehicle_params = state.m_vehicle->getParameters();
        // Initialize attributes
        initializeAttribute(state::km.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.moment_constants);
        initializeAttribute(state::kf.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.force_constants);
        initializeAttribute(state::maxVel.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.max_rotor_velocity);
        initializeAttribute(state::stdDev.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.params_std_dev);
        initializeAttribute(state::tau.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.time_constants);
        initializeAttribute(state::rotorDrag.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.rotor_drag_coef);
        initializeAttribute(state::airframeDrag.token(), omni::fabric::BaseDataType::eFloat, &vehicle_params.airframe_drag);

        if (auto timeline = omni::timeline::getTimeline())
        {
            state.m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                timeline->getTimelineEventStream(),
                [&state](carb::events::IEvent *timelineEvent)
                {
                    auto &processManager = Px4ProcessManager::getInstance();
                    auto &serverManager = MavlinkServerManager::getInstance();
                    if (static_cast<omni::timeline::TimelineEventType>(timelineEvent->type) ==
                        omni::timeline::TimelineEventType::ePlay)
                    {
                        // Timeline started
                        if (!processManager.isUxRCEAgentRunning())
                        {
                            processManager.launch_uxrce_agent();
                        }
                    }
                    else if (static_cast<omni::timeline::TimelineEventType>(timelineEvent->type) ==
                             omni::timeline::TimelineEventType::eStop)
                    {
                        // Timeline stopped, terminate sessions
                        processManager.terminateAll();
                        serverManager.removeAllConnections();
                        state.m_vehicle->reset();
                        state.m_throttle.clear();
                        state.isPX4Running_ = false;
                    }
                });
        }
    }

    static bool compute(PX4BridgeNodeDatabase &db)
    {
        const auto &airframe = db.inputs.airframe();
        const auto &px4_instance = db.inputs.px4Instance();
        const auto &vehicle_in = db.inputs.vehiclePrim();
        if (vehicle_in.empty())
        {
            CARB_LOG_WARN("No vehicle input at %s", db.abi_node().iNode->getPrimPath(db.abi_node()));
            return false;
        }

        auto &state = db.internalState<PX4BridgeNode>();
        auto &processManager = Px4ProcessManager::getInstance();
        if(!processManager.isUxRCEAgentRunning())
        {
            CARB_LOG_WARN("UXCRE agent is not running, check installation or port 8888 in use!");
            return false;
        }

        auto &serverManager = MavlinkServerManager::getInstance();
        if (!state.isPX4Running_)
        {
            // Launch PX4 instance
            if (processManager.launch_px4(px4_instance, db.tokenToString(db.state.simModel())) == 0)
                return false;
            state.isPX4Running_ = true;
        }
        if (!serverManager.isConnected(px4_instance))
        {
            // Create server socket
            if (!serverManager.getServerSocket(px4_instance) &&
                !serverManager.createServerSocket(px4_instance))
                return false;
            // Accept server connection
            if (!serverManager.connect(px4_instance))
                return false;
        }

        omni::fabric::PathC vehiclePath = vehicle_in[0];
        if (!state.m_vehicle->isVehiclePathEqual(vehiclePath))
        {
            state.m_vehicle->loadVehicle(vehiclePath);
            state.m_throttle = std::vector<float>(state.m_vehicle->getRotorCount(), 0.0f);
            state.m_airframe = state.m_vehicle->getAirframeConfig(db.tokenToString(airframe));
            if (state.m_airframe.size() != state.m_vehicle->getRotorCount())
            {
                CARB_LOG_WARN("Airframe %s does not match rotor size = %ld!", db.tokenToString(airframe), state.m_vehicle->getRotorCount());
                return false;
            }
        }

        // Handle data exchange
        std::array<uint8_t, 1024> receive_buffer{};
        size_t bytes_received = 0;
        if (!serverManager.receive(px4_instance, receive_buffer.data(), receive_buffer.size(), bytes_received))
        {
            CARB_LOG_ERROR("Failed to receive data from PX4 instance %d", px4_instance);
            return false;
        }

        std::vector<float> actuator_control(state.m_vehicle->getRotorCount(), 0.0f);
        state.parse_mavlink_message(receive_buffer.data(), bytes_received, actuator_control);
        state.m_vehicle->computeDynamics(actuator_control, state.m_throttle, state.m_airframe);
        return true;
    }

private:
    carb::ObjectPtr<carb::events::ISubscription> m_timelineEventsSubscription;
    std::unique_ptr<Px4Multirotor> m_vehicle;
    std::atomic<bool> isPX4Running_{false};
    std::vector<float> m_throttle;
    std::vector<int> m_airframe;

    void parse_mavlink_message(const uint8_t *buffer, size_t length, std::vector<float> &control)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (size_t i = 0; i < length; i++)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
            {
                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    // https://mavlink.io/en/messages/common.html#HEARTBEAT
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    printf("HEARTBEAT - System type: %d, Autopilot: %d\n", (int)heartbeat.type, (int)heartbeat.autopilot);
                    break;
                }
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    // https://mavlink.io/en/messages/common.html#SYS_STATUS
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    printf("SYS_STATUS - Battery remaining: %d\n", sys_status.battery_remaining);
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG:
                {
                    // https://mavlink.io/en/messages/common.html#COMMAND_LONG
                    mavlink_command_long_t command_long;
                    mavlink_msg_command_long_decode(&msg, &command_long);
                    printf("COMMAND_LONG - Command: %d, Confirmation: %d, Target system: %d, Target component: %d, Params: %f, %f, %f, %f, %f, %f, %f\n",
                           (int)command_long.command, (int)command_long.confirmation,
                           (int)command_long.target_system, (int)command_long.target_component,
                           command_long.param1, command_long.param2, command_long.param3, command_long.param4, command_long.param5, command_long.param6, command_long.param7);
                    break;
                }
                case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                {
                    // https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
                    mavlink_hil_actuator_controls_t actuator_controls;
                    mavlink_msg_hil_actuator_controls_decode(&msg, &actuator_controls);
                    std::copy_n(actuator_controls.controls, control.size(), control.begin());
                    break;
                }
                default:
                    CARB_LOG_WARN("Unknown MAVLink message received! id = %d", msg.msgid);
                    break;
                }
            }
        }
    }
};

// Following allow visibility of node to omnigraph
REGISTER_OGN_NODE()

} // px4
} // isaac
} // uosm