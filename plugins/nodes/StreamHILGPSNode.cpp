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
#include <StreamHILGPSNodeDatabase.h>
#include <MavlinkServerManager.hpp>
#include <Px4SensorUtils.hpp>

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <carb/events/EventsUtils.h>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;
using omni::graph::core::AttributeRole;

namespace uosm {
namespace isaac {
namespace px4 {

class StreamHILGPSNode
{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj)
    {
        constexpr u_char kDefaultSystemId = 1;
        const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
        auto &state = StreamHILGPSNodeDatabase::sPerInstanceState<StreamHILGPSNode>(nodeObj, graphInstanceIndex);

        state.m_vehicle = std::make_unique<Px4Multirotor>();
        state.m_vehicle->reset();
        state.m_gps = std::make_unique<GpsSensor>();
        GpsSensor::GpsParameters &gps_params = state.m_gps->getParameters();
        // onValueChanged callback
        auto cb = [](const omni::graph::core::AttributeObj &attr, const void *value)
        {
            const NodeObj nodeObj = attr.iAttribute->getNode(attr);
            const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
            auto &state = StreamHILGPSNodeDatabase::sPerInstanceState<StreamHILGPSNode>(nodeObj, graphInstanceIndex);
            GpsSensor::GpsParameters &gps_params = state.m_gps->getParameters();
            if (attr.iAttribute->getNameToken(attr) == state::gpsXYNoiseDensity.token())
                gps_params.gps_xy_noise_density = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsZNoiseDensity.token())
                gps_params.gps_z_noise_density = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsVXYNoiseDensity.token())
                gps_params.gps_vxy_noise_density = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsVZNoiseDensity.token())
                gps_params.gps_vz_noise_density = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsXYRandomWalk.token())
                gps_params.gps_xy_random_walk = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsZRandomWalk.token())
                gps_params.gps_z_random_walk = *static_cast<const float *>(value);

            if (attr.iAttribute->getNameToken(attr) == state::gpsCorrelationTime.token())
                gps_params.gps_correlation_time = *static_cast<const float *>(value);
        };

        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::systemId.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eUChar, &kDefaultSystemId, 0);

        // Helper function to initialize attributes
        auto initializeAttribute = [&](const auto &token, omni::fabric::BaseDataType type, const void *defaultValue)
        {
            AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, token);
            attr.iAttribute->setDefaultValue(attr, type, defaultValue, 0);
            attr.iAttribute->registerValueChangedCallback(attr, cb, true);
        };

        // Initialize attributes
        initializeAttribute(state::gpsXYNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_xy_noise_density);
        initializeAttribute(state::gpsZNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_z_noise_density);
        initializeAttribute(state::gpsVXYNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_vxy_noise_density);
        initializeAttribute(state::gpsVZNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_vz_noise_density);
        initializeAttribute(state::gpsXYRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_xy_random_walk);
        initializeAttribute(state::gpsZRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_z_random_walk);
        initializeAttribute(state::gpsCorrelationTime.token(), omni::fabric::BaseDataType::eFloat, &gps_params.gps_correlation_time);

        if (auto timeline = omni::timeline::getTimeline())
        {
            state.m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                timeline->getTimelineEventStream(),
                [&state](carb::events::IEvent *timelineEvent)
                {
                    if (static_cast<omni::timeline::TimelineEventType>(timelineEvent->type) ==
                        omni::timeline::TimelineEventType::eStop)
                    {
                        // Reset gps readings
                        state.gps_last_sampled_us = 0U;
                        state.m_gps->reset();
                        state.m_vehicle->reset();
                        CARB_LOG_INFO("Reset GPS readings");
                    }
                });
        }
    }

    static bool compute(StreamHILGPSNodeDatabase &db)
    {
        const auto &time = db.inputs.time();
        const auto &home = db.inputs.homeCoordinate();
        const auto &px4_instance = db.inputs.px4Instance();
        const auto &vehicle_in = db.inputs.vehiclePrim();

        // Sanitize inputs
        if (vehicle_in.empty())
        {
            CARB_LOG_WARN("No vehicle input at %s", db.abi_node().iNode->getPrimPath(db.abi_node()));
            return false;
        }
        if (time <= 0)
            return false;

        auto &state = db.internalState<StreamHILGPSNode>();
        auto &server = MavlinkServerManager::getInstance();
        if (!server.isConnected(px4_instance))
            return false;

        omni::fabric::PathC vehiclePath = vehicle_in[0];
        if (!state.m_vehicle->isVehiclePathEqual(vehiclePath))
        {
            if (!state.m_vehicle->loadVehicle(vehiclePath))
                return false;
        }

        Px4Multirotor::VechicleMotion motion;
        state.m_vehicle->getVehicleMotion(motion);
        const usdrt::GfVec3f home_coordinates = usdrt::GfVec3f(home[0], home[1], home[2]);
        uint64_t current_time_us = static_cast<uint64_t>(time * 1e6);
        double dt = (current_time_us - state.gps_last_sampled_us) * 1e-6;
        state.gps_last_sampled_us = current_time_us;

        state.m_gps->sample(motion, home_coordinates, dt);
        const auto gps_data = state.m_gps->getGpsReadings();

        const int32_t lat = static_cast<int32_t>(gps_data.gps_latlonalt[0] * 1e7);  // degE7
        const int32_t lon = static_cast<int32_t>(gps_data.gps_latlonalt[1] * 1e7);  // degE7
        const int32_t alt = static_cast<int32_t>(gps_data.gps_latlonalt[2] * 1000); // mm

        const uint16_t vel = static_cast<uint16_t>(std::sqrt(gps_data.gps_vel_enu[0] * gps_data.gps_vel_enu[0] +
                                                             gps_data.gps_vel_enu[1] * gps_data.gps_vel_enu[1]) *
                                                   100);                         // cm/s
        const int16_t vn = static_cast<int16_t>(gps_data.gps_vel_enu[1] * 100);  // cm/s
        const int16_t ve = static_cast<int16_t>(gps_data.gps_vel_enu[0] * 100);  // cm/s
        const int16_t vd = static_cast<int16_t>(-gps_data.gps_vel_enu[2] * 100); // cm/s

        // Ensure angle is in [0, 360) range
        uint16_t cog_degrees = static_cast<uint16_t>(std::atan2(gps_data.gps_vel_enu[0], gps_data.gps_vel_enu[1]) * 180.0f / M_PI * 100); // centi-deg
        if (cog_degrees < 0)
            cog_degrees += 36000;
        const uint16_t cog = vel > 10 ? cog_degrees : UINT16_MAX;

        int16_t yaw = static_cast<int16_t>(std::round(getYawfromQuat(motion.orient) * 180.0f / M_PI * 100)); // centi-deg
        if (yaw < 0)
            yaw += 36000;

        mavlink_message_t msg;
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        // https://mavlink.io/en/messages/common.html#HIL_GPS
        mavlink_msg_hil_gps_pack(db.state.systemId(), MAV_COMP_ID_GPS, &msg,
                                 current_time_us, // time_usec
                                 (uint8_t)3,      // fix_type (3 = 3D Fix)
                                 lat,             // lat
                                 lon,             // lon
                                 alt,             // alt
                                 (uint16_t)1,     // eph (between 1-2)
                                 (uint16_t)1,     // epv (between 1-2)
                                 vel,             // vel
                                 vn,              // vn
                                 ve,              // ve
                                 vd,              // vd
                                 cog,             // cog
                                 (uint8_t)12,     // satellites_visible (12 = RTK quality)
                                 0,               // id
                                 yaw              // yaw
        );
        int len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
        return server.send(px4_instance, buffer.data(), len);
    }

private:
    carb::ObjectPtr<carb::events::ISubscription> m_timelineEventsSubscription;
    std::unique_ptr<Px4Multirotor> m_vehicle;

    uint64_t gps_last_sampled_us = 0U;
    std::unique_ptr<GpsSensor> m_gps;
};

// Following allow visibility of node to omnigraph
REGISTER_OGN_NODE()

} // px4
} // isaac
} // uosm