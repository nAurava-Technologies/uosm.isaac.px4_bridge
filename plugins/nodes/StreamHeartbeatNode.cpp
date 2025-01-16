/****************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2024 limshoonkit
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
#include <StreamHeartbeatNodeDatabase.h>
#include <MavlinkServerManager.hpp>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace uosm {
namespace isaac {
namespace px4 {

class StreamHeartbeatNode
{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj)
    {
        constexpr u_char kDefaultSystemId = 1;

        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::systemId.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eUChar, &kDefaultSystemId, 0);
    }

    static bool compute(StreamHeartbeatNodeDatabase &db)
    {
        const auto &px4_instance = db.inputs.px4Instance();
        auto &server = MavlinkServerManager::getInstance();
        if (!server.isConnected(px4_instance))
            return false;

        mavlink_message_t msg;
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        // https://mavlink.io/en/messages/common.html#HEARTBEAT
        mavlink_msg_heartbeat_pack(db.state.systemId(), MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
                                   MAV_TYPE_GENERIC_MULTIROTOR, // Type
                                   MAV_AUTOPILOT_GENERIC,       // Autopilot type
                                   MAV_MODE_AUTO_ARMED,         // System mode
                                   0,                           // Custom mode
                                   MAV_STATE_ACTIVE             // System state
        );
        int len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
        return server.send(px4_instance, buffer.data(), len);
    }

private:
};

// Following allow visibility of node to omnigraph
REGISTER_OGN_NODE()

} // px4
} // isaac
} // uosm