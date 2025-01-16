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
#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/kit/IApp.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>

#include <MavlinkServerManager.hpp>
#include <Px4ProcessManager.hpp>

const struct carb::PluginImplDesc pluginImplDesc = { "uosm.isaac.px4.plugin",
                                                     "A PX4 Bridge for Isaac Sim", "UoSM",
                                                     carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry,
                      omni::fabric::IPath,
                      omni::fabric::IToken,
                      omni::kit::IApp)

DECLARE_OGN_NODES()

namespace uosm {
namespace isaac {
namespace px4 {

class PX4OmingraphNodeExtension : public omni::ext::IExt
{
public:
    void onStartup(const char *extId) override
    {
        printf("PX4 bridge extension startup..... (ext_id: %s).\n", extId);
        INITIALIZE_OGN_NODES()
    }

    void onShutdown() override
    {
        printf("PX4 bridge extension shutdown\n");
        auto &processManager = Px4ProcessManager::getInstance();
        auto &serverManager = MavlinkServerManager::getInstance();
        processManager.terminateAll();
        serverManager.removeAllConnections();
        RELEASE_OGN_NODES()
    }

private:
};

} // px4
} // isaac
} // uosm


CARB_PLUGIN_IMPL(pluginImplDesc, uosm::isaac::px4::PX4OmingraphNodeExtension)

void fillInterface(uosm::isaac::px4::PX4OmingraphNodeExtension& iface)
{
}
