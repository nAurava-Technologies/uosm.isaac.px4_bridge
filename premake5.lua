-- Setup the basic extension information.
local ext = get_current_extension_info()
local script_path = os.getcwd()
local project_name = "uosm/isaac/px4_bridge"

-- Function to check if path exists
local function path_exists(path)
    local ok, err, code = os.rename(path, path)
    return ok ~= nil or code == 13
end

-- Function to create directory if it doesn't exist
local function ensure_directory(path)
    if not path_exists(path) then
        os.execute("mkdir -p " .. path)
    end
end

-- Function to execute shell command and return output
local function execute_command(cmd)
    local handle = io.popen(cmd)
    local result = handle:read("*a")
    handle:close()
    return result
end

-- Setup function for Micro-XRCE-DDS-Agent
local function setup_xrce()
    local build_dir = script_path.."/third_party/Micro-XRCE-DDS-Agent/build"
    ensure_directory(build_dir)
    local binary_path = build_dir.."/MicroXRCEAgent"

    -- Base install directory
    local install_base = build_dir.."/temp_install"

    -- If binary doesn't exist, build the agent
    if not path_exists(binary_path) then
        print("Building Micro-XRCE-DDS-Agent...")
        os.execute("cd "..script_path.."/third_party/Micro-XRCE-DDS-Agent && cmake -B build && cmake --build build -j$(nproc)")

        -- Set RPATH to temporary install directory
        local rpath = "$ORIGIN:$ORIGIN/temp_install/fastdds-3.1/lib:$ORIGIN/temp_install/fastcdr-2.2.4/lib:$ORIGIN/temp_install/microxrcedds_client-3.0.0/lib:$ORIGIN/temp_install/microcdr-2.0.1/lib"
        os.execute(string.format("patchelf --set-rpath '%s' %s", rpath, binary_path))
    else
        print("Micro-XRCE-DDS-Agent already built")
    end
end

-- Setup function for PX4 SITL
local function setup_px4()
    local build_dir = script_path.."/third_party/PX4-Autopilot/build"
    ensure_directory(build_dir)
    local binary_path = build_dir.."/px4_sitl_default/bin/px4"

    if not path_exists(binary_path) then
        print("Building PX4 SITL...")
        -- Add the specific Python path for module installation
        os.execute("export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH")
        os.execute("export PATH=$PATH:$HOME/.local/bin")

        -- Set the environment for CMake
        os.execute("export CMAKE_PREFIX_PATH=$HOME/.local/lib/python3.10/site-packages:$CMAKE_PREFIX_PATH")

        os.execute("pip3 install --user pyyaml empy")
        os.execute("pip3 install --user kconfiglib")
        os.execute("cd "..script_path.."/third_party/PX4-Autopilot && " ..
            "pip3 install --user -r ./Tools/setup/optional-requirements.txt && " ..
            "./Tools/setup/ubuntu.sh --no-nutxx --no-sim-tools && " ..
            "PYTHONPATH=$HOME/.local/lib/python3.10/site-packages make px4_sitl_default -j$(nproc)" )
    else
        print("PX4 SITL already built")
    end
end

-- Run initial setup before project configuration
setup_xrce()
setup_px4()

-- Project configuration
project_ext(ext)

-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, project_name)

-- --------------------------------------------------------------------------------------------------------------
-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "include", ext.target_dir.."/include" },
    { "third_party/Micro-XRCE-DDS-Agent/build", ext.target_dir.."/uxrce" },
    { "third_party/PX4-Autopilot/build/px4_sitl_default", ext.target_dir.."/px4" }
}

-- --------------------------------------------------------------------------------------------------------------
-- Copy the __init__.py to allow building of a non-linked ogn/ import directory.
repo_build.prebuild_copy {
    { project_name.."/__init__.py", ogn.python_target_path }
}

-- --------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn( ext, ogn )

-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)
    -- It is important that you add all subdirectories containing C++ code to this project
    add_files("source", "plugins/"..ogn.module)
    add_files("nodes", "plugins/nodes")

    -- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
    add_ogn_dependencies(ogn)

    -- Begin OpenUSD
    extra_usd_libs = {
        "usdPhysics",
        "usdUtils",
    }

    add_usd(extra_usd_libs)

    includedirs {
        ext.target_dir.."/include",
        ext.target_dir.."/third_party/PX4-Autopilot/build/px4_sitl_default/mavlink",
        "%{target_deps}/cuda",
        "%{target_deps}/pybind11/include",
        "%{target_deps}/python/include",
        "%{kit_sdk}/dev/fabric/include",
        "%{kit_sdk}/dev/include",
    }
    libdirs {
        "%{target_deps}/python/libs",
        "%{target_deps}/cuda/lib/x64"
    }
    links {

    }

    defines { "NOMINMAX", "NDEBUG" }
    cppdialect "C++17"
    runtime "Release"
    rtti "On"

    filter { "system:linux" }
        exceptionhandling "On"
        staticruntime "Off"
        includedirs {
            "%{target_deps}/python/include/python3.10",
            "%{target_deps}/cuda"
        }
        -- NOTE: std::string not ABI safe in omni/carbonite context
        -- see https://docs.omniverse.nvidia.com/kit/docs/carbonite/latest/docs/omni/String.html
        buildoptions { "-D_GLIBCXX_USE_CXX11_ABI=0 -pthread -lstdc++fs -Wno-error -fabi-version=11" }
        linkoptions { "-Wl,--disable-new-dtags -Wl,-rpath,%{target_deps}/python/lib:" }