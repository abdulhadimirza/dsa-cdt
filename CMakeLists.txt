cmake_minimum_required(VERSION 3.24)
project(RaylibImGuiApp LANGUAGES CXX)

# Force modern C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensures strict ISO C++23 without non-standard compiler extensions

if(WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static -static-libgcc -static-libstdc++")
endif()

include(FetchContent)

# 1. Fetch Raylib
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib
    GIT_TAG        master # Or pin to a specific release like "5.5"
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # Don't build raylib examples
FetchContent_MakeAvailable(raylib)

# 2. Fetch Dear ImGui (Docking Branch is highly recommended)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui
    GIT_TAG        docking # Allows dockable windows
)
FetchContent_MakeAvailable(imgui)

# 3. Fetch rlImGui (The glue layer bridging raylib & imgui)
FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui
    GIT_TAG        main
)
FetchContent_MakeAvailable(rlimgui)

# Define our executable target
add_executable(${PROJECT_NAME} 
    src/main.cpp
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)

# 1. Explicitly tell the compiler where to find headers
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${imgui_SOURCE_DIR}
    ${rlimgui_SOURCE_DIR}
)

# 2. FIX: Link using the actual case-sensitive target names
# Note the capital 'I' and 'G' in rlImGui!
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)


# Setup runtime copy rules if you have assets (Images, Fonts, etc.)
# target_include_directories(${PROJECT_NAME} PRIVATE src)
