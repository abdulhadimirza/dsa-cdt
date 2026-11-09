# Constrained Delaunay Triangulation and Navigation

This project is an interactive 2D geometry editor. Draw boundaries and obstacles, validate the geometry, generate a navigation mesh, and find a path between selected start and goal points. Maps can be saved and loaded from the control panel.

## Windows setup

These instructions match the project’s current Windows toolchain: MSYS2 UCRT64, GCC, Ninja, and VS Code with CMake Tools.

### Prerequisites

- [Git for Windows](https://git-scm.com/download/win)
- [MSYS2](https://www.msys2.org/)
- [Visual Studio Code](https://code.visualstudio.com/)
- VS Code extensions: **CMake Tools** (`ms-vscode.cmake-tools`) and **C/C++** (`ms-vscode.cpptools`)
- An internet connection for the first CMake configuration, which fetches Raylib, Dear ImGui, rlImGui, and Clipper2 from GitHub

In the **MSYS2 UCRT64** terminal, update MSYS2 and install the compiler and build tools:

```sh
pacman -Syu
```

If the update asks you to close the terminal, reopen the **MSYS2 UCRT64** terminal and run the update command again. Then install the toolchain:

```sh
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja
```

Make sure `C:\msys64\ucrt64\bin` is on Windows `PATH` so VS Code can find `g++`, CMake, and Ninja. The project requires CMake 3.24 or newer and C++23 support.

### Clone and run in VS Code

1. Clone the repository and open its root folder in VS Code:

   ```powershell
   git clone <repository-url>
   cd dsa-cdt
   code .
   ```

2. If prompted, allow CMake Tools to configure the project. The checked-in workspace settings select Ninja and configure on open. On first configuration, CMake downloads the external dependencies listed above.
3. If CMake Tools asks you to select a kit, choose the MSYS2 UCRT64 GCC kit. Confirm that the selected compiler is the `g++.exe` under `C:\msys64\ucrt64\bin`.
4. To launch the app, press **Ctrl+Shift+P**, run **CMake: Debug**, and press **Enter**. CMake Tools will build and start the application with the debugger.

The workspace files `.vscode/settings.json` and `.vscode/c_cpp_properties.json` provide the Ninja/CMake Tools settings and C++23 IntelliSense configuration. They are included in the repository; the compiler must still be installed on each machine.

### PowerShell command-line fallback

From the repository root, configure, build, and run the Debug executable:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build
.\build\RaylibImGuiApp.exe
```

These commands require the MSYS2 UCRT64 `bin` directory on `PATH`. The first configure step needs internet access to fetch dependencies. Build output is written to `build/`.
