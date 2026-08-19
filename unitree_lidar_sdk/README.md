# Unitree LiDAR L2 C++ SDK

The SDK ships pre-built static archives for x86_64 and aarch64.

```bash
cmake -S . -B build
cmake --build build
cmake --install build --prefix /desired/prefix
```

The installation provides a relocatable CMake package:

```cmake
find_package(unilidar_sdk2 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE unilidar::sdk2)
```

Example programs are disabled for library consumers by default. Build them with
`-DUNILIDAR_BUILD_EXAMPLES=ON`.
