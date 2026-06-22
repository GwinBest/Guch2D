# Guch2D

`Guch2D` is a small C++23 2D physics engine focused on collision handling,
rigid-body dynamics, and solver experiments. The repository also contains an
SFML demo application and a GoogleTest test suite.

## What Is Inside

| Path | Purpose |
| --- | --- |
| `Source/` | Engine library sources: collision, dynamics, solvers, math, utilities |
| `Demo/` | SFML-based demo executable |
| `Test/` | GoogleTest unit tests |

Main CMake targets:

| Target | Description |
| --- | --- |
| `Guch2D` | Static/library target for the engine |
| `Guch2D-Demo` | Demo executable, enabled by `BUILD_DEMO=ON` |
| `Guch2D-Test` | Test executable, enabled by `BUILD_TEST=ON` |
| `format` | Applies `clang-format`, if formatting support is enabled |
| `format-check` | Checks formatting without modifying files |

## Requirements

- CMake 3.19 or newer
- A C++23-capable compiler
- Git, used by CMake `FetchContent`
- Internet access on the first configure, so CMake can fetch dependencies

Fetched dependencies:

| Dependency | Used For |
| --- | --- |
| `glm` | Math support |
| `SFML 3.0.2` | Demo application |
| `GoogleTest 1.17.0` | Unit tests |

## Presets

The recommended workflow is to configure through CMake presets and then build
the generated build directory.

| Preset | Build Directory | Type | Demo | Tests |
| --- | --- | --- | --- | --- |
| `debug` | `Build/Debug` | `Debug` | on | on |
| `release` | `Build/Release` | `Release` | on | off |
| `relwithdebinfo` | `Build/RelWithDebInfo` | `RelWithDebInfo` | on | off |

List available presets:

```bash
cmake --list-presets
```

## Quick Start

Configure and build the debug preset:

```bash
cmake --preset debug
cmake --build Build/Debug
```

Run tests:

```bash
ctest --test-dir Build/Debug --output-on-failure
```

Run the demo after building:

```bash
./Build/Debug/Demo/Guch2D-Demo
```

On Windows, the executable usually has the `.exe` suffix.

## Common Builds

Debug build with demo and tests:

```bash
cmake --preset debug
cmake --build Build/Debug
ctest --test-dir Build/Debug --output-on-failure
```

Release build:

```bash
cmake --preset release
cmake --build Build/Release
```

Optimized build with debug symbols:

```bash
cmake --preset relwithdebinfo
cmake --build Build/RelWithDebInfo
```

Build a specific target:

```bash
cmake --build Build/Debug --target Guch2D
cmake --build Build/Debug --target Guch2D-Demo
cmake --build Build/Debug --target Guch2D-Test
```

## Tests

Tests are enabled by default in the `debug` preset.

```bash
cmake --preset debug
cmake --build Build/Debug --target Guch2D-Test
ctest --test-dir Build/Debug --output-on-failure
```

To enable tests for another preset, pass `BUILD_TEST=ON` during configure:

```bash
cmake --preset relwithdebinfo -DBUILD_TEST=ON
cmake --build Build/RelWithDebInfo --target Guch2D-Test
ctest --test-dir Build/RelWithDebInfo --output-on-failure
```

## Demo

The demo is enabled in all current presets.

```bash
cmake --preset debug
cmake --build Build/Debug --target Guch2D-Demo
./Build/Debug/Demo/Guch2D-Demo
```

For release:

```bash
cmake --preset release
cmake --build Build/Release --target Guch2D-Demo
./Build/Release/Demo/Guch2D-Demo
```

## CMake Options

| Option | Default | Description |
| --- | --- | --- |
| `BUILD_DEMO` | `OFF` | Build the SFML demo executable |
| `BUILD_TEST` | `OFF` | Build GoogleTest tests |
| `ENABLE_ASAN` | `ON` | Enable AddressSanitizer when supported |
| `ENABLE_UBSAN` | `ON` | Enable UndefinedBehaviorSanitizer when supported |
| `ENABLE_CLANG_TIDY` | `OFF` | Run `clang-tidy` on project targets |
| `ENABLE_CLANG_FORMAT` | `ON` | Add `format` and `format-check` helper targets |
| `ENABLE_LINT_STRICT` | `OFF` | Fail configure if requested lint tools are missing |

Sanitizer flags are applied to `Debug` and `RelWithDebInfo` builds when the
compiler and linker support them.

You can override any option while configuring a preset:

```bash
cmake --preset debug -DENABLE_CLANG_TIDY=ON
cmake --preset release -DBUILD_TEST=ON
cmake --preset relwithdebinfo -DENABLE_ASAN=OFF -DENABLE_UBSAN=OFF
```

## Formatting

If `clang-format` is available and `ENABLE_CLANG_FORMAT=ON`, CMake creates two
formatting targets:

```bash
cmake --preset debug
cmake --build Build/Debug --target format-check
cmake --build Build/Debug --target format
```

Use `format-check` in CI-style checks and `format` when you want CMake to update
source files in place.
