# Guch2D

`Guch2D` is a 2D physics engine in C++23.

Project layout:

- `Src/` — library source code (`Collision`, `Dynamics`, `Solver`, `Math`, `Utils`)
- `Test/` — GoogleTest unit tests
- `Demo/` — SFML demo application

## Requirements

- CMake 3.19+
- A C++ compiler with C++23 support
- Git (for `FetchContent`: `glm`, `googletest`, `SFML`)

## Quick Start

Build the library only:

```bash
cmake -S . -B build
cmake --build build
```

## Tests

```bash
cmake -S . -B build-test -DBUILD_TEST=ON
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

## Demo

```bash
cmake -S . -B build-demo -DBUILD_DEMO=ON
cmake --build build-demo
```

Demo executable: `build-demo/Demo/Guch2D-Demo` (or `Guch2D-Demo.exe` on Windows).

## Useful CMake Options

- `-DBUILD_TEST=ON` — build tests
- `-DBUILD_DEMO=ON` — build demo
- `-DENABLE_ASAN=ON|OFF` — AddressSanitizer (default: `ON`)
- `-DENABLE_UBSAN=ON|OFF` — UndefinedBehaviorSanitizer (default: `ON`)
- `-DENABLE_CLANG_TIDY=ON|OFF` — enable `clang-tidy`
- `-DENABLE_CLANG_FORMAT=ON|OFF` — add formatting targets
- `-DENABLE_LINT_STRICT=ON|OFF` — fail at configure time if lint tools are missing

## Formatting

If `ENABLE_CLANG_FORMAT` is enabled and `clang-format` is available in `PATH`:

```bash
cmake --build build --target format
cmake --build build --target format-check
```
