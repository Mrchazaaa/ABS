# ABS

[![Build](https://github.com/Mrchazaaa/ABS/actions/workflows/build.yml/badge.svg?branch=master)](https://github.com/Mrchazaaa/ABS/actions/workflows/build.yml)
[![Tests](https://github.com/Mrchazaaa/ABS/actions/workflows/test.yml/badge.svg?branch=master)](https://github.com/Mrchazaaa/ABS/actions/workflows/test.yml)

Standalone ABS controller library extracted from the Charlie Robot Speed Dreams driver.

It exposes a small C API for:

- creating and resetting controller state
- stepping the controller from wheel-speed inputs
- retrieving brake-command outputs and debug telemetry

## Install

Clone the repository with its submodules:

```bash
git clone --recurse-submodules https://github.com/Mrchazaaa/ABS.git
cd ABS
```

If you already cloned the repository without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

## Build

Configure and build with CMake:

```bash
cmake -S . -B build
cmake --build build
```

This produces the static library in `build/`.

## Test

Configure and build the test target with CMake, then run the suite with CTest:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
