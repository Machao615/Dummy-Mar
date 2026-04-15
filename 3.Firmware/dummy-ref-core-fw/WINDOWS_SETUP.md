# REF Controller Windows Environment

This project can be built on Windows. Ubuntu is not required.

## Installed compile toolchain

The machine is already configured with:

- Kitware CMake
- Ninja
- GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)

## Build

From this directory:

```powershell
.\build_ref.ps1
```

Artifacts are generated in:

```text
build\Core-STM32F4-fw.elf
build\Core-STM32F4-fw.hex
build\Core-STM32F4-fw.bin
```

## Flash

Install STM32CubeProgrammer from the official ST page:

[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html#get-software)

After installation, connect the board with ST-Link over SWD and run:

```powershell
.\flash_ref.ps1
```

## Notes

- Target MCU: `STM32F405RGTx`
- Default flash transport in the script: `SWD`
- If CMake is resolved to the Python package version, `build_ref.ps1` prefers the Kitware install at `C:\Program Files\CMake\bin\cmake.exe`
