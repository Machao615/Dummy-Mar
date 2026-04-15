param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",
    [string]$BuildDir = "build"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Require-Command {
    param(
        [string]$Name,
        [string]$Hint
    )

    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if (-not $cmd) {
        throw "Missing required command: $Name. $Hint"
    }
    return $cmd.Source
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cmake = "C:\Program Files\CMake\bin\cmake.exe"
if (-not (Test-Path $cmake)) {
    $cmake = Require-Command "cmake" "Install Kitware CMake first."
}

$null = Require-Command "ninja" "Install Ninja first."
$null = Require-Command "arm-none-eabi-gcc" "Install GNU Arm Embedded Toolchain first."

Write-Host "Using CMake: $cmake"
Write-Host "Configuring: $BuildType"

$buildDirPath = Join-Path $scriptDir $BuildDir
$buildTypeArg = "-DCMAKE_BUILD_TYPE=$BuildType"

& $cmake -S $scriptDir -B $buildDirPath -G Ninja $buildTypeArg
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed."
}

& $cmake --build $buildDirPath -j
if ($LASTEXITCODE -ne 0) {
    throw "Build failed."
}

Write-Host ""
Write-Host "Build succeeded."
Write-Host "ELF: $(Join-Path $scriptDir "$BuildDir\Core-STM32F4-fw.elf")"
Write-Host "HEX: $(Join-Path $scriptDir "$BuildDir\Core-STM32F4-fw.hex")"
Write-Host "BIN: $(Join-Path $scriptDir "$BuildDir\Core-STM32F4-fw.bin")"
