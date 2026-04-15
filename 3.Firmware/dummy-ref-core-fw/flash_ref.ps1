param(
    [string]$BuildDir = "build",
    [string]$Target = "SWD"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Find-CubeProgrammerCli {
    $candidates = @(
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\Program Files\STMicroelectronics\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    )

    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    $searchRoots = @(
        "C:\Program Files\STMicroelectronics",
        "C:\Program Files (x86)\STMicroelectronics"
    )

    foreach ($root in $searchRoots) {
        if (-not (Test-Path $root)) {
            continue
        }

        $match = Get-ChildItem $root -Recurse -File -Filter "STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue |
            Select-Object -First 1 -ExpandProperty FullName
        if ($match) {
            return $match
        }
    }

    return $null
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$elfPath = Join-Path $scriptDir "$BuildDir\Core-STM32F4-fw.elf"

if (-not (Test-Path $elfPath)) {
    throw "Firmware image not found: $elfPath`nRun .\build_ref.ps1 first."
}

$programmerCli = Find-CubeProgrammerCli
if (-not $programmerCli) {
    throw "STM32CubeProgrammer CLI not found.`nInstall STM32CubeProgrammer from https://www.st.com/en/development-tools/stm32cubeprog.html#get-software and then rerun this script."
}

Write-Host "Using STM32CubeProgrammer CLI: $programmerCli"
Write-Host "Flashing: $elfPath"

& $programmerCli -c port=$Target -d $elfPath -v -rst
if ($LASTEXITCODE -ne 0) {
    throw "Flashing failed."
}

Write-Host ""
Write-Host "Flash succeeded."
