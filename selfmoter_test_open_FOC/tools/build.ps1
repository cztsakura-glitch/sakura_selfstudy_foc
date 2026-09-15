param(
  [switch]$Clean,
  [switch]$Flash
)

$ErrorActionPreference = "Stop"

function Find-Tool {
  param([string[]]$Names)

  foreach ($name in $Names) {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) {
      return $cmd.Source
    }
  }

  return $null
}

$make = Find-Tool @("make", "mingw32-make")
$gcc = Find-Tool @("arm-none-eabi-gcc")
$missing = @()

if (-not $make) {
  $missing += "make/mingw32-make"
}

if (-not $gcc) {
  $missing += "arm-none-eabi-gcc"
}

if ($missing.Count -gt 0) {
  Write-Error ("Missing build tools: " + ($missing -join ", ") + ". Install STM32CubeCLT or GNU Make + Arm GNU Toolchain, then add them to PATH.")
}

$argsList = @()

if ($Clean) {
  $argsList += "clean"
} elseif ($Flash) {
  $openocd = Find-Tool @("openocd")
  if (-not $openocd) {
    Write-Error "Missing openocd. Install OpenOCD or use STM32CubeCLT/OpenOCD and add it to PATH."
  }
  $argsList += "flash"
} else {
  $argsList += "-j"
}

& $make @argsList
exit $LASTEXITCODE
