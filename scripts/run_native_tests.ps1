# ================================================================
# Host-side unit tests for the blob detector, built with MSVC.
#
#   powershell -ExecutionPolicy Bypass -File scripts\run_native_tests.ps1
#
# Why this exists: `pio test -e native` needs a native gcc/g++ on PATH, which
# Windows doesn't ship. This machine has Visual Studio 2022, so we drive cl.exe
# directly instead. If you ever install MinGW or run on Linux/macOS, plain
# `pio test -e native` works and does the same thing.
#
# blob_detect.cpp and color_lut.cpp are deliberately free of <Arduino.h> and
# <esp_camera.h> so they compile here unchanged.
# ================================================================
$ErrorActionPreference = "Stop"

$root  = Split-Path -Parent $PSScriptRoot
$unity = Join-Path $root ".pio\libdeps\native\Unity\src"
$build = Join-Path $root ".pio\build\native_msvc"

if (-not (Test-Path (Join-Path $unity "unity.c"))) {
    Write-Host "Unity not found -- fetching it via PlatformIO." -ForegroundColor Yellow
    Write-Host "(Its build step then fails on the missing gcc; that is expected and harmless -"
    Write-Host " we only want the downloaded sources.)"
    $pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"

    # Do NOT redirect this native command's stderr: PowerShell wraps each stderr
    # line in an ErrorRecord, which with ErrorActionPreference=Stop aborts the
    # script even though the download succeeded. Let it write freely and just
    # check for the file afterwards.
    $prev = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try   { & $pio test -d $root -e native | Out-Null }
    catch { }
    $ErrorActionPreference = $prev
    $global:LASTEXITCODE = 0

    if (-not (Test-Path (Join-Path $unity "unity.c"))) {
        throw "Could not obtain Unity sources at $unity"
    }
    Write-Host "Unity fetched." -ForegroundColor Green
}

# ---- bring the MSVC toolchain into this session ----
if (-not (Get-Command cl -ErrorAction SilentlyContinue)) {
    $vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found -- is Visual Studio installed?" }

    $vcvars = $null
    foreach ($inst in (& $vswhere -latest -prerelease -products * -property installationPath)) {
        $candidate = Join-Path $inst "VC\Auxiliary\Build\vcvars64.bat"
        if (Test-Path $candidate) { $vcvars = $candidate; break }
    }
    if (-not $vcvars) {
        # vswhere reports the newest install first, but VC tools may live in another one.
        $found = Get-ChildItem "C:\Program Files*\Microsoft Visual Studio\2022" `
                    -Filter "vcvars64.bat" -Recurse -ErrorAction SilentlyContinue |
                 Select-Object -First 1
        if ($found) { $vcvars = $found.FullName }
    }
    if (-not $vcvars) { throw "Could not locate vcvars64.bat -- install the 'Desktop development with C++' workload." }

    Write-Host "Using $vcvars"
    cmd /c "`"$vcvars`" >nul 2>&1 && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($matches[1])" -Value $matches[2] }
    }
}

New-Item -ItemType Directory -Force -Path $build | Out-Null

$sources = @(
    (Join-Path $unity "unity.c"),
    (Join-Path $root  "src\blob_detect.cpp"),
    (Join-Path $root  "src\color_lut.cpp"),
    (Join-Path $root  "src\blob_filter.cpp"),
    (Join-Path $root  "src\tracker.cpp"),
    (Join-Path $root  "test\test_blob_detect\test_blob_detect.cpp"),
    (Join-Path $root  "test\test_blob_detect\test_filter_tracker.cpp")
)
$exe = Join-Path $build "test_blob_detect.exe"

Write-Host "Compiling..." -ForegroundColor Cyan
& cl /nologo /EHsc /std:c++17 /W3 /O2 `
     /I (Join-Path $root "include") /I $unity `
     /Fo:"$build\" /Fe:"$exe" `
     $sources
if ($LASTEXITCODE -ne 0) { throw "Compilation failed." }

Write-Host ""
Write-Host "Running..." -ForegroundColor Cyan
& $exe
$rc = $LASTEXITCODE

Write-Host ""
if ($rc -eq 0) { Write-Host "PASS" -ForegroundColor Green }
else           { Write-Host "FAIL (exit $rc)" -ForegroundColor Red }
exit $rc
