param(
    [string]$NativeCompiler,
    [switch]$SkipFirmware,
    [switch]$IncludeDiagnostics
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$cmakeExe = (Get-Command cmake -ErrorAction Stop).Source
$ctestExe = (Get-Command ctest -ErrorAction Stop).Source

if (-not $NativeCompiler) {
    $nativeCommand = Get-Command gcc -ErrorAction SilentlyContinue
    if ($nativeCommand) {
        $NativeCompiler = $nativeCommand.Source
    } else {
        $candidates = Get-ChildItem -Path 'C:\Program Files\JetBrains\CLion*\bin\mingw\bin\gcc.exe' -File -ErrorAction SilentlyContinue
        $NativeCompiler = ($candidates | Sort-Object FullName -Descending | Select-Object -First 1).FullName
    }
}
if (-not $NativeCompiler -or -not (Test-Path -LiteralPath $NativeCompiler)) {
    throw 'Pass -NativeCompiler with the path to a Windows host GCC compiler.'
}

$hostBuild = Join-Path $projectRoot 'build\static-fix\host'
$cmakeCompiler = $NativeCompiler.Replace('\', '/')
& $cmakeExe -S $PSScriptRoot -B $hostBuild -G Ninja "-DCMAKE_C_COMPILER:FILEPATH=$cmakeCompiler" -DCMAKE_BUILD_TYPE=Debug
if ($LASTEXITCODE -ne 0) { throw 'Host test configuration failed.' }
& $cmakeExe --build $hostBuild --parallel 4
if ($LASTEXITCODE -ne 0) { throw 'Host test build failed.' }
& $ctestExe --test-dir $hostBuild --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Host regression tests failed.' }

if (-not $SkipFirmware) {
    Push-Location -LiteralPath $projectRoot
    try {
        foreach ($configuration in @('Debug', 'Release', 'EncoderTest', 'EepromDebug', 'WheelTest',
                                      'UsbDebug', 'UsbWheelTest', 'UsbEncoderTest')) {
            $firmwareBuild = Join-Path $projectRoot "build\static-fix\firmware-$configuration"
            & $cmakeExe --preset $configuration -B $firmwareBuild
            if ($LASTEXITCODE -ne 0) { throw "$configuration firmware configuration failed." }
            & $cmakeExe --build $firmwareBuild --parallel 4
            if ($LASTEXITCODE -ne 0) { throw "$configuration firmware build failed." }
            & python (Join-Path $PSScriptRoot 'check_architecture.py') --compile-commands (Join-Path $firmwareBuild 'compile_commands.json')
            if ($LASTEXITCODE -ne 0) { throw "$configuration architecture boundary check failed." }
        }
        if ($IncludeDiagnostics) {
            foreach ($gpioMode in @(0, 1)) {
                $diagnosticBuild = Join-Path $projectRoot "build\static-fix\firmware-MotorDiagnostic-$gpioMode"
                $diagnosticFlags = "-DMOTOR_POLARITY_TEST_ENABLED=1 -DMOTOR_DIRECT_GPIO_DRIVE=$gpioMode"
                & $cmakeExe --preset Debug -B $diagnosticBuild "-DCMAKE_C_FLAGS=$diagnosticFlags"
                if ($LASTEXITCODE -ne 0) { throw "Diagnostic $gpioMode configuration failed." }
                & $cmakeExe --build $diagnosticBuild --parallel 4
                if ($LASTEXITCODE -ne 0) { throw "Diagnostic $gpioMode build failed." }
            }
            foreach ($motorMode in @(0, 1)) {
                $diagnosticBuild = Join-Path $projectRoot "build\static-fix\firmware-ImuDiagnostic-$motorMode"
                $diagnosticFlags = "-DIMU_UART_TEST_ENABLED=1 -DMOTOR_POLARITY_TEST_ENABLED=$motorMode"
                & $cmakeExe --preset Debug -B $diagnosticBuild "-DCMAKE_C_FLAGS=$diagnosticFlags"
                if ($LASTEXITCODE -ne 0) { throw "IMU diagnostic $motorMode configuration failed." }
                & $cmakeExe --build $diagnosticBuild --parallel 4
                if ($LASTEXITCODE -ne 0) { throw "IMU diagnostic $motorMode build failed." }
            }
        }
    } finally {
        Pop-Location
    }
}
Write-Output 'Static checks passed. No hardware was connected or flashed.'
