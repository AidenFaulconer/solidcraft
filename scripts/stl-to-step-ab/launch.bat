@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =============================================================================
REM launch.bat — ensure solidcraft-cli matches current source, then run STL→STEP A/B
REM
REM Rebuilds via build.bat when:
REM   - SOLIDCRAFT_FORCE_BUILD=1
REM   - no solidcraft-cli / no stamp
REM   - git HEAD differs from stamp
REM   - crates/apps sources newer than the CLI exe
REM
REM Usage:
REM   launch.bat
REM   launch.bat --case PSU_rear_mount_back_side_1-screw_v2
REM   SOLIDCRAFT_FORCE_BUILD=1 launch.bat
REM   SOLIDCRAFT_AB_PROFILE=debug launch.bat
REM =============================================================================

cd /d "%~dp0"
call "%~dp0solidcraft-dev-config.bat" || exit /b 1

call :resolve_cli

if "%SOLIDCRAFT_FORCE_BUILD%"=="1" (
  echo [launch] SOLIDCRAFT_FORCE_BUILD=1 — rebuilding...
  call "%~dp0build.bat"
  if errorlevel 1 exit /b 1
  call :resolve_cli
  goto :run_tests
)

call :needs_rebuild
if "!NEED_REBUILD!"=="1" (
  echo [launch] Build is out of date or missing — running build.bat ...
  call "%~dp0build.bat"
  if errorlevel 1 exit /b 1
  call :resolve_cli
) else (
  echo [launch] Build looks up to date.
)

goto :run_tests

:run_tests
if not exist "!SOLIDCRAFT_CLI!" (
  echo [launch] ERROR: solidcraft-cli.exe not found.
  exit /b 1
)
if not exist "%SOLIDCRAFT_AB_SCRIPT%" (
  echo [launch] ERROR: missing "%SOLIDCRAFT_AB_SCRIPT%"
  exit /b 1
)

where python >nul 2>&1
if errorlevel 1 (
  where py >nul 2>&1
  if errorlevel 1 (
    echo [launch] ERROR: python not found on PATH.
    exit /b 1
  )
  set "PY=py -3"
) else (
  set "PY=python"
)

echo.
echo [launch] CLI : !SOLIDCRAFT_CLI!
echo [launch] fixtures: %SOLIDCRAFT_AB_FIXTURES%
echo [launch] out : %SOLIDCRAFT_AB_OUT%
echo.

if not exist "%SOLIDCRAFT_AB_OUT%" mkdir "%SOLIDCRAFT_AB_OUT%"

%PY% "%SOLIDCRAFT_AB_SCRIPT%" --cli "!SOLIDCRAFT_CLI!" --fixtures "%SOLIDCRAFT_AB_FIXTURES%" --out "%SOLIDCRAFT_AB_OUT%" %*
set "EC=!ERRORLEVEL!"
echo.
if not "!EC!"=="0" (
  echo [launch] A/B tests failed ^(exit !EC!^). See "%SOLIDCRAFT_AB_OUT%\report.md"
  exit /b !EC!
)
echo [launch] A/B tests passed. Report: "%SOLIDCRAFT_AB_OUT%\report.md"
exit /b 0

:resolve_cli
set "SOLIDCRAFT_CLI="
if /I "%SOLIDCRAFT_AB_PROFILE%"=="debug" (
  if exist "%SOLIDCRAFT_CLI_DEBUG%" set "SOLIDCRAFT_CLI=%SOLIDCRAFT_CLI_DEBUG%"
) else (
  if exist "%SOLIDCRAFT_CLI_RELEASE%" set "SOLIDCRAFT_CLI=%SOLIDCRAFT_CLI_RELEASE%"
)
if not defined SOLIDCRAFT_CLI if exist "%SOLIDCRAFT_CLI_RELEASE%" set "SOLIDCRAFT_CLI=%SOLIDCRAFT_CLI_RELEASE%"
if not defined SOLIDCRAFT_CLI if exist "%SOLIDCRAFT_CLI_DEBUG%" set "SOLIDCRAFT_CLI=%SOLIDCRAFT_CLI_DEBUG%"
goto :eof

:needs_rebuild
set "NEED_REBUILD=0"

if not exist "!SOLIDCRAFT_CLI!" (
  echo [launch] No solidcraft-cli under "%CARGO_TARGET_DIR%"
  set "NEED_REBUILD=1"
  goto :eof
)

if not exist "%SOLIDCRAFT_BUILD_STAMP%" (
  echo [launch] Missing build stamp — rebuild needed.
  set "NEED_REBUILD=1"
  goto :eof
)

set "STAMP_GIT="
for /f "tokens=1,* delims==" %%A in ('findstr /b "GIT=" "%SOLIDCRAFT_BUILD_STAMP%"') do set "STAMP_GIT=%%B"

set "CUR_GIT=unknown"
pushd "%SOLIDCRAFT_ROOT%" >nul
git rev-parse HEAD >nul 2>&1
if not errorlevel 1 (
  for /f "delims=" %%H in ('git rev-parse HEAD') do set "CUR_GIT=%%H"
)
popd >nul

if not "!STAMP_GIT!"=="" if not "!STAMP_GIT!"=="unknown" if not "!CUR_GIT!"=="!STAMP_GIT!" (
  echo [launch] Git HEAD changed ^(!STAMP_GIT:~0,8!... -^> !CUR_GIT:~0,8!...^)
  set "NEED_REBUILD=1"
  goto :eof
)

REM Source newer than exe?
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$exe = Get-Item -LiteralPath $env:SOLIDCRAFT_CLI -ErrorAction SilentlyContinue; if (-not $exe) { exit 2 }; $roots = @('crates','apps'); $newer = $false; foreach ($r in $roots) { $p = Join-Path $env:SOLIDCRAFT_ROOT $r; if (Test-Path $p) { Get-ChildItem -LiteralPath $p -Recurse -Include *.rs,*.toml -ErrorAction SilentlyContinue | ForEach-Object { if ($_.LastWriteTimeUtc -gt $exe.LastWriteTimeUtc) { $newer = $true } } } }; if ($newer) { Write-Output 'source newer than CLI'; exit 1 } else { exit 0 }"
if errorlevel 1 (
  if errorlevel 2 (
    echo [launch] CLI missing during freshness check.
  ) else (
    echo [launch] Source newer than CLI exe.
  )
  set "NEED_REBUILD=1"
)
goto :eof
