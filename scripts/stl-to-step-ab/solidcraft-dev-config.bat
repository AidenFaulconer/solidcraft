@echo off
REM Paths for SolidCraft STL→STEP A/B build + launch (mirrors FREECAD freecad-dev-config.bat).
set "SOLIDCRAFT_ROOT=%~dp0..\.."
for %%I in ("%SOLIDCRAFT_ROOT%") do set "SOLIDCRAFT_ROOT=%%~fI"

if not defined CARGO_TARGET_DIR set "CARGO_TARGET_DIR=%SOLIDCRAFT_ROOT%\target"
set "SOLIDCRAFT_CLI_RELEASE=%CARGO_TARGET_DIR%\release\solidcraft-cli.exe"
set "SOLIDCRAFT_CLI_DEBUG=%CARGO_TARGET_DIR%\debug\solidcraft-cli.exe"
set "SOLIDCRAFT_BUILD_STAMP=%CARGO_TARGET_DIR%\stl-to-step-ab.stamp"
set "SOLIDCRAFT_AB_FIXTURES=%SOLIDCRAFT_ROOT%\tests\stl_to_step_a_b_tests"
set "SOLIDCRAFT_AB_OUT=%SOLIDCRAFT_AB_FIXTURES%\out"
set "SOLIDCRAFT_AB_SCRIPT=%~dp0run_ab_tests.py"

REM cargo profile: release for launch (override with SOLIDCRAFT_AB_PROFILE=debug)
if not defined SOLIDCRAFT_AB_PROFILE set "SOLIDCRAFT_AB_PROFILE=release"
