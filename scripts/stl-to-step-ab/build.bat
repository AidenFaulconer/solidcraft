@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =============================================================================
REM build.bat — build solidcraft-cli for STL→STEP A/B tests
REM =============================================================================

cd /d "%~dp0"
call "%~dp0solidcraft-dev-config.bat" || exit /b 1

echo.
echo === SolidCraft CLI build ===
echo ROOT    : %SOLIDCRAFT_ROOT%
echo TARGET  : %CARGO_TARGET_DIR%
echo PROFILE : %SOLIDCRAFT_AB_PROFILE%
echo.

where cargo >nul 2>&1
if errorlevel 1 (
  echo ERROR: cargo not found on PATH.
  exit /b 1
)

cd /d "%SOLIDCRAFT_ROOT%"
if /I "%SOLIDCRAFT_AB_PROFILE%"=="debug" (
  cargo build -p solidcraft-cli
) else (
  cargo build --release -p solidcraft-cli
)
if errorlevel 1 exit /b 1

set "CLI="
if /I "%SOLIDCRAFT_AB_PROFILE%"=="debug" (
  set "CLI=%SOLIDCRAFT_CLI_DEBUG%"
) else (
  set "CLI=%SOLIDCRAFT_CLI_RELEASE%"
)
if not exist "!CLI!" (
  echo ERROR: built CLI not found at "!CLI!"
  exit /b 1
)

set "CUR_GIT=unknown"
pushd "%SOLIDCRAFT_ROOT%" >nul
git rev-parse HEAD >nul 2>&1
if not errorlevel 1 (
  for /f "delims=" %%H in ('git rev-parse HEAD') do set "CUR_GIT=%%H"
)
popd >nul

(
  echo GIT=!CUR_GIT!
  echo PROFILE=%SOLIDCRAFT_AB_PROFILE%
  echo CLI=!CLI!
  echo BUILT=%DATE% %TIME%
) > "%SOLIDCRAFT_BUILD_STAMP%"

echo.
echo [build] OK → !CLI!
exit /b 0
