#!/usr/bin/env bash
# Build the SolidCraft desktop app when its sources are newer than the binary, then start it.
#
#   scripts/run-solidcraft.sh                  # sample design
#   scripts/run-solidcraft.sh path/to/part.step
#   SOLIDCRAFT_PROFILE=debug scripts/run-solidcraft.sh
#   SOLIDCRAFT_FORCE_BUILD=1 scripts/run-solidcraft.sh
#   SOLIDCRAFT_CHECK_ONLY=1 scripts/run-solidcraft.sh   # build if needed, do not start
#
# CARGO_TARGET_DIR defaults to <repo>/target. A binary left from the same sources is reused.

set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

if ! command -v cargo >/dev/null 2>&1; then
  if [[ -f "${HOME:-}/.cargo/env" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/.cargo/env"
  elif [[ -n "${USERPROFILE:-}" && -f "${USERPROFILE}/.cargo/env" ]]; then
    # shellcheck disable=SC1091
    source "${USERPROFILE}/.cargo/env"
  fi
fi
if ! command -v cargo >/dev/null 2>&1; then
  echo "[run] cargo is not on PATH" >&2
  exit 1
fi

profile=${SOLIDCRAFT_PROFILE:-release}
if [[ "$profile" != "release" && "$profile" != "debug" ]]; then
  echo "[run] SOLIDCRAFT_PROFILE must be release or debug" >&2
  exit 1
fi

if [[ -z "${CARGO_TARGET_DIR:-}" ]]; then
  export CARGO_TARGET_DIR="$root/target"
fi
mkdir -p "$CARGO_TARGET_DIR"

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) exe_name="solidcraft.exe" ;;
  *) exe_name="solidcraft" ;;
esac

if [[ "$profile" == "release" ]]; then
  exe="$CARGO_TARGET_DIR/release/$exe_name"
  cargo_args=(build --release -p solidcraft)
else
  exe="$CARGO_TARGET_DIR/debug/$exe_name"
  cargo_args=(build -p solidcraft)
fi

needs=0
if [[ "${SOLIDCRAFT_FORCE_BUILD:-}" == "1" ]]; then
  echo "[run] SOLIDCRAFT_FORCE_BUILD=1 — rebuilding"
  needs=1
elif [[ ! -f "$exe" ]]; then
  echo "[run] No binary at $exe"
  needs=1
else
  newer=$(
    find "$root/crates" "$root/apps" "$root/Cargo.toml" "$root/Cargo.lock" \
      -type f \( -name '*.rs' -o -name '*.toml' -o -name 'Cargo.lock' \) \
      -newer "$exe" -print -quit 2>/dev/null || true
  )
  if [[ -n "$newer" ]]; then
    echo "[run] Source is newer than the binary ($newer)"
    needs=1
  else
    echo "[run] Binary matches current sources: $exe"
  fi
fi

if [[ "$needs" -eq 1 ]]; then
  echo "[run] Building solidcraft ($profile)"
  echo "[run] target: $CARGO_TARGET_DIR"
  cargo "${cargo_args[@]}"
fi

if [[ ! -f "$exe" ]]; then
  echo "[run] solidcraft was not produced at $exe" >&2
  exit 1
fi

# Windows reports ERROR_BAD_EXE_FORMAT for a PE at or above 2 GiB. A debug
# build with full debuginfo crosses that. Dropping debug info keeps the code.
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    exe_bytes=$(stat -c %s "$exe" 2>/dev/null || echo 0)
    if [[ "$exe_bytes" -ge 2147483648 ]]; then
      if ! command -v strip >/dev/null 2>&1; then
        echo "[run] $exe is ${exe_bytes} bytes and Windows will not load it." >&2
        echo "[run] Install strip, or rebuild with less debug info." >&2
        exit 1
      fi
      echo "[run] Binary is ${exe_bytes} bytes; stripping debug info so Windows can load it"
      strip --strip-debug "$exe"
    fi
    ;;
esac

if [[ "${SOLIDCRAFT_CHECK_ONLY:-}" == "1" ]]; then
  echo "[run] Not starting (SOLIDCRAFT_CHECK_ONLY=1)."
  exit 0
fi

if [[ $# -eq 0 ]]; then
  set -- --sample
fi

echo "[run] Starting $exe $*"
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    # Git bash cannot exec this PE (Exec format error above 2 GiB, and a
    # console-subsystem debug build hangs if it inherits bash's console).
    # Start-Process gives the app its own window and waits until it exits.
    win_exe=$(cygpath -w "$exe")
    ps_args=""
    for arg in "$@"; do
      escaped=${arg//\'/\'\'}
      ps_args+="'$escaped',"
    done
    ps_args=${ps_args%,}
    exec powershell.exe -NoProfile -Command "Start-Process -FilePath '$win_exe' -ArgumentList @($ps_args) -Wait"
    ;;
  *)
    exec "$exe" "$@"
    ;;
esac
