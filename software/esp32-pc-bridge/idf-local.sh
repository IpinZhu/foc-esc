#!/usr/bin/env bash
# Compatibility with the earlier version/build/menuconfig commands; no arbitrary IDF actions.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && /bin/pwd -P)"
exec "${PYTHON:-python3}" -I "$ROOT/tools/idf_driver.py" raw "$@"
