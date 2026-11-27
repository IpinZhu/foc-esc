#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && /bin/pwd -P)"
exec "${PYTHON:-python3}" -I "$ROOT/tools/idf_driver.py" "$@"
