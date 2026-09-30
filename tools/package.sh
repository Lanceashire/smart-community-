#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out_file="${1:-${root_dir}/smart-community-source.zip}"
cd "${root_dir}"
zip -q -r "${out_file}" README.md NOTICE.md LICENSE src tools validation .gitignore \
  -x '*/__pycache__/*' '*.pyc' '*.egg-info/*'
echo "${out_file}"
