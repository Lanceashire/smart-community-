#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker build -t aic-smart-community:jazzy -f docker/Dockerfile.jazzy .
