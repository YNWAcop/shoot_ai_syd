#!/usr/bin/env bash
set -e
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WORKSPACE_DIR/vlm_shoot/run.sh" "$@"
