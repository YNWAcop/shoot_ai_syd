#!/usr/bin/env bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
exec bash "$WS/vlm_shoot/run.sh" "$@"
