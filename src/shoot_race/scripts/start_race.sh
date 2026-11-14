#!/usr/bin/env bash
WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd -P)"
exec bash "$WS/start_syd.sh" "$@"
