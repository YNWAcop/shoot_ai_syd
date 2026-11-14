#!/usr/bin/env bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
echo "This entry now starts the FULL competition. Use start_syd.sh for the current entry."
exec bash "$WS/start_syd.sh" "$@"
