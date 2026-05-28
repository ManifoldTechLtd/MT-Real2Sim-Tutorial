#!/bin/bash
# entrypoint_isaac.sh — Isaac Sim container entrypoint
# Sets up environment then executes the passed command
set -e

# Scene file symlink: /scenes inside container -> ~/dataset/isaac
# This way config.yaml paths like "~/dataset/isaac/..." work without modification
SCENE_LINK="$HOME/dataset/isaac"
if [ -d /scenes ] && [ ! -e "$SCENE_LINK" ]; then
    mkdir -p "$(dirname "$SCENE_LINK")"
    ln -s /scenes "$SCENE_LINK"
fi

# Add project root to PYTHONPATH
export PYTHONPATH="/workspace:${PYTHONPATH:-}"

exec "$@"
