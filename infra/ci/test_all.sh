#!/bin/bash
set -e

# Define workspace root depending on where script is run
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
ROBOT_WS="$WORKSPACE_ROOT/robot_ws"

echo "=========================================="
echo "Running CI Tests for OmniBot"
echo "Workspace: $ROBOT_WS"
echo "=========================================="

cd "$ROBOT_WS"

echo "Found the following packages:"
colcon list --names-only

echo ""
echo "Building..."
colcon build --symlink-install

echo ""
echo "Testing..."
colcon test

echo ""
echo "Test Results:"
colcon test-result --verbose

echo "=========================================="
echo "Done."
