#!/bin/bash
# Verifies that no Ebitengine imports leak into the core physics package.
# The physics/ package must remain free of rendering dependencies so it
# can be used headless (servers, tests, ML training) without pulling in
# graphics/audio libraries.
#
# Reference: execution guide §0.7 (Ebitengine Boundary), §1.4 (CI)
set -euo pipefail

cd "$(dirname "$0")/.."

# Search for any ebitengine import in core packages
if grep -rn "hajimehoshi/ebiten" physics/ ext/ mesh/ 2>/dev/null; then
    echo ""
    echo "ERROR: Ebitengine imported in core package."
    echo "The physics/ package must not depend on Ebitengine."
    echo "Ebitengine is allowed only in examples/."
    exit 1
fi

echo "OK: No Ebitengine imports in core packages."
