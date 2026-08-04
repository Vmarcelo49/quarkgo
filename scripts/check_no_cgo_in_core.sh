#!/bin/bash
# Verifies that no cgo leaks into the core packages. The port is pure Go.
#
# Reference: execution guide §0.5 (No cgo)
set -euo pipefail

cd "$(dirname "$0")/.."

# Look for import "C" or CGO_ENABLED-related pragmas
if grep -rn 'import "C"' physics/ ext/ mesh/ 2>/dev/null; then
    echo ""
    echo "ERROR: cgo import found in core package."
    echo "The port is pure Go — no cgo allowed (except temporary polypartition fallback)."
    exit 1
fi

# Also check for //go:cgo pragmas
if grep -rn "^//go:cgo" physics/ ext/ mesh/ 2>/dev/null; then
    echo ""
    echo "ERROR: cgo pragma found in core package."
    exit 1
fi

echo "OK: No cgo in core packages."
