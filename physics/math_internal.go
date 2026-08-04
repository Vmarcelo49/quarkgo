package physics

import "math"

// mathNaN returns a float64 NaN, used internally to construct float32 NaN.
// This indirection exists so math32.go can stay focused on wrappers; the
// NaN construction is a special case because Go has no math.NaN32().
func mathNaN() float64 { return math.NaN() }
