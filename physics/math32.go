package physics

import "math"

// This file provides float32 wrappers around the stdlib math package (which
// is float64-only). All QuarkPhysics algorithms operate on float32; do not
// use float64 in physics code — doing so breaks float-drift parity with the
// C++ engine and will cause the parity test suite to fail.
//
// Reference: analysis doc §7.6 (Numeric Precision).

// Pi is the ratio of a circle's circumference to its diameter.
const Pi = float32(math.Pi)

// Pi2 is π/2.
const Pi2 = float32(math.Pi / 2)

// MaxFloat32 is the maximum finite float32 value. Used as a sentinel in
// collision algorithms (matches MAXFLOAT in qmath_utils.h).
const MaxFloat32 = math.MaxFloat32

// Sqrt returns the square root of x.
func Sqrt(x float32) float32 { return float32(math.Sqrt(float64(x))) }

// Sin returns the sine of x (radians).
func Sin(x float32) float32 { return float32(math.Sin(float64(x))) }

// Cos returns the cosine of x (radians).
func Cos(x float32) float32 { return float32(math.Cos(float64(x))) }

// Atan2 returns the arc tangent of y/x, using the signs of both to
// determine the quadrant of the return value.
func Atan2(y, x float32) float32 { return float32(math.Atan2(float64(y), float64(x))) }

// Asin returns the arc sine of x, clamped to [-1, 1] to match
// QSoftBody::safe_asin in qsoftbody.h:65-73. Without this clamp, NaN
// can leak into shape matching and area preserving calculations.
func Asin(x float32) float32 {
	if x < -1.0 {
		return -Pi2
	}
	if x > 1.0 {
		return Pi2
	}
	return float32(math.Asin(float64(x)))
}

// Abs returns the absolute value of x.
func Abs(x float32) float32 { return float32(math.Abs(float64(x))) }

// Floor returns the greatest integer value less than or equal to x.
func Floor(x float32) float32 { return float32(math.Floor(float64(x))) }

// IsNaN reports whether x is an IEEE 754 "not-a-number" value.
func IsNaN(x float32) bool { return math.IsNaN(float64(x)) }

// Min returns the smaller of a or b. Faster than math.Min because it
// avoids the float64 conversion and handles NaN explicitly.
func Min(a, b float32) float32 {
	if IsNaN(a) || IsNaN(b) {
		return float32(math.NaN())
	}
	if a < b {
		return a
	}
	return b
}

// Max returns the larger of a or b.
func Max(a, b float32) float32 {
	if IsNaN(a) || IsNaN(b) {
		return float32(math.NaN())
	}
	if a > b {
		return a
	}
	return b
}
