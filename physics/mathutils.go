package physics

// Math constants and helpers mirroring qmath_utils.h.
// The math32 constants (Pi, Pi2, MaxFloat32) live in math32.go to keep
// all float32 math concerns in one file.

// MaxWorldSize is the maximum world size in pixels. Used as a sentinel in
// collision algorithms (matches QWorld::MAX_WORLD_SIZE in qworld.h:124).
//
// In the C++ engine this is an `inline static float` on QWorld. In Go it
// is a package-level constant — it is never mutated at runtime.
const MaxWorldSize = float32(99999.0)
