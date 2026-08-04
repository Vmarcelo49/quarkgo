package physics

import (
        "math"
        "testing"
)

func TestVec2Constructors(t *testing.T) {
        if !Vec2Zero().Equal(Vec2{X: 0, Y: 0}) {
                t.Error("Vec2Zero != (0,0)")
        }
        if !Vec2Up().Equal(Vec2{X: 0, Y: -1}) {
                t.Error("Vec2Up != (0,-1)")
        }
        if !Vec2Down().Equal(Vec2{X: 0, Y: 1}) {
                t.Error("Vec2Down != (0,1)")
        }
        if !Vec2Right().Equal(Vec2{X: 1, Y: 0}) {
                t.Error("Vec2Right != (1,0)")
        }
        if !Vec2Left().Equal(Vec2{X: -1, Y: 0}) {
                t.Error("Vec2Left != (-1,0)")
        }
}

func TestVec2Arithmetic(t *testing.T) {
        a := Vec2{X: 1, Y: 2}
        b := Vec2{X: 3, Y: 4}

        if got := a.Add(b); !got.Equal(Vec2{X: 4, Y: 6}) {
                t.Errorf("Add = %v, want (4,6)", got)
        }
        if got := b.Sub(a); !got.Equal(Vec2{X: 2, Y: 2}) {
                t.Errorf("Sub = %v, want (2,2)", got)
        }
        if got := a.Mul(2); !got.Equal(Vec2{X: 2, Y: 4}) {
                t.Errorf("Mul = %v, want (2,4)", got)
        }
        if got := a.Div(2); !got.Equal(Vec2{X: 0.5, Y: 1}) {
                t.Errorf("Div = %v, want (0.5,1)", got)
        }
        if got := a.Neg(); !got.Equal(Vec2{X: -1, Y: -2}) {
                t.Errorf("Neg = %v, want (-1,-2)", got)
        }
}

func TestVec2Dot(t *testing.T) {
        if got := (Vec2{X: 1, Y: 0}).Dot(Vec2{X: 0, Y: 1}); got != 0 {
                t.Errorf("Dot of perpendicular = %v, want 0", got)
        }
        if got := (Vec2{X: 3, Y: 4}).Dot(Vec2{X: 3, Y: 4}); got != 25 {
                t.Errorf("Dot of (3,4)·(3,4) = %v, want 25", got)
        }
}

func TestVec2Length(t *testing.T) {
        v := Vec2{X: 3, Y: 4}
        if got := v.LengthSquared(); got != 25 {
                t.Errorf("LengthSquared = %v, want 25", got)
        }
        if got := v.Length(); got != 5 {
                t.Errorf("Length = %v, want 5", got)
        }
}

func TestVec2Normalized(t *testing.T) {
        // Normal vector
        v := Vec2{X: 3, Y: 4}
        n := v.Normalized()
        if got := n.Length(); Abs(got-1) > 1e-6 {
                t.Errorf("Normalized.Length = %v, want 1", got)
        }

        // Zero vector returns zero, not NaN
        z := Vec2Zero()
        nz := z.Normalized()
        if !nz.Equal(Vec2Zero()) {
                t.Errorf("Normalized(Zero) = %v, want (0,0)", nz)
        }
        if IsNaN(nz.X) || IsNaN(nz.Y) {
                t.Errorf("Normalized(Zero) produced NaN: %v", nz)
        }
}

func TestVec2Perpendicular(t *testing.T) {
        v := Vec2{X: 1, Y: 0}
        p := v.Perpendicular()
        if !p.Equal(Vec2{X: 0, Y: -1}) {
                t.Errorf("Perpendicular of (1,0) = %v, want (0,-1)", p)
        }
        // Perpendicular should be orthogonal
        if got := v.Dot(p); got != 0 {
                t.Errorf("Dot(v, v.Perpendicular()) = %v, want 0", got)
        }
}

func TestVec2Rotated(t *testing.T) {
        v := Vec2{X: 1, Y: 0}
        // Rotate 90° clockwise (positive angle in screen-space convention)
        r := v.Rotated(Pi / 2)
        if Abs(r.X-0) > 1e-6 || Abs(r.Y-1) > 1e-6 {
                t.Errorf("Rotated(1,0) by π/2 = %v, want ~(0,1)", r)
        }
        // Rotate by 0 is identity
        r0 := v.Rotated(0)
        if !r0.Equal(v) {
                t.Errorf("Rotated(1,0) by 0 = %v, want (1,0)", r0)
        }
}

func TestVec2IsNaN(t *testing.T) {
        // Matches QVector::isNaN — both components must be NaN
        if !Vec2NaN().IsNaN() {
                t.Error("Vec2NaN().IsNaN() = false, want true")
        }
        if (Vec2{X: float32(math.NaN()), Y: 0}).IsNaN() {
                t.Error("(NaN, 0).IsNaN() = true, want false (both must be NaN)")
        }
        if (Vec2{X: 1, Y: 2}).IsNaN() {
                t.Error("(1,2).IsNaN() = true, want false")
        }
}

func TestAngleToUnitVector(t *testing.T) {
        v := AngleToUnitVector(0)
        if Abs(v.X-1) > 1e-6 || Abs(v.Y-0) > 1e-6 {
                t.Errorf("AngleToUnitVector(0) = %v, want (1,0)", v)
        }
        v = AngleToUnitVector(Pi / 2)
        if Abs(v.X-0) > 1e-6 || Abs(v.Y-1) > 1e-6 {
                t.Errorf("AngleToUnitVector(π/2) = %v, want (0,1)", v)
        }
}

func TestAngleBetweenTwoVectors(t *testing.T) {
        // Angle from reference=(1,0) to vector=(0,1) should be +π/2.
        // (In screen-space with Y-down, this is a clockwise rotation.)
        got := AngleBetweenTwoVectors(Vec2{X: 0, Y: 1}, Vec2{X: 1, Y: 0})
        if Abs(got-Pi/2) > 1e-6 {
                t.Errorf("angle from (1,0) to (0,1) = %v, want π/2", got)
        }
        // Angle from a vector to itself should be 0.
        got = AngleBetweenTwoVectors(Vec2{X: 1, Y: 0}, Vec2{X: 1, Y: 0})
        if Abs(got-0) > 1e-6 {
                t.Errorf("angle from (1,0) to (1,0) = %v, want 0", got)
        }
}

func TestGetBisectorUnitVector(t *testing.T) {
        // Three points forming a 90° angle at B
        // A=(0,1), B=(0,0), C=(1,0) — bisector should be (1,1)/√2
        b := GetBisectorUnitVector(Vec2{X: 0, Y: 1}, Vec2{X: 0, Y: 0}, Vec2{X: 1, Y: 0}, false)
        want := Vec2{X: 1 / Sqrt(2), Y: 1 / Sqrt(2)}
        if Abs(b.X-want.X) > 1e-6 || Abs(b.Y-want.Y) > 1e-6 {
                t.Errorf("bisector = %v, want %v", b, want)
        }
}

func TestVec2AddAssign(t *testing.T) {
        v := Vec2{X: 1, Y: 2}
        v.AddAssign(Vec2{X: 3, Y: 4})
        if !v.Equal(Vec2{X: 4, Y: 6}) {
                t.Errorf("after AddAssign: %v, want (4,6)", v)
        }
}
