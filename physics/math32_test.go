package physics

import (
	"math"
	"testing"
)

func TestMath32(t *testing.T) {
	tests := []struct {
		name string
		got  float32
		want float32
	}{
		{"Sqrt(4)", Sqrt(4), 2},
		{"Sqrt(2)", Sqrt(2), float32(math.Sqrt(2))},
		{"Sin(0)", Sin(0), 0},
		{"Sin(Pi/2)", Sin(Pi / 2), 1},
		{"Cos(0)", Cos(0), 1},
		{"Cos(Pi)", Cos(Pi), -1},
		{"Atan2(1,0)", Atan2(1, 0), Pi / 2},
		{"Abs(-5)", Abs(-5), 5},
		{"Abs(5)", Abs(5), 5},
		{"Floor(3.7)", Floor(3.7), 3},
		{"Floor(-3.2)", Floor(-3.2), -4},
	}
	const tol = 1e-6
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if Abs(tt.got-tt.want) > tol {
				t.Errorf("got %v, want %v (tol %v)", tt.got, tt.want, tol)
			}
		})
	}
}

func TestAsinClamp(t *testing.T) {
	// Asin should clamp to ±π/2 for inputs outside [-1, 1].
	// Matches QSoftBody::safe_asin in qsoftbody.h:65-73.
	if got := Asin(1.5); got != Pi2 {
		t.Errorf("Asin(1.5) = %v, want %v", got, Pi2)
	}
	if got := Asin(-1.5); got != -Pi2 {
		t.Errorf("Asin(-1.5) = %v, want %v", got, -Pi2)
	}
	if got := Asin(0.5); Abs(got-float32(math.Asin(0.5))) > 1e-6 {
		t.Errorf("Asin(0.5) = %v, want %v", got, float32(math.Asin(0.5)))
	}
}

func TestMinMax(t *testing.T) {
	if Min(3.0, 5.0) != 3.0 {
		t.Errorf("Min(3,5) = %v, want 3", Min(3.0, 5.0))
	}
	if Max(3.0, 5.0) != 5.0 {
		t.Errorf("Max(3,5) = %v, want 5", Max(3.0, 5.0))
	}
	if Min(5.0, 3.0) != 3.0 {
		t.Errorf("Min(5,3) = %v, want 3", Min(5.0, 3.0))
	}
}

func TestIsNaN(t *testing.T) {
	if !IsNaN(float32(math.NaN())) {
		t.Error("IsNaN(NaN) = false, want true")
	}
	if IsNaN(3.14) {
		t.Error("IsNaN(3.14) = true, want false")
	}
}
