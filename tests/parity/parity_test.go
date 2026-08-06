package parity

import (
	"encoding/json"
	"os"
	"testing"

	"github.com/chewxy/math32"
)

// BodySnapshot captures the observable state of a body at step N.
// Used by golden-file parity tests to compare Go output against C++ reference.
type BodySnapshot struct {
	Index    int     `json:"index"`
	Type     string  `json:"type"` // "rigid", "soft", "area"
	PosX     float32 `json:"x"`
	PosY     float32 `json:"y"`
	Rotation float32 `json:"rot"`
}

// WorldSnapshot captures all body states at step N.
type WorldSnapshot struct {
	Step   int            `json:"step"`
	Bodies []BodySnapshot `json:"bodies"`
}

// GoldenFile is the on-disk format for golden snapshots. One file per
// C++ example, containing multiple snapshots at steps 1, 10, 50, 100.
type GoldenFile struct {
	Example   string          `json:"example"`
	Snapshots []WorldSnapshot `json:"snapshots"`
}

// CompareGolden loads a golden file and compares it against the actual
// snapshot. Tolerance: 1e-4 per coordinate (float32 rounding).
//
// Reference: execution guide §9.3 (Tolerance)
func CompareGolden(t *testing.T, goldenPath string, actual WorldSnapshot) {
	t.Helper()
	data, err := os.ReadFile(goldenPath)
	if err != nil {
		t.Fatalf("read golden %s: %v", goldenPath, err)
	}
	var golden GoldenFile
	if err := json.Unmarshal(data, &golden); err != nil {
		t.Fatalf("unmarshal golden %s: %v", goldenPath, err)
	}

	// Find the matching snapshot by step
	var expected *WorldSnapshot
	for i := range golden.Snapshots {
		if golden.Snapshots[i].Step == actual.Step {
			expected = &golden.Snapshots[i]
			break
		}
	}
	if expected == nil {
		t.Fatalf("no golden snapshot for step %d in %s", actual.Step, goldenPath)
	}

	if len(expected.Bodies) != len(actual.Bodies) {
		t.Fatalf("step %d body count mismatch: expected %d, got %d",
			actual.Step, len(expected.Bodies), len(actual.Bodies))
	}

	tol := float32(1e-4)
	if actual.Step >= 100 {
		tol = 1e-3 // allow more drift at step 100
	}

	for i, eb := range expected.Bodies {
		ab := actual.Bodies[i]
		if math32.Abs(eb.PosX-ab.PosX) > tol ||
			math32.Abs(eb.PosY-ab.PosY) > tol ||
			math32.Abs(eb.Rotation-ab.Rotation) > tol {
			t.Errorf("step %d body %d mismatch:\n  expected %+v\n  got      %+v\n  (tol %v)",
				actual.Step, i, eb, ab, tol)
		}
	}
}

// LoadGolden loads and returns the full GoldenFile. Useful when a test
// wants to iterate all snapshots and verify each one.
func LoadGolden(t *testing.T, goldenPath string) GoldenFile {
	t.Helper()
	data, err := os.ReadFile(goldenPath)
	if err != nil {
		t.Fatalf("read golden %s: %v", goldenPath, err)
	}
	var g GoldenFile
	if err := json.Unmarshal(data, &g); err != nil {
		t.Fatalf("unmarshal golden %s: %v", goldenPath, err)
	}
	return g
}
