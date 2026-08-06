package parity

import (
	"testing"
)

// TestParityHarnessCompiles verifies the parity test framework loads and
// the helper functions work.
func TestParityHarnessCompiles(t *testing.T) {
	snap := WorldSnapshot{
		Step: 1,
		Bodies: []BodySnapshot{
			{Index: 0, Type: "rigid", PosX: 100, PosY: 200, Rotation: 0},
		},
	}
	if snap.Step != 1 {
		t.Fatal("snapshot step mismatch")
	}
	if len(snap.Bodies) != 1 {
		t.Fatal("snapshot body count mismatch")
	}
}
