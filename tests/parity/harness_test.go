package parity

import "testing"

// TestParityHarnessCompiles verifies the parity test framework loads and
// the helper functions work. This is a Phase 0 placeholder — real parity
// tests land in Phase 1 once the physics engine can simulate a world.
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

// TestAbsFloat32 verifies the tolerance helper.
func TestAbsFloat32(t *testing.T) {
	if got := absFloat32(-5); got != 5 {
		t.Errorf("absFloat32(-5) = %v, want 5", got)
	}
	if got := absFloat32(5); got != 5 {
		t.Errorf("absFloat32(5) = %v, want 5", got)
	}
	if got := absFloat32(0); got != 0 {
		t.Errorf("absFloat32(0) = %v, want 0", got)
	}
}
