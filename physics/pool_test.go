package physics

import "testing"

func TestContactPoolGetReset(t *testing.T) {
	pool := NewContactPool()
	c := pool.Get()

	// A freshly pooled Contact should be zeroed
	if c.Particle != nil {
		t.Error("fresh Contact.Particle should be nil")
	}
	if !c.Position.Equal(Vec2Zero()) {
		t.Error("fresh Contact.Position should be zero")
	}
	if c.Penetration != 0 {
		t.Error("fresh Contact.Penetration should be 0")
	}
	if c.Solved {
		t.Error("fresh Contact.Solved should be false")
	}
}

func TestContactPoolPutClearsFields(t *testing.T) {
	pool := NewContactPool()
	c := pool.Get()

	// Mutate the contact
	c.Particle = &Particle{}
	c.Position = Vec2{X: 5, Y: 5}
	c.Normal = Vec2{X: 1, Y: 0}
	c.Penetration = 3.5
	c.ReferenceParticles = []*Particle{{}}
	c.Solved = true

	pool.Put(c)

	// After Put, fields should be cleared (no stale references)
	// Note: we can't read the same pointer back reliably with sync.Pool,
	// but we can verify Put doesn't panic and the pool still works.
	c2 := pool.Get()
	if c2.Particle != nil {
		t.Error("recycled Contact.Particle should be nil after Put")
	}
	if c2.Penetration != 0 {
		t.Error("recycled Contact.Penetration should be 0 after Put")
	}
}

func TestContactConfigure(t *testing.T) {
	c := &Contact{}
	p := &Particle{}
	ref := []*Particle{{}, {}}
	c.Configure(p, Vec2{X: 1, Y: 2}, Vec2{X: 0, Y: 1}, 0.5, ref)

	if c.Particle != p {
		t.Error("Configure didn't set Particle")
	}
	if c.Position.X != 1 || c.Position.Y != 2 {
		t.Errorf("Position = %v, want (1,2)", c.Position)
	}
	if c.Normal.X != 0 || c.Normal.Y != 1 {
		t.Errorf("Normal = %v, want (0,1)", c.Normal)
	}
	if c.Penetration != 0.5 {
		t.Errorf("Penetration = %v, want 0.5", c.Penetration)
	}
	if len(c.ReferenceParticles) != 2 {
		t.Errorf("len(ReferenceParticles) = %d, want 2", len(c.ReferenceParticles))
	}
	if c.Solved {
		t.Error("Configure should set Solved=false")
	}
}
