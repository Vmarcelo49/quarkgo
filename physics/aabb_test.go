package physics

import "testing"

func TestAABBSize(t *testing.T) {
	a := NewAABB(Vec2{X: 1, Y: 2}, Vec2{X: 4, Y: 6})
	s := a.Size()
	if s.X != 3 || s.Y != 4 {
		t.Errorf("Size = %v, want (3,4)", s)
	}
}

func TestAABBPerimeter(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 3, Y: 4})
	if got := a.Perimeter(); got != 14 {
		t.Errorf("Perimeter = %v, want 14", got)
	}
}

func TestAABBArea(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 3, Y: 4})
	if got := a.Area(); got != 12 {
		t.Errorf("Area = %v, want 12", got)
	}
}

func TestAABBCenterPosition(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 4, Y: 6})
	c := a.CenterPosition()
	if c.X != 2 || c.Y != 3 {
		t.Errorf("Center = %v, want (2,3)", c)
	}
}

func TestAABBIsContain(t *testing.T) {
	outer := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 10, Y: 10})
	inner := NewAABB(Vec2{X: 2, Y: 2}, Vec2{X: 8, Y: 8})
	partial := NewAABB(Vec2{X: 5, Y: 5}, Vec2{X: 15, Y: 15})

	if !outer.IsContain(inner) {
		t.Error("outer should contain inner")
	}
	if outer.IsContain(partial) {
		t.Error("outer should NOT contain partial (extends beyond)")
	}
}

func TestAABBIsCollidingWith(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 5, Y: 5})
	overlapping := NewAABB(Vec2{X: 3, Y: 3}, Vec2{X: 8, Y: 8})
	touching := NewAABB(Vec2{X: 5, Y: 0}, Vec2{X: 10, Y: 5}) // shares edge
	disjoint := NewAABB(Vec2{X: 10, Y: 10}, Vec2{X: 20, Y: 20})

	if !a.IsCollidingWith(overlapping) {
		t.Error("a should collide with overlapping")
	}
	if !a.IsCollidingWith(touching) {
		t.Error("a should collide with touching (edge contact counts)")
	}
	if a.IsCollidingWith(disjoint) {
		t.Error("a should NOT collide with disjoint")
	}
	if !a.IsCollidingWith(a) {
		t.Error("a should collide with itself")
	}
}

func TestAABBCombine(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 5, Y: 5})
	b := NewAABB(Vec2{X: 3, Y: 3}, Vec2{X: 10, Y: 8})
	c := Combine(a, b)
	if c.Min.X != 0 || c.Min.Y != 0 {
		t.Errorf("Combined min = %v, want (0,0)", c.Min)
	}
	if c.Max.X != 10 || c.Max.Y != 8 {
		t.Errorf("Combined max = %v, want (10,8)", c.Max)
	}
}

func TestAABBFatten(t *testing.T) {
	a := NewAABB(Vec2{X: 5, Y: 5}, Vec2{X: 10, Y: 10})
	f := a.Fatten(2)
	if f.Min.X != 3 || f.Min.Y != 3 {
		t.Errorf("Fattened min = %v, want (3,3)", f.Min)
	}
	if f.Max.X != 12 || f.Max.Y != 12 {
		t.Errorf("Fattened max = %v, want (12,12)", f.Max)
	}
}

func TestAABBFattenedWithRate(t *testing.T) {
	a := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 10, Y: 10})
	// rate=0.2 → expand by 10% on each side → size goes from 10×10 to 12×12
	f := a.FattenedWithRate(0.2)
	if f.Min.X != -1 || f.Min.Y != -1 {
		t.Errorf("FattenedWithRate min = %v, want (-1,-1)", f.Min)
	}
	if f.Max.X != 11 || f.Max.Y != 11 {
		t.Errorf("FattenedWithRate max = %v, want (11,11)", f.Max)
	}
}

func TestGetAABBFromParticles(t *testing.T) {
	particles := []*Particle{
		{globalPosition: Vec2{X: 0, Y: 0}, r: 1},
		{globalPosition: Vec2{X: 10, Y: 5}, r: 2},
		{globalPosition: Vec2{X: 5, Y: 10}, r: 0.5},
	}
	a := GetAABBFromParticles(particles)
	// Expected: min = (0-1, 0-1) = (-1,-1); max = (10+2, 10+0.5) = (12, 10.5)
	if a.Min.X != -1 || a.Min.Y != -1 {
		t.Errorf("min = %v, want (-1,-1)", a.Min)
	}
	if a.Max.X != 12 || a.Max.Y != 10.5 {
		t.Errorf("max = %v, want (12,10.5)", a.Max)
	}
}

func TestGetAABBFromParticlesEmpty(t *testing.T) {
	a := GetAABBFromParticles(nil)
	if !a.Min.Equal(Vec2Zero()) || !a.Max.Equal(Vec2Zero()) {
		t.Errorf("empty particles AABB = %v, want zero", a)
	}
}
