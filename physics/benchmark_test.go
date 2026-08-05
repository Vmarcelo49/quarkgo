package physics

import (
	"fmt"
	"testing"
)

// BenchmarkRigidBodies benchmarks a world with N falling rigid bodies
// stacking on a floor. This is the canonical physics engine benchmark.
func BenchmarkRigidBodies(b *testing.B) {
	for _, n := range []int{10, 50, 100, 500} {
		b.Run(fmt.Sprintf("bodies=%d", n), func(b *testing.B) {
			world := NewWorld(
				WithGravity(Vec2{X: 0, Y: 0.2}),
				WithIterations(4),
			)

			// Floor
			floor := NewRigidBody()
			floor.AddMesh(NewRectMesh(Vec2{X: float32(n * 40), Y: 20}, Vec2Zero(), Vec2Zero()))
			floor.SetPosition(Vec2{X: float32(n * 20), Y: 500})
			floor.SetMode(BodyModeStatic)
			world.AddRigidBody(floor)

			// Falling boxes
			for i := range n {
				box := NewRigidBody()
				box.AddMesh(NewRectMesh(Vec2{X: 16, Y: 16}, Vec2Zero(), Vec2Zero()))
				box.SetPosition(Vec2{X: float32(i*20 + 10), Y: 0})
				world.AddRigidBody(box)
			}

			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				world.Update()
			}
		})
	}
}

// BenchmarkRigidBodiesParallel benchmarks with parallel narrowphase enabled.
func BenchmarkRigidBodiesParallel(b *testing.B) {
	for _, n := range []int{100, 500} {
		b.Run(fmt.Sprintf("bodies=%d", n), func(b *testing.B) {
			world := NewWorld(
				WithGravity(Vec2{X: 0, Y: 0.2}),
				WithIterations(4),
				WithConcurrency(ConcurrencyConfig{Enabled: true}),
			)

			floor := NewRigidBody()
			floor.AddMesh(NewRectMesh(Vec2{X: float32(n * 40), Y: 20}, Vec2Zero(), Vec2Zero()))
			floor.SetPosition(Vec2{X: float32(n * 20), Y: 500})
			floor.SetMode(BodyModeStatic)
			world.AddRigidBody(floor)

			for i := range n {
				box := NewRigidBody()
				box.AddMesh(NewRectMesh(Vec2{X: 16, Y: 16}, Vec2Zero(), Vec2Zero()))
				box.SetPosition(Vec2{X: float32(i*20 + 10), Y: 0})
				world.AddRigidBody(box)
			}

			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				world.Update()
			}
		})
	}
}

// BenchmarkSoftBodies benchmarks soft body simulation.
func BenchmarkSoftBodies(b *testing.B) {
	for _, n := range []int{5, 20, 50} {
		b.Run(fmt.Sprintf("bodies=%d", n), func(b *testing.B) {
			world := NewWorld(
				WithGravity(Vec2{X: 0, Y: 0.2}),
				WithIterations(4),
			)

			floor := NewRigidBody()
			floor.AddMesh(NewRectMesh(Vec2{X: float32(n * 60), Y: 20}, Vec2Zero(), Vec2Zero()))
			floor.SetPosition(Vec2{X: float32(n * 30), Y: 500})
			floor.SetMode(BodyModeStatic)
			world.AddRigidBody(floor)

			for i := range n {
				sb := NewSoftBody()
				sb.AddMesh(NewPolygonMesh(16, 6, Vec2Zero(), -1))
				sb.SetPosition(Vec2{X: float32(i*30 + 30), Y: 0})
				world.AddSoftBody(sb)
			}

			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				world.Update()
			}
		})
	}
}

// BenchmarkWorldCreation benchmarks world + body setup (no simulation).
func BenchmarkWorldCreation(b *testing.B) {
	for b.Loop() {
		world := NewWorld(WithGravity(Vec2{X: 0, Y: 0.2}), WithIterations(4))
		for j := range 100 {
			box := NewRigidBody()
			box.AddMesh(NewRectMesh(Vec2{X: 16, Y: 16}, Vec2Zero(), Vec2Zero()))
			box.SetPosition(Vec2{X: float32(j), Y: 0})
			world.AddRigidBody(box)
		}
	}
}
