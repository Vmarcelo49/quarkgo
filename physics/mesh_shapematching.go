package physics

// Mesh shape-matching and polygon constraint helpers.
// Matches static methods on QMesh in qmesh.cpp.

// GetAveragePositionAndRotation computes the average position and rotation
// of a set of particles. Used by shape matching to find the target transform.
// Matches QMesh::GetAveragePositionAndRotation in qmesh.cpp.
//
// The rotation is computed by finding the angle that best aligns the
// current particle positions with their local (rest) positions.
func GetAveragePositionAndRotation(particles []*Particle) (Vec2, float32) {
	if len(particles) == 0 {
		return Vec2Zero(), 0
	}
	if len(particles) == 1 {
		return particles[0].GlobalPosition(), 0
	}

	// Average position
	var averagePosition Vec2
	for _, p := range particles {
		averagePosition = averagePosition.Add(p.GlobalPosition())
	}
	averagePosition = averagePosition.Div(float32(len(particles)))

	// Average rotation via cos/sin accumulation
	var cosAxis, sinAxis float32
	for _, p := range particles {
		currentVec := p.GlobalPosition().Sub(averagePosition)
		cosAxis += currentVec.Dot(p.Position())
		sinAxis += currentVec.Dot(p.Position().Perpendicular())
	}

	rad := Atan2(sinAxis, cosAxis)
	return averagePosition, rad
}

// GetMatchingParticlePositions computes the target positions for shape matching.
// Each particle's LOCAL position is rotated by -targetRotation and translated
// to targetPosition. Matches QMesh::GetMatchingParticlePositions in qmesh.cpp.
func GetMatchingParticlePositions(particles []*Particle, targetPosition Vec2, targetRotation float32) []Vec2 {
	if len(particles) == 0 {
		return nil
	}

	// Local center
	var localCenter Vec2
	for _, p := range particles {
		localCenter = localCenter.Add(p.Position())
	}
	localCenter = localCenter.Div(float32(len(particles)))

	positions := make([]Vec2, len(particles))
	for n, p := range particles {
		targetPos := p.Position().Sub(localCenter).Rotated(-targetRotation)
		targetPos = targetPos.Add(targetPosition)
		positions[n] = targetPos
	}
	return positions
}

// ApplyAngleConstraintsToPolygon applies per-vertex angle constraints to
// the polygon. This is a simplified version of the C++ method that creates
// angle constraints on the fly and applies them.
//
// The C++ method (qmesh.cpp) checks for polygon self-intersection first
// and applies a shape-matching fallback if detected. For Phase 2, we
// implement the angle constraint portion only — self-intersection handling
// is deferred to Phase 3 (when polypartition is available).
//
// Matches QMesh::ApplyAngleConstraintsToPolygon (simplified).
func (m *Mesh) ApplyAngleConstraintsToPolygon() {
	if m.minAngleConstraintOfPolygon == 0.0 {
		return
	}
	if len(m.polygon) < 3 {
		return
	}

	// Lazy initialization of lastPolygonCornerAngles
	if len(m.lastPolygonCornerAngles) != len(m.polygon) {
		m.lastPolygonCornerAngles = make([]float32, len(m.polygon))
		for i := range m.lastPolygonCornerAngles {
			m.lastPolygonCornerAngles[i] = 0
		}
	}

	minAngle := m.minAngleConstraintOfPolygon
	maxAngle := (Pi * 2.0) - minAngle

	n := len(m.polygon)
	for i := 0; i < n; i++ {
		pi := (i - 1 + n) % n
		ni := (i + 1) % n

		pp := m.polygon[pi]
		p := m.polygon[i]
		np := m.polygon[ni]

		toPrev := pp.GlobalPosition().Sub(p.GlobalPosition())
		toNext := np.GlobalPosition().Sub(p.GlobalPosition())

		prevLen := toPrev.Length()
		nextLen := toNext.Length()
		if prevLen < 1e-6 || nextLen < 1e-6 {
			continue
		}

		cosA := toNext.Dot(toPrev) / (prevLen * nextLen)
		sinA := toNext.Dot(toPrev.Perpendicular()) / (prevLen * nextLen)

		angleRad := Atan2(sinA, cosA)
		if angleRad < 0 {
			angleRad = (Pi * 2.0) - Abs(angleRad)
		}

		// Simple angle constraint: if angle is outside [min, max], apply a
		// corrective force. This is a simplified version of the C++ logic
		// which uses prevAngle tracking for wrap-around.
		if angleRad > maxAngle || angleRad < minAngle {
			// Apply a small corrective force toward the rest angle
			targetAngle := (minAngle + maxAngle) * 0.5
			diff := targetAngle - angleRad
			angularForce := diff * 0.1

			if p.enabled {
				// Rotate p's neighbors slightly to correct the angle
				targetPrev := p.GlobalPosition().Add(toPrev.Rotated(angularForce))
				force := targetPrev.Sub(pp.GlobalPosition()).Mul(0.5)
				pp.ApplyForce(force)

				targetNext := p.GlobalPosition().Add(toNext.Rotated(-angularForce))
				force = targetNext.Sub(np.GlobalPosition()).Mul(0.5)
				np.ApplyForce(force)
			}
		}

		m.lastPolygonCornerAngles[i] = angleRad
	}
}
