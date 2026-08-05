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
// the polygon. Faithful port of QMesh::ApplyAngleConstraintsToPolygon
// (qmesh.cpp:314-439).
//
// Algorithm:
//  1. Intersection test: check if the polygon is self-intersecting via
//     pairwise segment intersection. If so, apply a shape-matching fallback
//     (pull particles toward the rest shape with force factor 0.2), clear
//     lastPolygonCornerAngles, and return.
//  2. First-frame skip: if lastPolygonCornerAngles size doesn't match
//     polygon size, initialize to zeros and set beginToSaveAngles=true.
//     On the first frame, just save angles without applying constraints.
//  3. Angle tracking with unwrap: compute the raw atan2 angle for each
//     vertex, then compute angleDifference = AngleBetweenTwoVectors(
//     AngleToUnitVector(angleRad), AngleToUnitVector(lastSaved)). The
//     unwrapped angle is lastSaved + angleDifference. This prevents
//     wrap-around jumps at ±π.
//  4. Position-based correction: if angle > maxAngle or < minAngle, directly
//     SetGlobalPosition on the neighbors (NOT ApplyForce). Force factor 0.5.
//     Check pp.Enabled() and np.Enabled() (NOT p.Enabled()).
func (m *Mesh) ApplyAngleConstraintsToPolygon() {
	if m.minAngleConstraintOfPolygon == 0.0 {
		return
	}
	if len(m.polygon) < 3 {
		return
	}

	// 1. Intersection test — check for polygon self-intersection.
	polygonIntersection := false
	for i := 0; i < len(m.polygon); i++ {
		ni := (i + 1) % len(m.polygon)
		d1A := m.polygon[i].GlobalPosition()
		d1B := m.polygon[ni].GlobalPosition()
		for n := i + 1; n < len(m.polygon); n++ {
			if n == i || n == ni || n == ((i-1+len(m.polygon))%len(m.polygon)) {
				continue
			}
			d2A := m.polygon[n].GlobalPosition()
			d2B := m.polygon[(n+1)%len(m.polygon)].GlobalPosition()
			intersection := LineIntersectionLine(d1A, d1B, d2A, d2B)
			if !intersection.IsNaN() {
				polygonIntersection = true
				break
			}
		}
		if polygonIntersection {
			break
		}
	}
	m.isPolygonSelfIntersected = polygonIntersection

	if polygonIntersection {
		// Shape-matching fallback: pull particles toward rest shape.
		avgPos, avgRot := GetAveragePositionAndRotation(m.polygon)
		matchingShape := GetMatchingParticlePositions(m.polygon, avgPos, avgRot)
		for i := range matchingShape {
			if !m.polygon[i].Enabled() {
				continue
			}
			force := matchingShape[i].Sub(m.polygon[i].GlobalPosition()).Mul(0.2)
			m.polygon[i].ApplyForce(force)
		}
		m.lastPolygonCornerAngles = m.lastPolygonCornerAngles[:0]
		return
	}

	// 2. First-frame skip / angle array initialization.
	beginToSaveAngles := false
	if len(m.lastPolygonCornerAngles) != len(m.polygon) {
		m.lastPolygonCornerAngles = make([]float32, len(m.polygon))
		for i := range m.lastPolygonCornerAngles {
			m.lastPolygonCornerAngles[i] = 0.0
		}
		beginToSaveAngles = true
	}

	minAngle := m.minAngleConstraintOfPolygon
	maxAngle := (Pi * 2.0) - minAngle

	n := len(m.polygon)
	for i := range n {
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

		// First frame: just save the angle, don't apply constraints.
		if beginToSaveAngles {
			m.lastPolygonCornerAngles[i] = angleRad
			continue
		}

		// 3. Angle tracking with unwrap via AngleBetweenTwoVectors.
		d1 := AngleToUnitVector(m.lastPolygonCornerAngles[i])
		d2 := AngleToUnitVector(angleRad)
		angleDifference := AngleBetweenTwoVectors(d2, d1)
		angleRad = m.lastPolygonCornerAngles[i] + angleDifference

		// 4. Position-based correction (SetGlobalPosition, NOT ApplyForce).
		if angleRad > maxAngle {
			diffAngle := maxAngle - angleRad
			angularForce := diffAngle * 0.5
			if pp.Enabled() {
				pp.SetGlobalPosition(p.GlobalPosition().Add(toPrev.Rotated(angularForce)))
			}
			if np.Enabled() {
				np.SetGlobalPosition(p.GlobalPosition().Add(toNext.Rotated(-angularForce)))
			}
		}

		if angleRad < minAngle {
			diffAngle := minAngle - angleRad
			angularForce := diffAngle * 0.5
			if pp.Enabled() {
				pp.SetGlobalPosition(p.GlobalPosition().Add(toPrev.Rotated(angularForce)))
			}
			if np.Enabled() {
				np.SetGlobalPosition(p.GlobalPosition().Add(toNext.Rotated(-angularForce)))
			}
		}

		m.lastPolygonCornerAngles[i] = angleRad
	}
}
