package physics

import (
	"reflect"
	"sort"
)

// ptrAddr returns the pointer address of a *Body as a uintptr.
// Used for stable ordering of body pairs (canonical map keys).
func ptrAddr(b *Body) uintptr {
	return reflect.ValueOf(b).Pointer()
}

// bodyPairKey is a canonical key for a body pair (smaller address first).
type bodyPairKey struct {
	a, b *Body
}

func newBodyPairKey(a, b *Body) bodyPairKey {
	if ptrAddr(a) <= ptrAddr(b) {
		return bodyPairKey{a: a, b: b}
	}
	return bodyPairKey{a: b, b: a}
}

// --- Brute-force pair generation (O(n²)) ---
// Used when broadphase is disabled. Matches the C++ fallback at qworld.cpp:202-221.

func bruteForcePairs(bodies []*Body) []BodyPair {
	var pairs []BodyPair
	for i, a := range bodies {
		if !a.enabled {
			continue
		}
		for j := i + 1; j < len(bodies); j++ {
			b := bodies[j]
			if !b.enabled {
				continue
			}
			if !a.aabb.IsCollidingWith(b.aabb) {
				continue
			}
			if !CanCollide(a, b, true) {
				continue
			}
			pairs = append(pairs, BodyPair{A: a, B: b})
		}
	}
	return pairs
}

// --- Built-in Sweep-and-Prune (SAP) ---
// Sorts bodies by AABB min.X, then nested-loop with early-out.
// Matches the C++ SAP at qworld.cpp:1126-1132 (SortBodiesHorizontal)
// and qworld.cpp:159-194 (the SAP pair generation).

func sapPairs(bodies []*Body) []BodyPair {
	// Filter to enabled bodies, then sort
	filtered := make([]*Body, 0, len(bodies))
	for _, b := range bodies {
		if b.enabled {
			filtered = append(filtered, b)
		}
	}
	sort.Slice(filtered, func(i, j int) bool {
		ai := filtered[i].aabb.Min.X
		aj := filtered[j].aabb.Min.X
		if ai == aj {
			return filtered[i].aabb.Max.Y > filtered[j].aabb.Max.Y
		}
		return ai < aj
	})

	var pairs []BodyPair
	for i, a := range filtered {
		for j := i + 1; j < len(filtered); j++ {
			b := filtered[j]
			// Early-out: if b's min.X > a's max.X, no further overlaps possible
			if b.aabb.Min.X > a.aabb.Max.X {
				break
			}
			if !a.aabb.IsCollidingWith(b.aabb) {
				continue
			}
			if !CanCollide(a, b, true) {
				continue
			}
			pairs = append(pairs, BodyPair{A: a, B: b})
		}
	}
	return pairs
}
