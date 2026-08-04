// Package physics is a Go port of QuarkPhysics, a 2D physics engine for games.
//
// QuarkPhysics is a MIT-licensed 2D physics engine created by Eray Zesen
// (https://github.com/erayzesen/QuarkPhysics). This package preserves the
// engine's Verlet integration scheme, iterative constraint solver, and
// unified rigid/soft/area body model while exposing an idiomatic Go API.
//
// All physics math operates on float32 to match the C++ engine's float type
// and preserve bit-for-bit behavioral parity. See the package-level math32
// helpers for float32 wrappers around the stdlib math package.
//
// The package is organized as a single Go package to mirror the C++ engine's
// friend-class encapsulation pattern: QWorld, QBody, QMesh, QParticle,
// QManifold, and QCollision all need mutual access to unexported fields.
// Putting them in one Go package gives the same access without exported
// fields polluting the public API.
package physics
