// Package decad is a headless CAD engine for Go: the 3D modeling layer above
// the [sketch] 2D constraint engine and the [r3] coordinate-math layer.
//
// # Why this exists
//
// decad is built to be driven by a coding agent as a verification step BEFORE
// it commits to writing real CAD software code (an Autodesk Fusion add-in, say).
// 3D modeling is easy to get subtly wrong — a profile that does not sweep the
// way you expect, a feature that leaves a self-intersecting or non-watertight
// body, a wall thinner than the tool that has to cut it, two components that
// interfere. Discovering that inside Fusion, after the script has run, is
// expensive.
//
// Instead, an agent reproduces the part here first and interrogates it
// programmatically — is the body watertight, what is its volume and bounding
// box, does it self-intersect, do these two bodies collide — and only carries
// the plan into the CAD package once the geometry is proven sound.
//
// # Layering
//
//	decad    3D bodies, features, verification   (this module)
//	  |
//	sketch   parametric 2D constraint solving    github.com/lestrrat-3d/sketch
//	  |
//	r3       vectors and orthonormal frames      github.com/lestrrat-3d/r3
//
// The arrows point down and NEVER back up: decad imports sketch and r3; neither
// of them knows decad exists. A 2D question — does this profile close, is this
// sketch fully constrained — is answered by sketch and consumed here as a given.
// decad does not re-derive it.
//
// r3 deliberately excludes shapes ("if it lives in ℝ³ it belongs there; if it
// IS a shape it does not"), and sketch deliberately excludes anything that must
// be computed FROM a solid. decad is the layer both of them were leaving room
// for.
//
// # Status
//
// Scaffolding only. The public API is not designed yet, and no kernel surface
// should be added to this package without a design doc landing first. See
// CLAUDE.md.
//
// [sketch]: https://github.com/lestrrat-3d/sketch
// [r3]: https://github.com/lestrrat-3d/r3
package decad
