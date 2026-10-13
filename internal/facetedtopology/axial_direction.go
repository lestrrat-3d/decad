package facetedtopology

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// AxialChainDirection proves that every held chord has one exact
// Cartesian Z direction. Callers must separately prove the represented
// source boundary is an axial extrusion; a straight held chain alone cannot
// make that claim for an arbitrary FacetedCurve.
func AxialChainDirection(verts []r3.Vec, chain []int) (r3.Vec, bool) {
	if len(chain) < 2 || chain[0] < 0 || chain[0] >= len(verts) {
		return r3.Vec{}, false
	}
	first := verts[chain[0]]
	if proofbound.IsNonFinite(first.X) || proofbound.IsNonFinite(first.Y) ||
		proofbound.IsNonFinite(first.Z) {
		return r3.Vec{}, false
	}
	previous := first.Z
	direction := 0.0
	for _, index := range chain[1:] {
		if index < 0 || index >= len(verts) {
			return r3.Vec{}, false
		}
		v := verts[index]
		if v.X != first.X || v.Y != first.Y || proofbound.IsNonFinite(v.Z) || v.Z == previous {
			return r3.Vec{}, false
		}
		step := 1.0
		if v.Z < previous {
			step = -1
		}
		if direction != 0 && direction != step {
			return r3.Vec{}, false
		}
		direction, previous = step, v.Z
	}
	return r3.Vec{Z: direction}, true
}
