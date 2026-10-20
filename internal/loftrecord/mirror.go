package loftrecord

import (
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/r3"
)

// MirrorCapTrianglesMatch reports whether the upper loft's cap triangles are
// exactly the lower loft's with opposite winding, each used once.
func (lower Payload) MirrorCapTrianglesMatch(upper Payload) bool {
	start := lower.Walls + lower.CapStartCount
	indices := make(map[[3]int]int, len(lower.Tris)-start)
	for _, triangle := range lower.Tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != 0 {
			return false
		}
		indices[key] = sign
	}
	for _, triangle := range upper.Tris[start:] {
		key, sign := facetedtopology.SortedTriangle(triangle)
		if indices[key] != -sign {
			return false
		}
		delete(indices, key)
	}
	return len(indices) == 0
}

// MirrorAxialSource reports whether both frames sit on the Z axis with the
// world X and Y directions and the end frame above the start frame.
func (lp Payload) MirrorAxialSource() bool {
	for _, f := range []r3.Frame{lp.Frame0, lp.Frame1} {
		if f.U() != (r3.Vec{X: 1}) || f.V() != (r3.Vec{Y: 1}) ||
			f.Origin().X != 0 || f.Origin().Y != 0 {
			return false
		}
	}
	return lp.Frame0.Origin().Z < lp.Frame1.Origin().Z
}
