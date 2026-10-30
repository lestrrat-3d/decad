package loftrecord

import (
	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// clearExactCapLevels sets each cap's AxialDelta to zero where CapOnRecord
// proves that the cap's Plane tag and every point its loops hold lie exactly
// on the plane its CapRecord denotes (agent-docs/loft-design.md §18). Any
// other cap keeps the loft's vertex displacement.
func clearExactCapLevels(caps ...*topo.Face) {
	for _, face := range caps {
		if CapOnRecord(face) {
			face.AxialDelta = 0
		}
	}
}

// CapOnRecord reports whether face's Plane tag is exactly the plane its
// CapRecord denotes, {B(O + u*U + v*V + Z*N) + t}, every held value read as
// an exact rational (capplane.OfRecord), and whether every point face's loops
// hold lies on that plane exactly. With m = (B*U) x (B*V) and
// P = B(O + Z*N) + t, it needs m nonzero; the tag's U and V orthogonal to m
// and its N parallel to m; the tag origin and every loop vertex x with
// m.(x - P) = 0; and every Circle3 or Arc3 edge's center on the plane with its
// axis parallel to m. An edge of any other curve kind, a non-Plane tag, a
// missing record or a non-finite value refuses. It only admits what the
// rational equalities prove: a vertex within rounding of the plane but off it
// refuses.
func CapOnRecord(face *topo.Face) bool {
	rec := face.CapRecord
	tag, ok := face.Surface.(surfacegeom.Plane)
	if rec == nil || !ok {
		return false
	}
	plane, ok := capplane.OfRecord(rec)
	if !ok || !plane.HoldsFrame(tag.Frame) {
		return false
	}
	for _, loop := range face.Loops {
		for _, ce := range loop.CoEdges {
			if !plane.HoldsEdge(ce.Edge) {
				return false
			}
		}
	}
	return true
}
