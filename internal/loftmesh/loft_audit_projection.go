package loftmesh

import (
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

type loftFloatVerdict uint8

const (
	loftFloatUnknown loftFloatVerdict = iota
	loftFloatApart
	loftFloatEdge
	loftFloatVertex
)

// loftFloatPairProof reads only float vertices, whose exact lifts are the
// geometry being audited. Every nonzero orientation is certified by its
// existing error bound; zero or uncertainty proves nothing. ExactInput
// disables this path because its rational vertices may differ from the floats.
func loftFloatPairProof(verts []r3.Vec, tris [][3]int, i, j int, shortcuts LoftAuditShortcuts) loftFloatVerdict {
	if shortcuts.ExactInput {
		return loftFloatUnknown
	}
	shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
	ta, tb := LoftTriCorners(verts, tris[i]), LoftTriCorners(verts, tris[j])
	switch count {
	case 0:
		if shortcuts.BroadPhase && loftFloatSeparated(ta, tb) {
			return loftFloatApart
		}
	case 1:
		if shortcuts.Certificates && loftFloatVertexProof(ta, tb, tris[i], tris[j], shared[0]) {
			return loftFloatVertex
		}
	case 2:
		if !shortcuts.Certificates {
			return loftFloatUnknown
		}
		apex := verts[tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])]
		if sign, certain := proof.OrientSignFloat(ta[0], ta[1], ta[2], apex); certain && sign != 0 {
			return loftFloatEdge
		}
		pa := verts[tessellation.TriangleApexIndex(tris[i], shared[0], shared[1])]
		for _, axes := range [3][2]int{{0, 1}, {0, 2}, {1, 2}} {
			p, q := loftProjectFloat(verts[shared[0]], axes), loftProjectFloat(verts[shared[1]], axes)
			side := meshbool.Orient2Float(p, q, loftProjectFloat(pa, axes))
			if side != 0 && meshbool.Orient2Float(p, q, loftProjectFloat(apex, axes)) == -side {
				return loftFloatEdge
			}
		}
	}
	return loftFloatUnknown
}

func loftProjectFloat(v r3.Vec, axes [2]int) [2]float64 {
	return [2]float64{meshbool.AxisCoord(v, axes[0]), meshbool.AxisCoord(v, axes[1])}
}

// A coordinate projection is linear. An oriented line in that projection
// lifts to a plane in 3D, with the same signs. If a triangle's third corner
// is strictly on one side of its edge's line, the triangle occupies that
// closed half-space and meets the plane exactly in the edge. Three strictly
// opposite corners put the other triangle wholly outside that half-space.
// Thus a projected strict separator proves 3D disjointness, even when the
// triangles are not coplanar. Two shared-edge apexes on opposite strict sides
// likewise prove that the whole 3D intersection is the expected edge.
func loftFloatSeparated(ta, tb [3]r3.Vec) bool {
	if signs, certain := floatPlaneSigns(ta, tb, [3]int{}, -1); certain && meshbool.AllOneSide(signs) {
		return true
	}
	if signs, certain := floatPlaneSigns(tb, ta, [3]int{}, -1); certain && meshbool.AllOneSide(signs) {
		return true
	}
	for _, axes := range [3][2]int{{0, 1}, {0, 2}, {1, 2}} {
		var a, b [3][2]float64
		for k := range 3 {
			a[k], b[k] = loftProjectFloat(ta[k], axes), loftProjectFloat(tb[k], axes)
		}
		for swap := range 2 {
			if swap == 1 {
				a, b = b, a
			}
			for edge := range 3 {
				p, q := a[edge], a[(edge+1)%3]
				side := meshbool.Orient2Float(p, q, a[(edge+2)%3])
				if side == 0 {
					continue
				}
				apart := true
				for _, point := range b {
					if meshbool.Orient2Float(p, q, point) != -side {
						apart = false
						break
					}
				}
				if apart {
					return true
				}
			}
		}
	}
	return false
}

// A projected edge line through the shared vertex lifts to a plane containing
// that edge. If this triangle's third corner is strictly on one side and the
// other triangle's two remaining corners are strictly on the other, the
// other triangle meets the plane only at the shared vertex. Convexity confines
// the whole intersection to that vertex. Coplanarity is not required.
func loftFloatVertexProof(ta, tb [3]r3.Vec, a, b [3]int, shared int) bool {
	if signs, certain := floatPlaneSigns(ta, tb, b, shared); certain && IsolatedSharedVertex(b, shared, signs) {
		return true
	}
	if signs, certain := floatPlaneSigns(tb, ta, a, shared); certain && IsolatedSharedVertex(a, shared, signs) {
		return true
	}
	var v r3.Vec
	var aa, bb [2]r3.Vec
	if !splitSharedCorner(a, ta, shared, &v, &aa) || !splitSharedCorner(b, tb, shared, &v, &bb) {
		return false
	}
	if q1, q2, certain := separatingPlanePoints(v, aa, bb); certain && separatesAtVertex(v, q1, q2, aa, bb) {
		return true
	}
	for swap := range 2 {
		if swap == 1 {
			aa, bb = bb, aa
		}
		for edge := range 2 {
			for _, axes := range [3][2]int{{0, 1}, {0, 2}, {1, 2}} {
				p, q := loftProjectFloat(v, axes), loftProjectFloat(aa[edge], axes)
				side := meshbool.Orient2Float(p, q, loftProjectFloat(aa[1-edge], axes))
				if side != 0 && meshbool.Orient2Float(p, q, loftProjectFloat(bb[0], axes)) == -side &&
					meshbool.Orient2Float(p, q, loftProjectFloat(bb[1], axes)) == -side {
					return true
				}
			}
		}
	}
	return false
}
