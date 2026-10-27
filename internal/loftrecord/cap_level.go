package loftrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
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
// an exact rational, and whether every point face's loops hold lies on that
// plane exactly. With m = (B*U) x (B*V) and P = B(O + Z*N) + t, it needs m
// nonzero; the tag's U and V orthogonal to m and its N parallel to m; the tag
// origin and every loop vertex x with m.(x - P) = 0; and every Circle3 or
// Arc3 edge's center on the plane with its axis parallel to m. An edge of any
// other curve kind, a non-Plane tag, a missing record or a non-finite value
// refuses. It only admits what the rational equalities prove: a vertex
// within rounding of the plane but off it refuses.
func CapOnRecord(face *topo.Face) bool {
	rec := face.CapRecord
	tag, ok := face.Surface.(surfacegeom.Plane)
	if rec == nil || !ok {
		return false
	}
	plane, ok := exactRecordPlane(rec)
	if !ok {
		return false
	}
	tu, okU := massmoment.ExactVec(tag.Frame.U())
	tv, okV := massmoment.ExactVec(tag.Frame.V())
	tn, okN := massmoment.ExactVec(tag.Frame.N())
	if !okU || !okV || !okN || ratDot3(plane.normal, tu).Sign() != 0 || ratDot3(plane.normal, tv).Sign() != 0 ||
		!ratParallel3(plane.normal, tn) || !plane.holds(tag.Frame.Origin()) {
		return false
	}
	for _, loop := range face.Loops {
		for _, ce := range loop.CoEdges {
			if !plane.holdsEdge(ce.Edge) {
				return false
			}
		}
	}
	return true
}

// recordPlane is the exact plane a CapRecord denotes: the points x with
// normal.(x - point) = 0.
type recordPlane struct {
	normal, point [3]*big.Rat
}

// exactRecordPlane reads rec's plane exactly. ok is false when any held
// value is not finite or the mapped normal is zero.
func exactRecordPlane(rec *topo.CapRecord) (recordPlane, bool) {
	b := rec.Xform.Basis()
	var vecs [7][3]*big.Rat
	for i, v := range []r3.Vec{rec.Frame.U(), rec.Frame.V(), rec.Frame.N(), rec.Frame.Origin(), b.EX, b.EY, b.EZ} {
		exact, ok := massmoment.ExactVec(v)
		if !ok {
			return recordPlane{}, false
		}
		vecs[i] = exact
	}
	t, okT := massmoment.ExactVec(rec.Xform.Translation())
	z := proofarith.FloatRat(rec.Z)
	if !okT || z == nil {
		return recordPlane{}, false
	}
	fu, fv, fn, fo := vecs[0], vecs[1], vecs[2], vecs[3]
	apply := func(a [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = proofbound.RatAdd(proofbound.RatMul(vecs[4][i], a[0]), proofbound.RatMul(vecs[5][i], a[1]),
				proofbound.RatMul(vecs[6][i], a[2]))
		}
		return out
	}
	normal := ratCross3(apply(fu), apply(fv))
	if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
		return recordPlane{}, false
	}
	var local [3]*big.Rat
	for i := range local {
		local[i] = proofbound.RatAdd(fo[i], proofbound.RatMul(z, fn[i]))
	}
	point := apply(local)
	for i := range point {
		point[i] = proofbound.RatAdd(point[i], t[i])
	}
	return recordPlane{normal: normal, point: point}, true
}

// holds reports whether the held point x lies exactly on the plane.
func (p recordPlane) holds(x r3.Vec) bool {
	exact, ok := massmoment.ExactVec(x)
	if !ok {
		return false
	}
	var rel [3]*big.Rat
	for i := range rel {
		rel[i] = new(big.Rat).Sub(exact[i], p.point[i])
	}
	return ratDot3(p.normal, rel).Sign() == 0
}

// holdsEdge reports whether e's end vertices lie on the plane and its curve
// is a line between them or a circle or arc whose center lies on the plane
// and whose axis is parallel to its normal.
func (p recordPlane) holdsEdge(e *topo.Edge) bool {
	if e == nil || e.Start == nil || e.End == nil || !p.holds(e.Start.Position) || !p.holds(e.End.Position) {
		return false
	}
	var center, axis r3.Vec
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		return true
	case surfacegeom.Circle3:
		center, axis = c.Center, c.Axis
	case surfacegeom.Arc3:
		center, axis = c.Center, c.Axis
	default:
		return false
	}
	exactAxis, ok := massmoment.ExactVec(axis)
	return ok && p.holds(center) && ratParallel3(p.normal, exactAxis)
}

func ratDot3(a, b [3]*big.Rat) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

func ratCross3(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		out[i] = new(big.Rat).Sub(proofbound.RatMul(a[j], b[k]), proofbound.RatMul(a[k], b[j]))
	}
	return out
}

// ratParallel3 reports whether a and b are parallel exactly, in either sense.
// A zero b is never parallel; a is the nonzero record normal.
func ratParallel3(a, b [3]*big.Rat) bool {
	if b[0].Sign() == 0 && b[1].Sign() == 0 && b[2].Sign() == 0 {
		return false
	}
	c := ratCross3(a, b)
	return c[0].Sign() == 0 && c[1].Sign() == 0 && c[2].Sign() == 0
}
