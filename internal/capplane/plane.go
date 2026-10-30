// Package capplane reads the exact plane a planar face's CapRecord denotes,
// every held value read as an exact rational, and decides plane identity and
// direction questions about it without rounding. A Loft cap's level proof
// (internal/loftrecord) and a Boolean's upstream-origin attribution
// (internal/boolupstream) read it.
package capplane

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Plane is an exact plane: the points x with Normal.(x - Point) = 0. Normal
// is nonzero and not normalized.
type Plane struct {
	normal, point [3]*big.Rat
}

// OfRecord reads the plane rec denotes at its held level Z,
// {B(O + u*U + v*V + Z*N) + t}, with B and t the linear part and translation
// of rec.Xform. ok is false when any held value is not finite or the mapped
// normal (B*U) x (B*V) is zero.
func OfRecord(rec *topo.CapRecord) (Plane, bool) {
	z := proofarith.FloatRat(rec.Z)
	if z == nil {
		return Plane{}, false
	}
	return ofRecordAt(rec, z)
}

// OfFace reads the exact plane face lies on: its CapRecord taken at the
// denoted level ZDenoted where the producer records one, and otherwise at the
// held level Z only when the face states a zero axial displacement. A face
// with no CapRecord, a displaced face with no denoted level, or a record
// OfRecord refuses has no exact plane, and ok is false.
func OfFace(face *topo.Face) (Plane, bool) {
	rec := face.CapRecord
	if rec == nil {
		return Plane{}, false
	}
	if rec.ZDenoted != nil {
		return ofRecordAt(rec, rec.ZDenoted)
	}
	if !face.HasAxialDelta || face.AxialDelta != 0 {
		return Plane{}, false
	}
	return OfRecord(rec)
}

func ofRecordAt(rec *topo.CapRecord, z *big.Rat) (Plane, bool) {
	b := rec.Xform.Basis()
	var vecs [7][3]*big.Rat
	for i, v := range []r3.Vec{rec.Frame.U(), rec.Frame.V(), rec.Frame.N(), rec.Frame.Origin(), b.EX, b.EY, b.EZ} {
		exact, ok := massmoment.ExactVec(v)
		if !ok {
			return Plane{}, false
		}
		vecs[i] = exact
	}
	t, okT := massmoment.ExactVec(rec.Xform.Translation())
	if !okT {
		return Plane{}, false
	}
	basis := [3][3]*big.Rat{vecs[4], vecs[5], vecs[6]}
	fu, fv, fn, fo := vecs[0], vecs[1], vecs[2], vecs[3]
	normal := Cross(apply(basis, fu), apply(basis, fv))
	if isZero(normal) {
		return Plane{}, false
	}
	var local [3]*big.Rat
	for i := range local {
		local[i] = proofbound.RatAdd(fo[i], proofbound.RatMul(z, fn[i]))
	}
	point := apply(basis, local)
	for i := range point {
		point[i] = proofbound.RatAdd(point[i], t[i])
	}
	return Plane{normal: normal, point: point}, true
}

// Axis is the exact direction B*N that frame's normal N takes under xform's
// linear part B: the direction a prism record in frame, placed by xform,
// sweeps its section along. ok is false when a held value is not finite or
// the direction is zero.
func Axis(frame r3.Frame, xform r3.Transform) ([3]*big.Rat, bool) {
	b := xform.Basis()
	var vecs [4][3]*big.Rat
	for i, v := range []r3.Vec{frame.N(), b.EX, b.EY, b.EZ} {
		exact, ok := massmoment.ExactVec(v)
		if !ok {
			return [3]*big.Rat{}, false
		}
		vecs[i] = exact
	}
	out := apply([3][3]*big.Rat{vecs[1], vecs[2], vecs[3]}, vecs[0])
	if isZero(out) {
		return [3]*big.Rat{}, false
	}
	return out, true
}

// Same reports whether p and o are one plane exactly: parallel normals and
// o's point on p.
func (p Plane) Same(o Plane) bool {
	return Parallel(p.normal, o.normal) && p.holdsExact(o.point)
}

// ParallelTo reports whether p's normal is parallel to o's exactly, in
// either sense: the two planes are the same plane or disjoint.
func (p Plane) ParallelTo(o Plane) bool { return Parallel(p.normal, o.normal) }

// Crosses reports whether direction d is not parallel to p exactly
// (Normal.d != 0). A surface that contains every line along d through its
// own points then meets p in a set of zero area.
func (p Plane) Crosses(d [3]*big.Rat) bool { return Dot(p.normal, d).Sign() != 0 }

// Sense is the sign of Normal.v for the held vector v read exactly: +1 when
// v points to p's normal side, -1 when it points away, 0 when v lies in p or
// is not finite.
func (p Plane) Sense(v r3.Vec) int {
	exact, ok := massmoment.ExactVec(v)
	if !ok {
		return 0
	}
	return Dot(p.normal, exact).Sign()
}

// Holds reports whether the held point x lies exactly on p.
func (p Plane) Holds(x r3.Vec) bool {
	exact, ok := massmoment.ExactVec(x)
	return ok && p.holdsExact(exact)
}

func (p Plane) holdsExact(x [3]*big.Rat) bool {
	var rel [3]*big.Rat
	for i := range rel {
		rel[i] = new(big.Rat).Sub(x[i], p.point[i])
	}
	return Dot(p.normal, rel).Sign() == 0
}

// HoldsEdge reports whether e's end vertices lie on p and its curve is a line
// between them or a circle or arc whose center lies on p and whose axis is
// parallel to p's normal.
func (p Plane) HoldsEdge(e *topo.Edge) bool {
	if e == nil || e.Start == nil || e.End == nil || !p.Holds(e.Start.Position) || !p.Holds(e.End.Position) {
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
	return ok && p.Holds(center) && Parallel(p.normal, exactAxis)
}

// HoldsFrame reports whether a Plane tag's frame is exactly p: its U and V
// orthogonal to p's normal, its N parallel to it, and its origin on p.
func (p Plane) HoldsFrame(frame r3.Frame) bool {
	tu, okU := massmoment.ExactVec(frame.U())
	tv, okV := massmoment.ExactVec(frame.V())
	tn, okN := massmoment.ExactVec(frame.N())
	return okU && okV && okN && Dot(p.normal, tu).Sign() == 0 && Dot(p.normal, tv).Sign() == 0 &&
		Parallel(p.normal, tn) && p.Holds(frame.Origin())
}

// Dot is a.b exactly.
func Dot(a, b [3]*big.Rat) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

// Cross is a x b exactly.
func Cross(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		out[i] = new(big.Rat).Sub(proofbound.RatMul(a[j], b[k]), proofbound.RatMul(a[k], b[j]))
	}
	return out
}

// Parallel reports whether a and b are parallel exactly, in either sense. A
// zero b is never parallel; a is a nonzero normal.
func Parallel(a, b [3]*big.Rat) bool {
	if isZero(b) {
		return false
	}
	return isZero(Cross(a, b))
}

func isZero(v [3]*big.Rat) bool {
	return v[0].Sign() == 0 && v[1].Sign() == 0 && v[2].Sign() == 0
}

// apply is basis * a, the columns of basis being the images of the three axes.
func apply(basis [3][3]*big.Rat, a [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = proofbound.RatAdd(proofbound.RatMul(basis[0][i], a[0]), proofbound.RatMul(basis[1][i], a[1]),
			proofbound.RatMul(basis[2][i], a[2]))
	}
	return out
}
