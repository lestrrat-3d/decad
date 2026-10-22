package patternrecord

import (
	"math/big"

	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PlaneMap is the exact in-plane map from one placed section frame to
// another: (u, v) ↦ A·(u, v) + T, with every entry an exact rational of the
// held floats. It exists only when the two frames' planes coincide exactly and
// the target frame is exactly orthonormal, and when A is a similarity, so a
// line maps to a line and a circle to a circle.
type PlaneMap struct {
	A [2][2]*big.Rat
	T [2]*big.Rat
	// ScaleLo and ScaleHi enclose √(A00² + A10²), the factor by which the map
	// scales every length. Both are 1 when the placement is an exact rotation.
	ScaleLo, ScaleHi float64
}

// NewPlaneMap reads the in-plane map from the section frame `from` placed by
// fromX onto the section frame `to` placed by toX. ok is false when the two
// planes do not coincide exactly, `to` is not exactly orthonormal, or the map
// is not a similarity: each is a refusal, never a rounding.
func NewPlaneMap(from r3.Frame, fromX r3.Transform, to r3.Frame, toX r3.Transform) (PlaneMap, bool) {
	u, v, o := placedAxes(from, fromX)
	tu, tv, to3 := placedAxes(to, toX)
	one := big.NewRat(1, 1)
	if ratVecDot(tu, tu).Cmp(one) != 0 || ratVecDot(tv, tv).Cmp(one) != 0 || ratVecDot(tu, tv).Sign() != 0 {
		return PlaneMap{}, false
	}
	n := ratVecCross(tu, tv)
	d := ratVec{new(big.Rat).Sub(o[0], to3[0]), new(big.Rat).Sub(o[1], to3[1]), new(big.Rat).Sub(o[2], to3[2])}
	if ratVecDot(u, n).Sign() != 0 || ratVecDot(v, n).Sign() != 0 || ratVecDot(d, n).Sign() != 0 {
		return PlaneMap{}, false
	}
	pm := PlaneMap{
		A: [2][2]*big.Rat{{ratVecDot(u, tu), ratVecDot(v, tu)}, {ratVecDot(u, tv), ratVecDot(v, tv)}},
		T: [2]*big.Rat{ratVecDot(d, tu), ratVecDot(d, tv)},
	}
	if pm.A[0][0].Cmp(pm.A[1][1]) != 0 || pm.A[0][1].Cmp(new(big.Rat).Neg(pm.A[1][0])) != 0 {
		return PlaneMap{}, false
	}
	sq := proofbound.RatAdd(proofbound.RatMul(pm.A[0][0], pm.A[0][0]), proofbound.RatMul(pm.A[1][0], pm.A[1][0]))
	pm.ScaleLo, pm.ScaleHi = proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
	return pm, pm.ScaleLo > 0
}

// ScaleDeviation is an upper bound on |scale − 1|.
func (pm PlaneMap) ScaleDeviation() float64 {
	return max(pm.ScaleHi-1, 1-pm.ScaleLo, 0)
}

// Exact returns the exact image of a held point.
func (pm PlaneMap) Exact(p Point2) (u, v *big.Rat) {
	pu, pv := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	u = proofbound.RatAdd(proofbound.RatMul(pm.A[0][0], pu), proofbound.RatMul(pm.A[0][1], pv), pm.T[0])
	v = proofbound.RatAdd(proofbound.RatMul(pm.A[1][0], pu), proofbound.RatMul(pm.A[1][1], pv), pm.T[1])
	return u, v
}

// Motion is the map as a PointMotion: the exact image rounded once.
func (pm PlaneMap) Motion() PointMotion {
	return func(p Point2) (Point2, float64, error) {
		u, v := pm.Exact(p)
		return heldPoint(proofbound.PointInterval(u), proofbound.PointInterval(v))
	}
}

func ratVecCross(a, b ratVec) ratVec {
	cross := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(proofbound.RatMul(a[i], b[j]), proofbound.RatMul(a[j], b[i]))
	}
	return ratVec{cross(1, 2), cross(2, 0), cross(0, 1)}
}

// PlaneHeight is how far the plane that lies level·N from the frame's origin
// (N the frame's own normal, level an exact offset along it) sits from the
// target frame's plane, measured along the target's exact unit normal, once
// the frame is placed by x and the target by toX. ok is false when the target
// frame is not exactly orthonormal. Two sweeps whose planes coincide have the
// same height at each end, exactly.
func PlaneHeight(frame r3.Frame, x r3.Transform, level *big.Rat, to r3.Frame, toX r3.Transform) (*big.Rat, bool) {
	_, _, o := placedAxes(frame, x)
	tu, tv, to3 := placedAxes(to, toX)
	one := big.NewRat(1, 1)
	if ratVecDot(tu, tu).Cmp(one) != 0 || ratVecDot(tv, tv).Cmp(one) != 0 || ratVecDot(tu, tv).Sign() != 0 {
		return nil, false
	}
	basis := x.Basis()
	n := ratVecOf(frame.N())
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	var lifted ratVec
	for k := range lifted {
		step := proofbound.RatAdd(proofbound.RatMul(ex[k], n[0]), proofbound.RatMul(ey[k], n[1]), proofbound.RatMul(ez[k], n[2]))
		lifted[k] = new(big.Rat).Sub(new(big.Rat).Add(o[k], proofbound.RatMul(level, step)), to3[k])
	}
	return ratVecDot(lifted, ratVecCross(tu, tv)), true
}
