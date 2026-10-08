package revolvemesh

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RecordedMeridian is the recorded plane-local geometry one carrier sample
// denotes: the point (U, V) when R is zero, or else the circle of radius R
// about centre (U, V), the circle's plane angle θ running from +u toward +v.
// UV and RBound are the walk's own proven bounds on those leaves
// (survey2d.SegmentWalk's StartBound/EndBound and RadiusBound).
type RecordedMeridian struct {
	U, V      float64
	UV        proofbound.WalkEndBound
	R, RBound float64
}

// HeldMeridian is how a clearance carrier holds the same sample: the exact
// surface its float fields denote,
//
//	Origin + Axis·(Z[0]·cos θ + Z[1]·sin θ) + Radial(φ)·(Rho[0] + Rho[1]·cos θ + Rho[2]·sin θ)
//
// with Radial(φ) = Radial[0]·cos φ + Radial[1]·sin φ and θ the recorded
// circle's own plane angle (zero for a point sample, whose Z and Rho[1:] are
// zero). Origin is an enclosure so a caller can state an exact sum of held
// floats, such as an anchor plus the axis times an axial coordinate, without
// rounding it.
type HeldMeridian struct {
	Origin proofbound.IvVec3
	Axis   r3.Vec
	Radial [2]r3.Vec
	Z      [2]proofbound.RatInterval
	Rho    [3]proofbound.RatInterval
}

// MeridianGap proves an upper bound on how far a clearance carrier's sample
// sits from the geometry the record denotes: the recorded meridian rec,
// re-expressed about the axis the record names and swept by the same angle φ,
// at every φ at once.
//
// The true side is SweptPointGap's construction over rational intervals: the
// frame's origin, U and V and the placement's basis and translation are exact
// leaves, the axis anchor and direction are widened by ab, and the recorded
// leaves by rec's own bounds, so the comparison covers the float (z, ρ)
// re-expression, a radius snapped onto the axis, a centre the build read as
// on the axis, and the axis's own anchor and direction error, beside the
// frame lift and placement rounding.
//
// Both sides are the same trigonometric form in cos θ, sin θ, cos φ and
// sin φ, so their difference is a fixed sum of coefficient vectors, each
// multiplied by a product of at most two of those, every one at most 1 in
// magnitude. The worst coordinate of the difference is therefore at most the
// sum of each coefficient's own worst coordinate, whatever the angles. That
// sum, over every coordinate, rounded upward and widened to a Euclidean
// length by Radius3D, is the answer. It is zero exactly where every
// coefficient matches the record exactly, and +Inf for a non-finite leaf.
func (l RevolveLift) MeridianGap(ab AxisBound, xform r3.Transform, held HeldMeridian, rec RecordedMeridian) float64 {
	basis := xform.Basis()
	vecs := [...]r3.Vec{
		l.Frame.Origin(), l.Frame.U(), l.Frame.V(), basis.EX, basis.EY, basis.EZ, xform.Translation(),
		held.Axis, held.Radial[0], held.Radial[1],
	}
	var iv [len(vecs)]proofbound.IvVec3
	for i, w := range vecs {
		enc, ok := proofbound.IvVec3Of(w)
		if !ok {
			return math.Inf(1)
		}
		iv[i] = enc
	}
	o, fu, fv, ex, ey, ez, tr := iv[0], iv[1], iv[2], iv[3], iv[4], iv[5], iv[6]
	hAxis, hR0, hR1 := iv[7], iv[8], iv[9]
	widen := func(x, w float64) (proofbound.RatInterval, bool) {
		r, b := proofarith.FloatRat(x), proofarith.FloatRat(math.Abs(w))
		if r == nil || b == nil {
			return proofbound.RatInterval{}, false
		}
		return proofbound.IntervalWiden(proofbound.PointInterval(r), b), true
	}
	var leaves [7]proofbound.RatInterval
	for i, pair := range [...][2]float64{
		{l.AU, ab.AU}, {l.AV, ab.AV}, {l.DU, ab.DU}, {l.DV, ab.DV}, {rec.U, rec.UV.U}, {rec.V, rec.UV.V}, {rec.R, rec.RBound},
	} {
		enc, ok := widen(pair[0], pair[1])
		if !ok {
			return math.Inf(1)
		}
		leaves[i] = enc
	}
	aU, aV, dU, dV, pu, pv, r := leaves[0], leaves[1], leaves[2], leaves[3], leaves[4], leaves[5], leaves[6]

	// The record's own meridian in the same form: centre (zc, ρc) and the
	// circle's radius rotated into axis coordinates by the true direction.
	du, dv := proofbound.IntervalSub(pu, aU), proofbound.IntervalSub(pv, aV)
	zc := proofbound.IntervalAdd(proofbound.IntervalMul(du, dU), proofbound.IntervalMul(dv, dV))
	rhoc := proofbound.IntervalSub(proofbound.IntervalMul(dv, dU), proofbound.IntervalMul(du, dV))
	tz := [2]proofbound.RatInterval{proofbound.IntervalMul(r, dU), proofbound.IntervalMul(r, dV)}
	trho := [3]proofbound.RatInterval{rhoc, proofbound.IntervalNeg(proofbound.IntervalMul(r, dV)), proofbound.IntervalMul(r, dU)}

	place := func(v proofbound.IvVec3) proofbound.IvVec3 {
		return proofbound.IvVec3Add(proofbound.IvVec3Add(proofbound.IvVec3Mul(ex, v[0]), proofbound.IvVec3Mul(ey, v[1])), proofbound.IvVec3Mul(ez, v[2]))
	}
	a3 := proofbound.IvVec3Add(o, proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, aU), proofbound.IvVec3Mul(fv, aV)))
	w := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, dU), proofbound.IvVec3Mul(fv, dV))
	e0 := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, proofbound.IntervalNeg(dV)), proofbound.IvVec3Mul(fv, dU))
	e1 := proofbound.IvVec3Cross(w, e0)
	tOrigin := proofbound.IvVec3Add(place(proofbound.IvVec3Add(a3, proofbound.IvVec3Mul(w, zc))), tr)
	tAxis := place(w)
	tRadial := [2]proofbound.IvVec3{place(e0), place(e1)}

	terms := []proofbound.IvVec3{proofbound.IvVec3Sub(held.Origin, tOrigin)}
	for k := range 2 {
		terms = append(terms, proofbound.IvVec3Sub(proofbound.IvVec3Mul(hAxis, held.Z[k]), proofbound.IvVec3Mul(tAxis, tz[k])))
	}
	for k, hr := range [2]proofbound.IvVec3{hR0, hR1} {
		for j := range 3 {
			terms = append(terms, proofbound.IvVec3Sub(proofbound.IvVec3Mul(hr, held.Rho[j]), proofbound.IvVec3Mul(tRadial[k], trho[j])))
		}
	}
	perCoord := 0.0
	for i := range 3 {
		sum := new(big.Rat)
		for _, t := range terms {
			sum.Add(sum, proofbound.IntervalAbsUpper(t[i]))
		}
		perCoord = math.Max(perCoord, proofbound.IntervalFloatError(proofbound.PointInterval(sum), 0))
	}
	if perCoord == 0 {
		return 0
	}
	return proofbound.Radius3D(perCoord)
}

// ZeroInterval is the exact zero, for a HeldMeridian coefficient a sample
// does not carry.
func ZeroInterval() proofbound.RatInterval { return proofbound.PointInterval(new(big.Rat)) }

// FloatInterval encloses the held float x exactly, or reports false for a
// non-finite x.
func FloatInterval(x float64) (proofbound.RatInterval, bool) {
	r := proofarith.FloatRat(x)
	if r == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.PointInterval(r), true
}
