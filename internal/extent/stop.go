package extent

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const stopTol = 1e-9

// PlaneStop is the exact level c at which the sweep frame's plane
// O' + u*U' + v*V' + c*N' coincides with the plane a cap record denotes,
// {B(O + u*U + v*V + z*N) + t}, every held value read exactly. It holds for
// every (u, v) only when that plane is an exact level set of the sweep:
// m.U' = m.V' = 0 in rationals, m = (B*U) x (B*V). Then
// c = m.(P - O') / (m.N'), P = B(O + z*N) + t. k converts the record's axial
// displacement into a displacement of c, rounded up:
// max(|m.(B*N)| / |m.N'|, |m| / |m.N'|). It is exactly 1 when the frames and
// the basis are exactly orthonormal and the plane normal is exactly +-N'.
// ok is false when any input is not finite, m is zero, m.U' or m.V' is
// nonzero, m.N' is zero, or k is not finite.
func PlaneStop(frame r3.Frame, xform r3.Transform, z float64, sweep r3.Frame) (*big.Rat, float64, bool) {
	return PlaneStopAt(frame, xform, proofarith.FloatRat(z), sweep)
}

// PlaneStopAt is PlaneStop for the record's plane at the exact level zr
// rather than a held float: the level a displaced cap denotes. ok is false
// for a nil zr.
func PlaneStopAt(frame r3.Frame, xform r3.Transform, zr *big.Rat, sweep r3.Frame) (*big.Rat, float64, bool) {
	var (
		fu, fv, fn, fo = ratVec(frame.U()), ratVec(frame.V()), ratVec(frame.N()), ratVec(frame.Origin())
		b              = xform.Basis()
		bx, by, bz     = ratVec(b.EX), ratVec(b.EY), ratVec(b.EZ)
		t              = ratVec(xform.Translation())
		su, sv, sn, so = ratVec(sweep.U()), ratVec(sweep.V()), ratVec(sweep.N()), ratVec(sweep.Origin())
	)
	for _, v := range [][3]*big.Rat{fu, fv, fn, fo, bx, by, bz, t, su, sv, sn, so} {
		if v[0] == nil || v[1] == nil || v[2] == nil {
			return nil, 0, false
		}
	}
	if zr == nil {
		return nil, 0, false
	}
	// apply is B*(a) for the three-vector a.
	apply := func(a [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range out {
			out[i] = proofbound.RatAdd(
				proofbound.RatMul(bx[i], a[0]), proofbound.RatMul(by[i], a[1]), proofbound.RatMul(bz[i], a[2]))
		}
		return out
	}
	bu, bv, bn := apply(fu), apply(fv), apply(fn)
	m := ratCross(bu, bv)
	if m[0].Sign() == 0 && m[1].Sign() == 0 && m[2].Sign() == 0 {
		return nil, 0, false
	}
	if ratDot(m, su).Sign() != 0 || ratDot(m, sv).Sign() != 0 {
		return nil, 0, false
	}
	mn := ratDot(m, sn)
	if mn.Sign() == 0 {
		return nil, 0, false
	}
	var local [3]*big.Rat
	for i := range local {
		local[i] = proofbound.RatAdd(fo[i], proofbound.RatMul(zr, fn[i]))
	}
	p := apply(local)
	var rel [3]*big.Rat
	for i := range rel {
		rel[i] = new(big.Rat).Sub(proofbound.RatAdd(p[i], t[i]), so[i])
	}
	c := new(big.Rat).Quo(ratDot(m, rel), mn)
	mn2 := new(big.Rat).Mul(mn, mn)
	bnRatio := new(big.Rat).Quo(new(big.Rat).Abs(ratDot(m, bn)), new(big.Rat).Abs(mn))
	mRatio := new(big.Rat).Quo(ratDot(m, m), mn2)
	k := math.Max(proofbound.RatFloatUp(bnRatio), proofbound.RatSqrtUp(mRatio))
	if math.IsNaN(k) || math.IsInf(k, 0) {
		return nil, 0, false
	}
	return c, k, true
}

// StopLevelError is |held - (c + travel*offset)|, rounded up: the exact
// distance from a held stop level to the level the exact plane level c and the
// offset denote. It is +Inf when travel or offset is not finite.
func StopLevelError(c *big.Rat, travel, offset, held float64) float64 {
	t, o := proofarith.FloatRat(travel), proofarith.FloatRat(offset)
	if c == nil || t == nil || o == nil {
		return math.Inf(1)
	}
	return proofarith.RationalFloatError(proofbound.RatAdd(c, proofbound.RatMul(t, o)), held)
}

// ratVec reads a vector's components exactly; a component is nil when it is
// not finite.
func ratVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	return proofbound.RatAdd(
		proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		out[i] = new(big.Rat).Sub(proofbound.RatMul(a[j], b[k]), proofbound.RatMul(a[k], b[j]))
	}
	return out
}

// ExactlyParallel reports whether a and b are parallel exactly, in either
// sense: their cross product, computed in rationals from the held float
// components, is zero. A zero or non-finite vector is never parallel.
func ExactlyParallel(a, b r3.Vec) bool {
	av, bv := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	var ar, br [3]*big.Rat
	for i := range av {
		ar[i], br[i] = proofarith.FloatRat(av[i]), proofarith.FloatRat(bv[i])
		if ar[i] == nil || br[i] == nil {
			return false
		}
	}
	if (ar[0].Sign() == 0 && ar[1].Sign() == 0 && ar[2].Sign() == 0) ||
		(br[0].Sign() == 0 && br[1].Sign() == 0 && br[2].Sign() == 0) {
		return false
	}
	for i := range ar {
		j, k := (i+1)%3, (i+2)%3
		if new(big.Rat).Sub(proofbound.RatMul(ar[j], br[k]), proofbound.RatMul(ar[k], br[j])).Sign() != 0 {
			return false
		}
	}
	return true
}

// throughStopRound bounds the rounding in a through-all stop level.
func throughStopRound(origin, dir r3.Vec, hi, travel, held float64) float64 {
	o := [3]float64{origin.X, origin.Y, origin.Z}
	g := [3]float64{dir.X, dir.Y, dir.Z}
	base := new(big.Rat)
	for i := range o {
		oi, gi := proofarith.FloatRat(o[i]), proofarith.FloatRat(g[i])
		if oi == nil || gi == nil {
			return math.Inf(1)
		}
		base.Add(base, proofbound.RatMul(oi, gi))
	}
	h, t := proofarith.FloatRat(hi), proofarith.FloatRat(travel)
	if h == nil || t == nil {
		return math.Inf(1)
	}
	return proofarith.RationalFloatError(new(big.Rat).Mul(t, new(big.Rat).Sub(h, base)), held)
}

// RelativeStopTolerance scales the stop contact tolerance to the input.
func RelativeStopTolerance(scale float64) float64 {
	return stopTol * math.Max(1, math.Abs(scale))
}
