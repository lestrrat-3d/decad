package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// segFilterCase drives the filter against the exact predicate it guards. Every
// case asserts the ONE property the filter owes: it never rejects a candidate
// onSegmentInterior3 would accept.
func requireFilterAgreesWithExact(t *testing.T, a, b, p r3.Vec, tau2 float64) bool {
	t.Helper()
	rejected := newSegFilter(a, b, tau2).tooFar(p)
	accepted := onSegmentInterior3(xptOf(a), xptOf(b), xptOf(p))
	require.False(t, rejected && accepted,
		`the filter is reject-only: it may never reject a candidate the exact predicate accepts`)
	return rejected
}

func TestSegFilterKeepsWhatTheExactPredicateAccepts(t *testing.T) {
	t.Parallel()
	a, b := r3.NewVec(0, 0, 0), r3.NewVec(10, 4, -3)
	tau2 := segAdmissionRadius2(0, 10)

	t.Run(`a vertex exactly on the segment survives the filter`, func(t *testing.T) {
		p := r3.NewVec(2.5, 1, -0.75) // exactly a quarter of the way along
		require.True(t, onSegmentInterior3(xptOf(a), xptOf(b), xptOf(p)),
			`the case is only meaningful while the exact predicate accepts p`)
		require.False(t, requireFilterAgreesWithExact(t, a, b, p, tau2),
			`a vertex the exact predicate accepts must reach it`)
	})

	t.Run(`an endpoint survives the filter`, func(t *testing.T) {
		// The exact predicate rejects the endpoints itself (the interior is
		// open); the filter must still hand them over rather than pre-empt it.
		require.False(t, requireFilterAgreesWithExact(t, a, b, a, tau2))
		require.False(t, requireFilterAgreesWithExact(t, a, b, b, tau2))
	})

	t.Run(`a vertex just inside the threshold survives the filter`, func(t *testing.T) {
		// tau is the admission radius; step off the segment's midpoint by
		// nine tenths of it, along a direction perpendicular to the segment.
		tau := math.Sqrt(tau2)
		mid := r3.NewVec(5, 2, -1.5)
		perp := b.Sub(a).Cross(r3.NewVec(0, 0, 1))
		off := perp.Scale(0.9 * tau / perp.Len())
		p := mid.Add(off)
		require.Positive(t, off.Len(), `the offset must be a real displacement`)
		require.Less(t, off.Len(), tau, `the case is only meaningful inside the threshold`)
		require.False(t, requireFilterAgreesWithExact(t, a, b, p, tau2),
			`a candidate inside the admission radius must reach the exact predicate`)
	})

	t.Run(`a vertex well outside the threshold is rejected`, func(t *testing.T) {
		mid := r3.NewVec(5, 2, -1.5)
		perp := b.Sub(a).Cross(r3.NewVec(0, 0, 1))
		p := mid.Add(perp.Scale(1e-6 / perp.Len()))
		require.True(t, requireFilterAgreesWithExact(t, a, b, p, tau2),
			`a candidate a micron off a ten-millimetre segment is provably not on it`)
	})

	t.Run(`a vertex beyond the far endpoint is rejected`, func(t *testing.T) {
		p := r3.NewVec(20, 8, -6) // on the carrier line, twice as far as b
		require.False(t, onSegmentInterior3(xptOf(a), xptOf(b), xptOf(p)),
			`the case is only meaningful while the exact predicate rejects p`)
		require.True(t, requireFilterAgreesWithExact(t, a, b, p, tau2),
			`the clamped projection must see a point past the far endpoint`)
	})

	t.Run(`a degenerate float segment abstains`, func(t *testing.T) {
		require.False(t, requireFilterAgreesWithExact(t, a, a, r3.NewVec(500, 500, 500), tau2),
			`a segment with no float length has nothing to project onto and must abstain`)
	})

	t.Run(`a non-finite candidate abstains`, func(t *testing.T) {
		inf := r3.NewVec(math.Inf(1), 0, 0)
		require.False(t, newSegFilter(a, b, tau2).tooFar(inf),
			`an overflowing candidate must fail closed, not reject`)
		nan := r3.NewVec(math.NaN(), 0, 0)
		require.False(t, newSegFilter(a, b, tau2).tooFar(nan),
			`a NaN candidate must fail closed, not reject`)
	})

	t.Run(`a segment whose squared length overflows abstains`, func(t *testing.T) {
		// |B−A| above roughly 1.34e154 saturates vv. The interior test c₁ < vv
		// then reads Inf < Inf, which is FALSE, so the branch clamps to B and
		// computes a perfectly finite |P−B|² for a projection sitting at the
		// exact middle of the segment.
		lo, hi := r3.NewVec(-1e154, 0, 0), r3.NewVec(1e154, 0, 0)
		mid := r3.NewVec(0, 0, 0)
		f := newSegFilter(lo, hi, segAdmissionRadius2(0, 1e154))
		require.True(t, math.IsInf(f.vv, 1),
			`the case is only meaningful once vv has saturated`)
		require.False(t, isNonFinite(f.tau2),
			`the case is only meaningful while the threshold is a real number`)
		require.True(t, onSegmentInterior3(xptOf(lo), xptOf(hi), xptOf(mid)),
			`the exact predicate accepts the midpoint`)
		require.False(t, requireFilterAgreesWithExact(t, lo, hi, mid, f.tau2),
			`a saturated vv must abstain, never clamp to an endpoint`)
	})
}

// TestSegFilterAbstainsOnEverySaturatedIntermediate pins the finiteness guard on
// each intermediate tooFar forms, one subtest per guard.
//
// The vv gate is what makes the other three guards rather than live soundness
// routes. Behind it a TRUE hit's ww, c₁ and |P−B|² all sit within a rounding of
// vv itself — |P−A| ≤ |B−A| for a point ON the segment, |c₁| ≤ √(ww·vv) by
// Cauchy–Schwarz, and |P−B|² = ww − 2c₁ + vv ≤ ww whenever c₁ ≥ vv picks the
// clamp — so no candidate the exact predicate accepts can saturate any of them
// by more than the last ulp of a value already at MaxFloat64. They are tested
// rather than argued away, because arguing them away is what failed here twice.
// ww is reachable through newSegFilter by a candidate PAST the far endpoint,
// which the exact predicate would reject anyway; c₁ and |P−B|² need a
// hand-built filter whose vv disagrees with its v, the only way to reach the
// state at all.
func TestSegFilterAbstainsOnEverySaturatedIntermediate(t *testing.T) {
	t.Parallel()
	a, b := r3.NewVec(0, 0, 0), r3.NewVec(100, 0, 0)
	tau2 := segAdmissionRadius2(0, 100)

	t.Run(`a saturated ww abstains`, func(t *testing.T) {
		// Reached through newSegFilter: a candidate one part in thirty past the
		// far endpoint of a 1.3e154 segment squares its distance from A past
		// MaxFloat64 while its distance from B stays finite.
		far := r3.NewVec(1.3e154, 0, 0)
		p := r3.NewVec(1.35e154, 0, 0)
		f := newSegFilter(r3.Vec{}, far, segAdmissionRadius2(0, 1.35e154))
		require.False(t, isNonFinite(f.vv), `the segment itself must stay finite`)
		require.True(t, math.IsInf(p.Sub(f.a).Dot(p.Sub(f.a)), 1),
			`the case is only meaningful once |P−A|² has saturated`)
		require.False(t, f.tooFar(p),
			`a saturated |P−A|² must abstain, whatever the branch would have computed`)
	})

	t.Run(`a saturated c1 abstains`, func(t *testing.T) {
		f := segFilter{a: a, b: b, v: r3.NewVec(math.MaxFloat64, 0, 0), vv: 1, tau2: tau2}
		p := r3.NewVec(1e154, 0, 0)
		require.True(t, math.IsInf(p.Sub(f.a).Dot(f.v), 1),
			`the case is only meaningful once w·v has saturated`)
		require.False(t, f.tooFar(p),
			`a saturated w·v decides no branch and must abstain`)
	})

	t.Run(`a saturated distance to the far endpoint abstains`, func(t *testing.T) {
		f := segFilter{a: a, b: r3.NewVec(-1.3e154, 0, 0), v: r3.NewVec(1, 0, 0), vv: 1, tau2: tau2}
		p := r3.NewVec(1.3e154, 0, 0)
		require.True(t, math.IsInf(p.Sub(f.b).Dot(p.Sub(f.b)), 1),
			`the case is only meaningful once |P−B|² has saturated`)
		require.False(t, f.tooFar(p),
			`a saturated |P−B|² must abstain rather than lean on a NaN comparison`)
	})

	t.Run(`a threshold the radius cannot state abstains`, func(t *testing.T) {
		require.False(t, newSegFilter(a, b, math.Inf(1)).tooFar(r3.NewVec(50, 1e6, 0)),
			`a +Inf threshold rejects nothing, however far the candidate sits`)
	})
}

// TestSegFilterNeverRejectsAnExactHit sweeps the exact predicate's accepting set
// itself: rational points constructed ON the segment, whose float roundings are
// what the filter actually sees. Every one must survive.
func TestSegFilterNeverRejectsAnExactHit(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	for range 4000 {
		a := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
		b := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
		xa, xb := xptOf(a), xptOf(b)
		// An exact interior point of the exact segment, at a rational parameter
		// no float64 can represent, so its rounding is a genuine approximation.
		xp := xlerp(xa, xb, big.NewInt(int64(rng.IntN(9999)+1)), big.NewInt(10000))
		p := xp.vec()
		require.True(t, onSegmentInterior3(xa, xb, xp),
			`the constructed point must be exactly interior to the exact segment`)

		maxAbs := 0.0
		for _, v := range []r3.Vec{a, b, p} {
			maxAbs = math.Max(maxAbs, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
		}
		require.False(t, newSegFilter(a, b, segAdmissionRadius2(0, maxAbs)).tooFar(p),
			`the filter must never reject the float rounding of a point exactly on the segment`)
	}
}

// TestSegFilterNeverRejectsAnExactHitAtEveryScale is the same property as
// TestSegFilterNeverRejectsAnExactHit over the whole float64 exponent range
// rather than a [−100, 100] coordinate box.
//
// It has to be a SWEEP rather than a witness. The evaluation leaves the
// normalized range at BOTH ends of the exponent range, and each end fails its
// own way.
//
// At the bottom the interior branch works at the (D·L)² scale — the square of
// the scale ww and vv work at — so it underflows across a wide band, whose two
// halves differ again: at the very bottom c₁² flushes to zero outright, while
// most of the band loses it gradually and keeps a shrinking number of
// significand bits.
//
// At the top, from about 2⁵¹¹, vv and then ww and c₁ saturate to +Inf while the
// threshold itself is still a real number, which is where the branch test
// c₁ < vv reads Inf < Inf and clamps a mid-segment projection to an endpoint.
// That band closes again above about 2⁵⁶³, where τ² saturates too and the
// filter rejects nothing at all.
//
// A case pinned to any one of those regimes says nothing about the others, so
// the sweep asserts mechanically that it visited each.
func TestSegFilterNeverRejectsAnExactHitAtEveryScale(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(19, 23))
	// A coordinate is a random magnitude within one binade of the scale, so
	// every component stays normal down to the bottom of the sweep and the
	// exponent alone moves.
	coord := func(s float64) float64 {
		x := (rng.Float64()*0.5 + 0.5) * s
		if rng.IntN(2) == 0 {
			return -x
		}
		return x
	}
	flushed, gradual, saturated := 0, 0, 0
	for e := -1020; e <= 1023; e++ {
		s := math.Ldexp(1, e)
		for range 8 {
			a := r3.NewVec(coord(s), coord(s), coord(s))
			b := r3.NewVec(coord(s), coord(s), coord(s))
			xa, xb := xptOf(a), xptOf(b)
			xp := xlerp(xa, xb, big.NewInt(int64(rng.IntN(9999)+1)), big.NewInt(10000))
			p := xp.vec()
			require.True(t, onSegmentInterior3(xa, xb, xp),
				`the constructed point must be exactly interior to the exact segment at 2^%d`, e)

			maxAbs := 0.0
			for _, v := range []r3.Vec{a, b, p} {
				maxAbs = math.Max(maxAbs, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
			}
			f := newSegFilter(a, b, segAdmissionRadius2(0, maxAbs))
			require.False(t, f.tooFar(p),
				`the filter must never reject the float rounding of a point exactly on the segment, at 2^%d`, e)

			w, v := p.Sub(a), b.Sub(a)
			ww, c1 := w.Dot(w), w.Dot(v)

			// Record the OVERFLOW regime: an intermediate saturated while the
			// threshold is still finite, so the filter could have rejected on
			// it. Above that band τ² saturates too and no case proves anything.
			if !isNonFinite(f.tau2) && (isNonFinite(f.vv) || isNonFinite(ww) || isNonFinite(c1)) {
				saturated++
			}

			// Record which underflow regime this case put the interior branch
			// in, counting only cases whose distance term clears the absolute
			// floor — below that the filter could not reject at any accuracy,
			// so such a case would prove nothing.
			if c1 <= 0 || c1 >= f.vv || !(ww > segFilterFloor) {
				continue
			}
			switch sq := c1 * c1; {
			case sq == 0:
				flushed++
			case sq < segFilterMinNormal:
				gradual++
			}
		}
	}
	require.Positive(t, flushed,
		`the sweep must reach scales where the (D·L)² intermediate flushes to zero`)
	require.Positive(t, gradual,
		`the sweep must reach scales where the (D·L)² intermediate underflows gradually`)
	require.Positive(t, saturated,
		`the sweep must reach scales where an intermediate saturates under a threshold that has not`)
}

// TestSegFilterRejectsTheOverwhelmingMajority is the reason the filter exists:
// on a mesh-spanning edge nearly every vertex must be turned away before the
// rational predicate ever runs.
func TestSegFilterRejectsTheOverwhelmingMajority(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	a, b := r3.NewVec(-50, -50, -50), r3.NewVec(50, 50, 50)
	f := newSegFilter(a, b, segAdmissionRadius2(0, 50))
	rejected := 0
	const n = 10000
	for range n {
		p := r3.NewVec(rng.Float64()*100-50, rng.Float64()*100-50, rng.Float64()*100-50)
		if f.tooFar(p) {
			rejected++
		}
	}
	require.Equal(t, n, rejected,
		`every vertex scattered through the mesh is provably off a single edge`)
}

func TestSegAdmissionRadiusCoversTheRoundingItMustCover(t *testing.T) {
	t.Parallel()
	t.Run(`the radius dominates the coordinate rounding it is derived from`, func(t *testing.T) {
		// Two roundings of a 100 mm coordinate, read as a 3D distance, is the
		// worst (1) allows; the published radius must exceed it.
		worst := 2 * radius3D(100*unitRoundoff)
		require.Greater(t, segAdmissionRadius2(0, 100), worst*worst,
			`tau must cover two coordinate roundings at the mesh's own scale`)
	})

	t.Run(`the pass slack is carried on top`, func(t *testing.T) {
		require.Greater(t, segAdmissionRadius2(1e-6, 100), segAdmissionRadius2(0, 100),
			`the grid slack widens the threshold, never narrows it`)
	})

	t.Run(`a threshold that underflows to zero still cannot reject`, func(t *testing.T) {
		// Below 2⁻⁵³⁷ the squared threshold underflows to zero. The filter's own
		// error floor carries the guarantee from there: it rejects nothing until
		// the distance passes 2⁻⁵⁰⁰, far above any tau that small.
		tau2 := segAdmissionRadius2(0, 0x1p-500)
		require.Zero(t, tau2, `the case is only meaningful once the threshold has underflowed`)
		f := newSegFilter(r3.Vec{}, r3.NewVec(0x1p-500, 0, 0), tau2)
		require.Positive(t, f.vv, `the case is only meaningful on a segment the filter will project onto`)
		require.False(t, f.tooFar(r3.NewVec(0, 0x1p-600, 0)),
			`the error floor must keep a rounding-scale candidate out of reach of a rejection`)
	})

	t.Run(`a threshold that overflows abstains instead of saturating`, func(t *testing.T) {
		// Above τ ≈ 1.34e154 the square saturates. +Inf is the reading that
		// rejects nothing, which is the only reading a saturated threshold has.
		require.True(t, math.IsInf(segAdmissionRadius2(0, 1e300), 1),
			`a threshold whose square overflows must reject nothing`)
		require.True(t, math.IsInf(segAdmissionRadius2(1e200, 100), 1),
			`the same holds when the slack is what overflows it`)
	})

	t.Run(`an argument outside the derivation abstains`, func(t *testing.T) {
		for _, tc := range []struct {
			name          string
			slack, maxAbs float64
		}{
			{`a NaN mesh extent`, 0, math.NaN()},
			{`a NaN slack`, math.NaN(), 100},
			{`an infinite mesh extent`, 0, math.Inf(1)},
			{`an infinite slack`, math.Inf(1), 100},
			// A negative slack is the one that would bite: it cancels the
			// rounding term instead of widening it, leaving a threshold BELOW
			// the proven requirement, which rejects true hits. newConformScan
			// builds its slack from a vector length and a positive cell width
			// and so cannot produce one; the guard makes the precondition total.
			{`a slack that cancels the rounding term`, -100 * 0x1p-50, 100},
			{`a negative mesh extent`, 0, -100},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.True(t, math.IsInf(segAdmissionRadius2(tc.slack, tc.maxAbs), 1),
					`a threshold this function cannot state must reject nothing`)
			})
		}
	})
}

// xhpBenchTriangle is the fixed triangle every xhp differential test and
// benchmark below sweeps against: a=(0.1,0.2,0.3), b=(1.7,0.35,-0.9),
// c=(-0.55,2.25,0.125).
func xhpBenchTriangle() (a, b, c xhp) {
	return xhpOf(r3.NewVec(0.1, 0.2, 0.3)),
		xhpOf(r3.NewVec(1.7, 0.35, -0.9)),
		xhpOf(r3.NewVec(-0.55, 2.25, 0.125))
}

// xhpGrid is the 8×8×8 probe grid every xhp differential test sweeps.
var xhpGrid = [8]float64{-2.5, -1, -0.125, 0, 0.1, 0.3, 1, 2.75}

// refPoint is an exact rational 3D point built with math/big.Rat directly —
// the same shape this package's exact kernel carried before this change.
// It is kept entirely apart from production's xpt/xhp kernel, private to this
// test file, so the differential tests below stay a genuine independent
// check rather than becoming circular once the production sign path is
// itself built on xhp.
type refPoint struct{ x, y, z *big.Rat }

func refPointOf(v r3.Vec) refPoint {
	return refPoint{mustRatOf(v.X), mustRatOf(v.Y), mustRatOf(v.Z)}
}

func refSub(a, b refPoint) refPoint {
	return refPoint{new(big.Rat).Sub(a.x, b.x), new(big.Rat).Sub(a.y, b.y), new(big.Rat).Sub(a.z, b.z)}
}

func refCross(a, b refPoint) refPoint {
	return refPoint{
		new(big.Rat).Sub(new(big.Rat).Mul(a.y, b.z), new(big.Rat).Mul(a.z, b.y)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.z, b.x), new(big.Rat).Mul(a.x, b.z)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)),
	}
}

func refDot(a, b refPoint) *big.Rat {
	s := new(big.Rat).Mul(a.x, b.x)
	s.Add(s, new(big.Rat).Mul(a.y, b.y))
	return s.Add(s, new(big.Rat).Mul(a.z, b.z))
}

func refOrientSign(a, b, c, d refPoint) int {
	return refDot(refCross(refSub(b, a), refSub(c, a)), refSub(d, a)).Sign()
}

func refLerp(a, b refPoint, t *big.Rat) refPoint {
	d := refSub(b, a)
	return refPoint{
		new(big.Rat).Add(a.x, new(big.Rat).Mul(t, d.x)),
		new(big.Rat).Add(a.y, new(big.Rat).Mul(t, d.y)),
		new(big.Rat).Add(a.z, new(big.Rat).Mul(t, d.z)),
	}
}

// TestXHPAgreesWithTheRationalOrientSign is the direct differential proof
// behind this change (fu163 §1): the homogeneous-integer sign must never
// disagree with an independent math/big.Rat computation of the same
// determinant, because disagreement would mean a topology decision could
// flip. It sweeps 512 float-derived probes and, separately, 512
// refLerp-derived probes at t = 37/91 — the investigation's own check, which
// passed on all 1024.
func TestXHPAgreesWithTheRationalOrientSign(t *testing.T) {
	t.Parallel()
	a, b, c := r3.NewVec(0.1, 0.2, 0.3), r3.NewVec(1.7, 0.35, -0.9), r3.NewVec(-0.55, 2.25, 0.125)
	ra, rb, rc := refPointOf(a), refPointOf(b), refPointOf(c)
	ha, hb, hc := xhpOf(a), xhpOf(b), xhpOf(c)
	tRat := big.NewRat(37, 91)
	tn, td := big.NewInt(37), big.NewInt(91)

	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := r3.NewVec(px, py, pz)
				rp := refPointOf(p)

				want := refOrientSign(ra, rb, rc, rp)
				got := xhpOrientSign(ha, hb, hc, xhpOf(p))
				require.Equalf(t, want, got, `direct probe (%v,%v,%v): the homogeneous and independent rational signs must agree`, px, py, pz)

				wantLerp := refOrientSign(ra, rb, rc, refLerp(ra, rp, tRat))
				gotLerp := xhpOrientSign(ha, hb, hc, xhpLerp(ha, xhpOf(p), tn, td))
				require.Equalf(t, wantLerp, gotLerp, `lerped probe (%v,%v,%v): the homogeneous and independent rational signs must agree`, px, py, pz)
			}
		}
	}
}

// TestXHPLerpDenotesTheSameCoordinate pins the positivity invariant the sign
// argument rests on: for every lerped probe xhpLerp produces, the exact
// rational it denotes (x/w, y/w, z/w) equals the coordinate an independent
// math/big.Rat lerp computes for the identical t, and w is strictly positive.
func TestXHPLerpDenotesTheSameCoordinate(t *testing.T) {
	t.Parallel()
	a := r3.NewVec(0.1, 0.2, 0.3)
	ra, ha := refPointOf(a), xhpOf(a)
	tRat := big.NewRat(37, 91)
	tn, td := big.NewInt(37), big.NewInt(91)

	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := r3.NewVec(px, py, pz)
				want := refLerp(ra, refPointOf(p), tRat)
				got := xhpLerp(ha, xhpOf(p), tn, td)
				require.Positive(t, got.w.Sign(), `the homogeneous denominator must stay positive`)
				gx, gy, gz := xhpRat(got)
				require.Zerof(t, gx.Cmp(want.x), `x at probe (%v,%v,%v)`, px, py, pz)
				require.Zerof(t, gy.Cmp(want.y), `y at probe (%v,%v,%v)`, px, py, pz)
				require.Zerof(t, gz.Cmp(want.z), `z at probe (%v,%v,%v)`, px, py, pz)
				wx, _ := want.x.Float64()
				wy, _ := want.y.Float64()
				wz, _ := want.z.Float64()
				require.Equalf(t, r3.Vec{X: wx, Y: wy, Z: wz}, xhpVec(got), `float rounding at probe (%v,%v,%v)`, px, py, pz)
			}
		}
	}
}

// TestOrientRatAgreesWithOrientSignExact checks the split orientVal was cut
// into: orientRat's materialised value and orientSignExact's plain integer
// sign must agree over the same probes, sign consumer and value consumer
// alike.
func TestOrientRatAgreesWithOrientSignExact(t *testing.T) {
	t.Parallel()
	a, b, c := r3.NewVec(0.1, 0.2, 0.3), r3.NewVec(1.7, 0.35, -0.9), r3.NewVec(-0.55, 2.25, 0.125)
	xa, xb, xc := xptOf(a), xptOf(b), xptOf(c)
	for _, px := range xhpGrid {
		for _, py := range xhpGrid {
			for _, pz := range xhpGrid {
				p := xptOf(r3.NewVec(px, py, pz))
				require.Equalf(t, orientSignExact(xa, xb, xc, p), orientRat(xa, xb, xc, p).Sign(),
					`probe (%v,%v,%v): the sign and the materialised value must agree`, px, py, pz)
			}
		}
	}
}

// xhpKeyOf is the four canonical integers joined the same way xpt.key joins
// them (both route the exact welding identity through xhpCanon) — kept local
// to this test file so the xhp differential tests need nothing from
// production's own key().
func xhpKeyOf(p xhp) string {
	return p.x.String() + `|` + p.y.String() + `|` + p.z.String() + `|` + p.w.String()
}

// TestXHPCanonIsAUniqueIdentity is what stands between this change and a
// silently cracked mesh: a homogeneous point has infinitely many spellings,
// and welding (boolean_mesh.go:998) is by exact identity, so two spellings of
// one point MUST canonicalise to one key. It takes a depth-2 lerped point,
// scales all four integers by 6 — a second, non-reduced spelling of the same
// coordinate — and asserts both canonicalise to the same key, the same
// four-tuple, and the same rational coordinate.
func TestXHPCanonIsAUniqueIdentity(t *testing.T) {
	t.Parallel()
	a, b, c := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	depth1 := xhpLerp(a, b, tn, td)
	depth2 := xhpLerp(depth1, c, tn, td)

	six := big.NewInt(6)
	scaled := xhp{
		x: new(big.Int).Mul(depth2.x, six),
		y: new(big.Int).Mul(depth2.y, six),
		z: new(big.Int).Mul(depth2.z, six),
		w: new(big.Int).Mul(depth2.w, six),
	}

	canonA, canonB := xhpCanon(depth2), xhpCanon(scaled)
	require.Equal(t, xhpKeyOf(canonA), xhpKeyOf(canonB),
		`two spellings of one point must canonicalise to the same key`)
	require.Equal(t, canonA.x.String(), canonB.x.String())
	require.Equal(t, canonA.y.String(), canonB.y.String())
	require.Equal(t, canonA.z.String(), canonB.z.String())
	require.Equal(t, canonA.w.String(), canonB.w.String())

	rx, ry, rz := xhpRat(depth2)
	crx, cry, crz := xhpRat(canonA)
	require.Zero(t, rx.Cmp(crx), `canonicalising must not change the denoted x`)
	require.Zero(t, ry.Cmp(cry), `canonicalising must not change the denoted y`)
	require.Zero(t, rz.Cmp(crz), `canonicalising must not change the denoted z`)
}

// TestXHPDenominatorStaysBounded is the guard against the one way this
// change can make the kernel SLOWER than the math/big.Rat it replaces:
// unreduced, a chain of xhpLerps grows multiplicatively (measured 380 bits at
// depth 1, 823 at depth 2, 14113 at depth 6, against 59-92 bits for the
// reduced big.Rat over the same chain). Stripping the common power of two
// after every lerp is what keeps it bounded — measured 462 bits at depth 6 —
// without paying xhpCanon's full GCD.
func TestXHPDenominatorStaysBounded(t *testing.T) {
	t.Parallel()
	p := xhpOf(r3.NewVec(0.1, 0.2, 0.3))
	targets := []r3.Vec{
		r3.NewVec(1.7, 0.35, -0.9),
		r3.NewVec(-0.55, 2.25, 0.125),
		r3.NewVec(3.3, -1.1, 0.7),
		r3.NewVec(-2.2, 0.05, 5.5),
		r3.NewVec(0.9, -3.3, 1.1),
		r3.NewVec(2.05, 1.9, -0.45),
	}
	tn, td := big.NewInt(37), big.NewInt(91)
	for _, target := range targets {
		p = xhpStripTwos(xhpLerp(p, xhpOf(target), tn, td))
	}
	require.Less(t, p.w.BitLen(), 4096,
		`the stripped denominator must stay far below the unreduced 14113-bit growth at the same depth`)
}

// BenchmarkOrientSignExact measures one exact orient sign over the
// homogeneous-integer representation on float-derived points (fu163 §9's
// cost guard: the math/big.Rat baseline this replaces was 14.6us/206
// allocs/op).
func BenchmarkOrientSignExact(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	p := xhpOf(r3.NewVec(0.3, -1, 2.75))
	b.ResetTimer()
	for b.Loop() {
		xhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkOrientSignExactLerped is BenchmarkOrientSignExact with an
// xhpLerp-derived probe in place of a float-derived one (baseline:
// 15.0us/199 allocs/op — indistinguishable from the dyadic case, which is
// what disproves the "non-dyadic denominators" half of the original claim).
func BenchmarkOrientSignExactLerped(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	p := xhpLerp(tb, tc, tn, td)
	b.ResetTimer()
	for b.Loop() {
		xhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkPlaneCrossingChain measures the planeCrossings → predicate path:
// two lerps feeding one orient sign (baseline: 26.1us/348 allocs/op).
func BenchmarkPlaneCrossingChain(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	b.ResetTimer()
	for b.Loop() {
		p1 := xhpStripTwos(xhpLerp(ta, tb, tn, td))
		p2 := xhpStripTwos(xhpLerp(tb, tc, tn, td))
		xhpOrientSign(ta, p1, p2, tc)
	}
}

// BenchmarkLerpDepth3Orient is the growth-control benchmark: three nested
// lerps, each stripped of its common power of two before feeding the next,
// then one orient sign (baseline: 31.4us/432 allocs/op; the arrangement to
// avoid — a full xhpCanon on every construction instead of xhpStripTwos —
// measured 23.4us, only 1.3x, per this file's xhpCanon doc comment).
func BenchmarkLerpDepth3Orient(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	b.ResetTimer()
	for b.Loop() {
		p := xhpStripTwos(xhpLerp(ta, tb, tn, td))
		p = xhpStripTwos(xhpLerp(p, tc, tn, td))
		p = xhpStripTwos(xhpLerp(p, ta, tn, td))
		xhpOrientSign(ta, tb, tc, p)
	}
}

// BenchmarkExactVertexKey measures the canonicalise-and-key cost paid once
// per welded vertex, on a depth-2 lerped point (baseline: math/big.Rat's
// RatString with no canonicalisation step at all, 1.5us/20 allocs/op — the
// canonical key legitimately costs more per vertex than the old spelling
// did, because welding correctness, not speed, is what the canonical form
// buys).
func BenchmarkExactVertexKey(b *testing.B) {
	ta, tb, tc := xhpBenchTriangle()
	tn, td := big.NewInt(37), big.NewInt(91)
	p1 := xhpStripTwos(xhpLerp(ta, tb, tn, td))
	p2 := xhpStripTwos(xhpLerp(p1, tc, tn, td))
	b.ResetTimer()
	for b.Loop() {
		_ = xhpKeyOf(xhpCanon(p2))
	}
}

// This section is fu158 task 1's own proof obligation
// (.tmp/followup-tasks/fu158-tasks.md §4/§5): floatInterval's five operations
// and its contains/disjoint tests, first on hand-picked values, then a
// randomized proof that its arithmetic really does enclose the exact
// (big.Rat) value of the same expression — the same "reject-only, never
// argued" discipline segFilter's own tests above pin for the conforming
// pass's segment filter.

// TestFloatIntervalArithmeticSanity exercises floatInterval's own five
// operations and its contains/disjoint tests directly, on hand-picked values,
// ahead of the randomized enclosure proof below.
func TestFloatIntervalArithmeticSanity(t *testing.T) {
	t.Parallel()
	a := floatInterval{lo: 1, hi: 2}
	b := floatInterval{lo: 3, hi: 4}

	require.True(t, a.disjoint(b))
	require.False(t, a.disjoint(floatInterval{lo: 1.5, hi: 5}))
	require.True(t, a.contains(1.5))
	require.False(t, a.contains(2.5))

	sum := a.add(b)
	require.LessOrEqual(t, sum.lo, 4.0)
	require.GreaterOrEqual(t, sum.hi, 6.0)

	diff := b.sub(a)
	require.LessOrEqual(t, diff.lo, 1.0)
	require.GreaterOrEqual(t, diff.hi, 3.0)

	prod := a.mul(b)
	require.LessOrEqual(t, prod.lo, 3.0)
	require.GreaterOrEqual(t, prod.hi, 8.0)

	quot := b.div(a)
	require.LessOrEqual(t, quot.lo, 1.5)
	require.GreaterOrEqual(t, quot.hi, 4.0)

	straddling := floatInterval{lo: -1, hi: 1}
	require.True(t, a.div(straddling).abstains(),
		`a denominator interval containing zero must abstain, never resolve`)

	require.True(t, fivPoint(math.NaN()).abstains())
	require.True(t, fivPoint(math.Inf(1)).abstains())
	require.True(t, floatInterval{lo: math.Inf(1), hi: math.Inf(1)}.add(a).abstains())
}

// TestTriTriIntervalEnclosesExact is fu158 task 1's proof obligation: for a
// few thousand pseudo-random float64 triples, the interval evaluation of
// a*b - c*d and of a/(a-b) must enclose the big.Rat value of the very same
// expression — checked as a big.Rat comparison, never a float one, so the
// test cannot pass by the same rounding the filter has to survive.
func TestTriTriIntervalEnclosesExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	sample := func() float64 { return (rng.Float64()*2 - 1) * 1000 }

	for range 5000 {
		a, b, c, d := sample(), sample(), sample(), sample()

		got := fivPoint(a).mul(fivPoint(b)).sub(fivPoint(c).mul(fivPoint(d)))
		exact := new(big.Rat).Sub(
			new(big.Rat).Mul(mustRatOf(a), mustRatOf(b)),
			new(big.Rat).Mul(mustRatOf(c), mustRatOf(d)),
		)
		require.False(t, got.abstains(), `a finite product/difference must never abstain`)
		require.LessOrEqual(t, mustRatOf(got.lo).Cmp(exact), 0)
		require.GreaterOrEqual(t, mustRatOf(got.hi).Cmp(exact), 0)

		if a == b {
			continue // a/(a-b) is undefined; the filter's own div guard covers it
		}
		gotDiv := fivPoint(a).div(fivPoint(a).sub(fivPoint(b)))
		if gotDiv.abstains() {
			continue // an abstained bound trivially encloses every value
		}
		exactDiv := new(big.Rat).Quo(mustRatOf(a), new(big.Rat).Sub(mustRatOf(a), mustRatOf(b)))
		require.LessOrEqual(t, mustRatOf(gotDiv.lo).Cmp(exactDiv), 0)
		require.GreaterOrEqual(t, mustRatOf(gotDiv.hi).Cmp(exactDiv), 0)
	}
}

// TestHomogeneousProjectedCrossMatchesRational verifies that the loft audit's
// cached homogeneous projection path returns the same exact orientation value
// as the existing rational projection path on every coordinate-plane choice.
func TestHomogeneousProjectedCrossMatchesRational(t *testing.T) {
	t.Parallel()
	points := [3]xpt{
		xptOf(r3.NewVec(0.1, 2.0, -3.5)),
		xptOf(r3.NewVec(4.25, -1.5, 0.75)),
		xptOf(r3.NewVec(-2.0, 3.125, 5.5)),
	}
	for _, axes := range [][2]int{{0, 1}, {2, 0}, {1, 2}} {
		hom := [3]xp2{
			newXP2FromXpt(points[0], axes[0], axes[1]),
			newXP2FromXpt(points[1], axes[0], axes[1]),
			newXP2FromXpt(points[2], axes[0], axes[1]),
		}
		rat := [3]xp2{
			newXP2(ratCoordOf(points[0], axes[0]), ratCoordOf(points[0], axes[1])),
			newXP2(ratCoordOf(points[1], axes[0]), ratCoordOf(points[1], axes[1])),
			newXP2(ratCoordOf(points[2], axes[0]), ratCoordOf(points[2], axes[1])),
		}
		want := cross2x(rat[0], rat[1], rat[2])
		got := cross2x(hom[0], hom[1], hom[2])
		require.Zero(t, got.Cmp(want), "projection axes %v must preserve the exact cross product", axes)
		require.Equal(t, cross2xSign(rat[0], rat[1], rat[2]), cross2xSign(hom[0], hom[1], hom[2]))
	}
}

// referenceMeshParityContext is the PRE-OPTIMIZATION reference copy of
// meshParityContext, captured verbatim before the query projection was hoisted
// out of the triangle loop: same arithmetic, same branch order, same returns,
// and the same per-triangle newXP2 construction the hoist removes.
//
// It is frozen on purpose. A later production edit must NEVER be mirrored into
// this function — the whole value of the comparison is that the reference
// still states the behaviour production had before the change. If production
// and this function disagree, production changed a classification and the
// change is wrong, not this copy.
func referenceMeshParityContext(ctx context.Context, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	for _, ray := range axisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			tri := tris[ti]
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			pa := newXP2(ratCoordOf(p, ray.u), ratCoordOf(p, ray.v))
			qa := newXP2(mustRatOf(coordOf(a, ray.u)), mustRatOf(coordOf(a, ray.v)))
			qb := newXP2(mustRatOf(coordOf(b, ray.u)), mustRatOf(coordOf(b, ray.v)))
			qc := newXP2(mustRatOf(coordOf(c, ray.u)), mustRatOf(coordOf(c, ray.v)))
			s1 := cross2xSign(qa, qb, pa)
			s2 := cross2xSign(qb, qc, pa)
			s3 := cross2xSign(qc, qa, pa)
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue // strictly outside the projection
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				// On the projected boundary: the ray may graze an edge or a
				// vertex, and the count would be unreliable — try another axis.
				ambiguous = true
				break
			}
			// Strictly inside the projection: the projected area is nonzero,
			// so the plane normal's swept component cannot vanish.
			xa, xb, xc := xptOf(a), xptOf(b), xptOf(c)
			n := xcross(xsub(xb, xa), xsub(xc, xa))
			nAxis := xIntCoordOf(n, ray.axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			tNum := xdotNum(xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.dir; {
			case s > 0:
				crossings++
			case tNum.Sign() == 0:
				onBoundary = true
			}
			if onBoundary {
				break
			}
		}
		if onBoundary {
			return false, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, nil
	}
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, ErrBooleanFailed)
}

// parityCubeVerts returns the eight unit-cube corners translated by o, in the
// index order every cube fixture in this file assumes.
func parityCubeVerts(o r3.Vec) []r3.Vec {
	return []r3.Vec{
		r3.NewVec(o.X, o.Y, o.Z),
		r3.NewVec(o.X+1, o.Y, o.Z),
		r3.NewVec(o.X+1, o.Y+1, o.Z),
		r3.NewVec(o.X, o.Y+1, o.Z),
		r3.NewVec(o.X, o.Y, o.Z+1),
		r3.NewVec(o.X+1, o.Y, o.Z+1),
		r3.NewVec(o.X+1, o.Y+1, o.Z+1),
		r3.NewVec(o.X, o.Y+1, o.Z+1),
	}
}

// parityCubeTris returns the twelve consistently outward-wound cube facets,
// with every vertex index shifted by base so several cubes can share one
// vertex buffer.
func parityCubeTris(base int) [][3]int {
	src := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	out := make([][3]int, len(src))
	for i, t := range src {
		out[i] = [3]int{t[0] + base, t[1] + base, t[2] + base}
	}
	return out
}

// parityFar translates a fixture 2^20 along every axis: far from the origin,
// yet every fixture coordinate stays exactly representable, so the translated
// run is a true repeat of the untranslated one.
const parityFar = 1048576

// parityBenchCubes is how many disjoint closed cubes the benchmark mesh holds —
// enough to carry well past 1,024 triangles.
const parityBenchCubes = 96

// parityTetraVerts returns the four corners of the closed unit tetrahedron.
func parityTetraVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
	}
}

// parityTetraTris returns the tetrahedron's four outward-wound facets.
func parityTetraTris() [][3]int {
	return [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
}

// paritySoupVerts returns a triangle soup's vertices whose first entry is the
// origin every facet shares.
func paritySoupVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
		r3.NewVec(1, 1, 1),
	}
}

// paritySoupTris returns facets that all carry vertex 0, so a query at the
// origin projects onto a facet vertex on every axis and the kernel must refuse
// rather than count an unreliable crossing.
func paritySoupTris() [][3]int {
	return [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {0, 1, 4}, {0, 4, 2}}
}

// parityIdentitySubset selects every facet in stored order.
func parityIdentitySubset(n int) []int {
	out := make([]int, n)
	for i := range n {
		out[i] = i
	}
	return out
}

// parityStrideSubset selects every facet exactly once in a deterministic
// rotated order. stride must be coprime with n, which the caller picks.
func parityStrideSubset(n, stride int) []int {
	out := make([]int, n)
	for i := range n {
		out[i] = (i * stride) % n
	}
	return out
}

// parityReverseWinding flips every facet's orientation, turning an outward
// mesh into an inward one without moving a single coordinate.
func parityReverseWinding(tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for i, t := range tris {
		out[i] = [3]int{t[0], t[2], t[1]}
	}
	return out
}

// parityRat is the offset o plus num/den, exactly — a query coordinate no
// float64 represents. Every fixture offset in this file is a whole number, so
// the integer conversion loses nothing.
func parityRat(o float64, num, den int64) *big.Rat {
	return new(big.Rat).Add(big.NewRat(int64(o), 1), big.NewRat(num, den))
}

// parityCase is one query point under a stable name.
type parityCase struct {
	name string
	p    xpt
}

// parityCubeQueries builds the query points for the unit cube translated by o.
// The Nextafter pair straddles that cube's own x = o.X face rather than a fixed
// coordinate, so the sub-ulp fixture keeps its meaning after translation.
func parityCubeQueries(o r3.Vec) []parityCase {
	v := parityCubeVerts(o)
	return []parityCase{
		{`interior`, xptOf(r3.NewVec(o.X+0.25, o.Y+0.375, o.Z+0.5625))},
		{`exterior`, xptOf(r3.NewVec(o.X+2, o.Y+0.375, o.Z+0.5625))},
		{`face interior`, xptOf(r3.NewVec(o.X, o.Y+0.25, o.Z+0.375))},
		{`edge`, xptOf(r3.NewVec(o.X, o.Y, o.Z+0.375))},
		{`vertex`, xptOf(v[0])},
		{`one ulp inside the face`, xptOf(r3.NewVec(math.Nextafter(o.X, o.X+1), o.Y+0.25, o.Z+0.375))},
		{`one ulp outside the face`, xptOf(r3.NewVec(math.Nextafter(o.X, o.X-1), o.Y+0.25, o.Z+0.375))},
		{`facet centroid`, xCentroid(xptOf(v[0]), xptOf(v[2]), xptOf(v[1]))},
		{`rational interior`, xptFromRat(
			parityRat(o.X, 1, 3), parityRat(o.Y, 1, 7), parityRat(o.Z, 5, 11))},
	}
}

// paritySentinels are the sentinels a parity answer could conceivably wrap.
// Every arm must agree on each one, so a candidate cannot swap which failure it
// reports while keeping the text.
var paritySentinels = []error{ErrBooleanFailed, ErrUnsupported, ErrDegenerate, context.Canceled, context.DeadlineExceeded}

// parityOutcome is one kernel's complete answer, so two arms compare field by
// field rather than through three separate returns.
type parityOutcome struct {
	inside     bool
	onBoundary bool
	err        error
}

// requireParityOutcome requires one arm to reproduce the reference exactly:
// both booleans, whether an error came back, which sentinel it wraps, and its
// exact text. arm names the arm so a failure says which one drifted.
func requireParityOutcome(t *testing.T, arm string, want, got parityOutcome) {
	t.Helper()
	require.Equal(t, want.err == nil, got.err == nil,
		`%s: error presence must match the reference (reference %v, candidate %v)`, arm, want.err, got.err)
	if want.err != nil {
		require.Equal(t, want.err.Error(), got.err.Error(), `%s: error text must match the reference`, arm)
		for _, sentinel := range paritySentinels {
			require.Equal(t, errors.Is(want.err, sentinel), errors.Is(got.err, sentinel),
				`%s: sentinel identity must match the reference for %v`, arm, sentinel)
		}
	}
	require.Equal(t, want.inside, got.inside, `%s: inside must match the reference`, arm)
	require.Equal(t, want.onBoundary, got.onBoundary, `%s: onBoundary must match the reference`, arm)
}

// parityTetraQueries builds the query points for the unit tetrahedron.
func parityTetraQueries() []parityCase {
	v := parityTetraVerts()
	return []parityCase{
		{`interior`, xptOf(r3.NewVec(0.125, 0.1875, 0.25))},
		{`exterior`, xptOf(r3.NewVec(2, 0.1875, 0.25))},
		{`slanted face`, xptOf(r3.NewVec(0.25, 0.25, 0.5))},
		{`vertex`, xptOf(r3.NewVec(0, 0, 0))},
		{`facet centroid`, xCentroid(xptOf(v[1]), xptOf(v[2]), xptOf(v[3]))},
		{`rational interior`, xptFromRat(big.NewRat(1, 7), big.NewRat(1, 11), big.NewRat(1, 13))},
	}
}

// requireParityMatches runs the frozen reference and every production entry
// point on one fixture and requires them all to agree. The prepared kernel is
// asked twice on ONE cache — once with it empty, once with every projection it
// needs already materialized — because a cached entry rebuilt wrong or used as
// an arithmetic destination would only show on the warm pass. It returns the
// reference's own answer so a caller can assert a known outcome on top.
func requireParityMatches(ctx context.Context, t *testing.T, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
	want := parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr}

	gotIn, gotBoundary, gotErr := meshParityContext(ctx, p, verts, tris, subset)
	requireParityOutcome(t, `raw wrapper`, want,
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})

	prepared := newParityMesh(verts, tris)
	coldIn, coldBoundary, coldErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared cold`, want,
		parityOutcome{inside: coldIn, onBoundary: coldBoundary, err: coldErr})
	warmIn, warmBoundary, warmErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared warm`, want,
		parityOutcome{inside: warmIn, onBoundary: warmBoundary, err: warmErr})

	return wantIn, wantBoundary
}

// requireCubeQueriesMatch replays every cube query against one facet buffer and
// requires the production kernel to reproduce the reference on all of them.
func requireCubeQueriesMatch(ctx context.Context, t *testing.T, o r3.Vec, verts []r3.Vec, tris [][3]int, subset []int) {
	t.Helper()
	for _, c := range parityCubeQueries(o) {
		t.Run(c.name, func(t *testing.T) {
			requireParityMatches(ctx, t, c.p, verts, tris, subset)
		})
	}
}

func TestParityQueryProjectionMatchesReference(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	origin := r3.NewVec(0, 0, 0)
	farVec := r3.NewVec(parityFar, parityFar, parityFar)

	t.Run(`unit cube`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, subset)

		// The known outcomes, asserted independently of the reference.
		inside, onBoundary := requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, subset)
		require.True(t, inside, `(0.25, 0.375, 0.5625) is interior to the unit cube`)
		require.False(t, onBoundary, `an interior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			xptOf(r3.NewVec(2, 0.375, 0.5625)), verts, tris, subset)
		require.False(t, inside, `(2, 0.375, 0.5625) is outside the unit cube`)
		require.False(t, onBoundary, `an exterior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0, 0.25, 0.375)), verts, tris, subset)
		require.False(t, inside, `a boundary point is not reported inside`)
		require.True(t, onBoundary, `(0, 0.25, 0.375) lies on the cube's x = 0 face`)
	})

	t.Run(`cube translated far from the origin`, func(t *testing.T) {
		verts, tris := parityCubeVerts(farVec), parityCubeTris(0)
		requireCubeQueriesMatch(ctx, t, farVec, verts, tris, parityIdentitySubset(len(tris)))
	})

	t.Run(`reversed winding`, func(t *testing.T) {
		verts := parityCubeVerts(origin)
		tris := parityReverseWinding(parityCubeTris(0))
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, parityIdentitySubset(len(tris)))
	})

	t.Run(`permuted subset order`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, parityStrideSubset(len(tris), 7))
	})

	t.Run(`tetrahedron`, func(t *testing.T) {
		verts, tris := parityTetraVerts(), parityTetraTris()
		queries := parityTetraQueries()
		for _, order := range []struct {
			name   string
			facets [][3]int
			subset []int
		}{
			{`stored order`, tris, parityIdentitySubset(len(tris))},
			{`permuted order`, tris, parityStrideSubset(len(tris), 3)},
			{`reversed winding`, parityReverseWinding(tris), parityIdentitySubset(len(tris))},
		} {
			t.Run(order.name, func(t *testing.T) {
				for _, c := range queries {
					t.Run(c.name, func(t *testing.T) {
						requireParityMatches(ctx, t, c.p, verts, order.facets, order.subset)
					})
				}
			})
		}
	})

	t.Run(`two disjoint cubes in one buffer`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		first, rest := parityIdentitySubset(12), parityStrideSubset(12, 7)
		for i := range rest {
			rest[i] += 12
		}

		t.Run(`first component`, func(t *testing.T) {
			requireCubeQueriesMatch(ctx, t, origin, verts, tris, first)
		})
		t.Run(`second component`, func(t *testing.T) {
			requireCubeQueriesMatch(ctx, t, second, verts, tris, rest)
		})
		t.Run(`first query against the second component`, func(t *testing.T) {
			inside, onBoundary := requireParityMatches(ctx, t,
				xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, rest)
			require.False(t, inside, `a point in the first cube is outside the second`)
			require.False(t, onBoundary, `it is not on the second cube's boundary either`)
		})
	})

	t.Run(`empty subset`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		inside, onBoundary := requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, nil)
		require.False(t, inside, `no facet can enclose the query`)
		require.False(t, onBoundary, `no facet can carry the query`)
	})

	t.Run(`context canceled before entry`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		p := xptOf(r3.NewVec(0.25, 0.375, 0.5625))

		_, _, wantErr := referenceMeshParityContext(canceled, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, context.Canceled, `the reference reports the cancellation`)
		requireParityMatches(canceled, t, p, verts, tris, subset)

		// An empty subset never enters the triangle loop, so cancellation is
		// not observed there — on either side of the change.
		_, _, emptyErr := referenceMeshParityContext(canceled, p, verts, tris, nil)
		require.NoError(t, emptyErr, `an empty subset returns before any context check`)
		requireParityMatches(canceled, t, p, verts, tris, nil)
	})

	t.Run(`every ray hits a facet vertex`, func(t *testing.T) {
		verts, tris := paritySoupVerts(), paritySoupTris()
		subset := parityIdentitySubset(len(tris))
		p := xptOf(r3.NewVec(0, 0, 0))

		_, _, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, ErrBooleanFailed, `the reference refuses rather than guessing`)
		require.EqualError(t, wantErr,
			`decad: boolean operation failed: every parity ray was ambiguous`,
			`the reference names the ambiguity`)
		requireParityMatches(ctx, t, p, verts, tris, subset)
	})
}

// parityNamedSubset is one facet selection under a stable name.
type parityNamedSubset struct {
	name   string
	facets []int
}

// parityReuseFixture is one geometry with every subset and query the reuse test
// replays against a SINGLE prepared object built from it.
type parityReuseFixture struct {
	name    string
	verts   []r3.Vec
	tris    [][3]int
	subsets []parityNamedSubset
	queries []parityCase
}

// parityReuseFixtures returns every geometry, subset and query fixture the
// reference comparison uses, regrouped so each geometry is a single prepared
// object the whole group shares.
func parityReuseFixtures() []parityReuseFixture {
	origin := r3.NewVec(0, 0, 0)
	farVec := r3.NewVec(parityFar, parityFar, parityFar)
	second := r3.NewVec(3, 0, 0)
	cube := parityCubeTris(0)
	tetra := parityTetraTris()

	identity12 := parityNamedSubset{`stored order`, parityIdentitySubset(12)}
	stride12 := parityNamedSubset{`permuted order`, parityStrideSubset(12, 7)}

	secondCube := parityStrideSubset(12, 7)
	for i := range secondCube {
		secondCube[i] += 12
	}

	return []parityReuseFixture{
		{
			name:    `unit cube`,
			verts:   parityCubeVerts(origin),
			tris:    cube,
			subsets: []parityNamedSubset{identity12, stride12},
			queries: parityCubeQueries(origin),
		},
		{
			name:    `cube translated far from the origin`,
			verts:   parityCubeVerts(farVec),
			tris:    cube,
			subsets: []parityNamedSubset{identity12},
			queries: parityCubeQueries(farVec),
		},
		{
			name:    `reversed winding`,
			verts:   parityCubeVerts(origin),
			tris:    parityReverseWinding(cube),
			subsets: []parityNamedSubset{identity12},
			queries: parityCubeQueries(origin),
		},
		{
			name:  `tetrahedron`,
			verts: parityTetraVerts(),
			tris:  tetra,
			subsets: []parityNamedSubset{
				{`stored order`, parityIdentitySubset(len(tetra))},
				{`permuted order`, parityStrideSubset(len(tetra), 3)},
			},
			queries: parityTetraQueries(),
		},
		{
			name:  `two disjoint cubes in one buffer`,
			verts: append(parityCubeVerts(origin), parityCubeVerts(second)...),
			tris:  append(parityCubeTris(0), parityCubeTris(8)...),
			subsets: []parityNamedSubset{
				{`first component`, parityIdentitySubset(12)},
				{`second component`, secondCube},
				{`empty`, nil},
			},
			queries: append(parityCubeQueries(origin), parityCubeQueries(second)...),
		},
		{
			name:    `every ray hits a facet vertex`,
			verts:   paritySoupVerts(),
			tris:    paritySoupTris(),
			subsets: []parityNamedSubset{{`stored order`, parityIdentitySubset(len(paritySoupTris()))}},
			queries: []parityCase{{`origin`, xptOf(r3.NewVec(0, 0, 0))}},
		},
	}
}

// requirePreparedMatchesReference requires one query against one prepared
// object to reproduce the frozen reference on the same buffers.
func requirePreparedMatchesReference(ctx context.Context, t *testing.T, arm string, prepared *parityMesh, p xpt, subset []int) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, prepared.verts, prepared.tris, subset)
	gotIn, gotBoundary, gotErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, arm,
		parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr},
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})
}

// parityAllocatedAxes reports which of the three axis projection slices the
// cache has allocated at all.
func parityAllocatedAxes(pm *parityMesh) [3]bool {
	var out [3]bool
	for axis := range out {
		out[axis] = pm.projections[axis] != nil
	}
	return out
}

// parityFilledVertices returns, in index order, the vertices whose projection
// on axis is materialized. A nil u is the unfilled marker; an exact zero
// coordinate is a non-nil rational, so a filled entry never reads as unfilled.
func parityFilledVertices(pm *parityMesh, axis int) []int {
	var out []int
	for vi, e := range pm.projections[axis] {
		if e.u != nil {
			out = append(out, vi)
		}
	}
	return out
}

// parityCacheEntry is one materialized projection, kept for a later comparison:
// its identity in the cache, the two rational pointers the entry holds, and
// their exact printed values.
type parityCacheEntry struct {
	axis, vi int
	u, v     *big.Rat
	us, vs   string
}

// parityCacheSnapshot records every materialized entry of the cache.
func parityCacheSnapshot(pm *parityMesh) []parityCacheEntry {
	var out []parityCacheEntry
	for axis := range pm.projections {
		for _, vi := range parityFilledVertices(pm, axis) {
			e := pm.projections[axis][vi]
			out = append(out, parityCacheEntry{
				axis: axis, vi: vi,
				u: e.u, v: e.v,
				us: e.u.RatString(), vs: e.v.RatString(),
			})
		}
	}
	return out
}

// requireSnapshotPreserved requires every entry recorded earlier to still be
// present, still be the SAME *big.Rat, and still hold the same value: a later
// query must neither rebuild an entry nor use one as an arithmetic destination.
func requireSnapshotPreserved(t *testing.T, want []parityCacheEntry, pm *parityMesh) {
	t.Helper()
	require.NotEmpty(t, want, `the snapshot must cover at least one materialized entry`)
	for _, e := range want {
		require.NotNil(t, pm.projections[e.axis], `axis %d must still be allocated`, e.axis)
		got := pm.projections[e.axis][e.vi]
		require.Same(t, e.u, got.u, `axis %d vertex %d must keep its u rational`, e.axis, e.vi)
		require.Same(t, e.v, got.v, `axis %d vertex %d must keep its v rational`, e.axis, e.vi)
		require.Equal(t, e.us, got.u.RatString(), `axis %d vertex %d must keep its u value`, e.axis, e.vi)
		require.Equal(t, e.vs, got.v.RatString(), `axis %d vertex %d must keep its v value`, e.axis, e.vi)
	}
}

// parityCancelAfterErr wraps a parent context so that Err reports cancellation
// only from the check after clean onward. The parity scan then stops at a
// deterministic facet INSIDE the subset rather than before the first one. The
// wrapper counts, so each arm needs its own: sharing one would cancel the two
// arms at different facets.
type parityCancelAfterErr struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	checks          int
	clean           int
}

func (c *parityCancelAfterErr) Err() error {
	c.checks++
	if c.checks > c.clean {
		return context.Canceled
	}
	return c.Context.Err()
}

func TestPreparedParityProjectionReuse(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	origin := r3.NewVec(0, 0, 0)

	t.Run(`every fixture through one prepared object`, func(t *testing.T) {
		for _, fixture := range parityReuseFixtures() {
			t.Run(fixture.name, func(t *testing.T) {
				// ONE cache for the whole group: every later query in it reads
				// projections an earlier query materialized.
				prepared := newParityMesh(fixture.verts, fixture.tris)
				for _, subset := range fixture.subsets {
					for _, c := range fixture.queries {
						requirePreparedMatchesReference(ctx, t,
							subset.name+`/`+c.name, prepared, c.p, subset.facets)
					}
				}
			})
		}
	})

	t.Run(`interleaved queries repeat identically`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		first := parityIdentitySubset(12)
		rest := parityStrideSubset(12, 7)
		for i := range rest {
			rest[i] += 12
		}
		prepared := newParityMesh(verts, tris)
		p := xptOf(r3.NewVec(0.25, 0.375, 0.5625))

		firstIn, firstBoundary, firstErr := meshParityPreparedContext(ctx, p, prepared, first)
		require.NoError(t, firstErr)
		require.True(t, firstIn, `the query is interior to the first cube`)
		require.False(t, firstBoundary)

		// Different queries against different subsets, in between.
		for _, subset := range []parityNamedSubset{{`first`, first}, {`second`, rest}, {`empty`, nil}} {
			for _, c := range append(parityCubeQueries(origin), parityCubeQueries(second)...) {
				requirePreparedMatchesReference(ctx, t,
					subset.name+`/`+c.name, prepared, c.p, subset.facets)
			}
		}

		againIn, againBoundary, againErr := meshParityPreparedContext(ctx, p, prepared, first)
		require.NoError(t, againErr, `the repeated query must not start failing`)
		require.Equal(t, firstIn, againIn, `the repeated query must return its first answer`)
		require.Equal(t, firstBoundary, againBoundary, `the repeated query must keep its boundary verdict`)
	})

	t.Run(`each coordinate projection is allocated only when its ray is swept`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		for _, tc := range []struct {
			name  string
			query r3.Vec
			axes  [3]bool
		}{
			// (y, z) projects clear of every cube-face diagonal, so the first
			// ray decides and no later axis is ever swept.
			{`first ray decides`, r3.NewVec(0.25, 0.375, 0.5625), [3]bool{true, false, false}},
			// y + z = 1 puts the projection on the x-face diagonal, so the x
			// rays are ambiguous and the y rays decide.
			{`falls back to the second axis`, r3.NewVec(0.25, 0.375, 0.625), [3]bool{true, true, false}},
			// y + z = 1 and z = x: both the x and the y rays are ambiguous, and
			// the z rays decide.
			{`falls back to the third axis`, r3.NewVec(0.25, 0.75, 0.25), [3]bool{true, true, true}},
		} {
			t.Run(tc.name, func(t *testing.T) {
				prepared := newParityMesh(verts, tris)
				p := xptOf(tc.query)
				requirePreparedMatchesReference(ctx, t, tc.name, prepared, p, subset)
				require.Equal(t, tc.axes, parityAllocatedAxes(prepared),
					`only the axes actually swept may hold a projection slice`)

				inside, onBoundary, err := meshParityPreparedContext(ctx, p, prepared, subset)
				require.NoError(t, err)
				require.True(t, inside, `the query is interior to the unit cube`)
				require.False(t, onBoundary)
			})
		}
	})

	t.Run(`cached rationals survive later queries`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		prepared := newParityMesh(verts, tris)

		// Sweep all three axes first, so the snapshot covers every slot.
		requirePreparedMatchesReference(ctx, t, `seed`, prepared,
			xptOf(r3.NewVec(0.25, 0.75, 0.25)), subset)
		snapshot := parityCacheSnapshot(prepared)
		require.Equal(t, [3]bool{true, true, true}, parityAllocatedAxes(prepared))

		for _, c := range parityCubeQueries(origin) {
			requirePreparedMatchesReference(ctx, t, c.name, prepared, c.p, subset)
		}
		requireSnapshotPreserved(t, snapshot, prepared)
	})

	t.Run(`two prepared meshes keep their own coordinates`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		firstMesh := newParityMesh(parityCubeVerts(origin), parityCubeTris(0))
		secondMesh := newParityMesh(parityCubeVerts(second), parityCubeTris(0))
		subset := parityIdentitySubset(12)

		for _, c := range parityCubeQueries(origin) {
			requirePreparedMatchesReference(ctx, t, `first/`+c.name, firstMesh, c.p, subset)
			requirePreparedMatchesReference(ctx, t, `second/`+c.name, secondMesh, c.p, subset)
		}
		for _, c := range parityCubeQueries(second) {
			requirePreparedMatchesReference(ctx, t, `first/`+c.name, firstMesh, c.p, subset)
			requirePreparedMatchesReference(ctx, t, `second/`+c.name, secondMesh, c.p, subset)
		}

		// Vertex 6 is (1, 1, 1) in the first mesh and (4, 1, 1) in the second.
		// Axis 0 projects onto (y, z), which the offset leaves alone, so the
		// separation shows on an axis the offset does move.
		firstIn, _, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(3.25, 0.375, 0.5625)), firstMesh, subset)
		require.NoError(t, err)
		require.False(t, firstIn, `a point in the second cube is outside the first`)
		secondIn, _, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(3.25, 0.375, 0.5625)), secondMesh, subset)
		require.NoError(t, err)
		require.True(t, secondIn, `and interior to the second`)
	})

	t.Run(`an empty subset allocates nothing`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		prepared := newParityMesh(verts, tris)
		inside, onBoundary, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), prepared, nil)
		require.NoError(t, err)
		require.False(t, inside)
		require.False(t, onBoundary)
		require.Equal(t, [3]bool{false, false, false}, parityAllocatedAxes(prepared),
			`no ray was swept, so no axis may hold a projection slice`)
	})

	t.Run(`a canceled context returns before the cache fills`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		prepared := newParityMesh(verts, tris)

		_, _, err := meshParityPreparedContext(canceled,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), prepared, subset)
		require.ErrorIs(t, err, context.Canceled, `a nonempty subset observes the cancellation`)
		require.Equal(t, [3]bool{false, false, false}, parityAllocatedAxes(prepared),
			`the context check runs before the first projection`)
	})

	t.Run(`a component query leaves the other component unprojected`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		prepared := newParityMesh(verts, tris)

		requirePreparedMatchesReference(ctx, t, `first component`, prepared,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), parityIdentitySubset(12))

		for axis := range prepared.projections {
			if prepared.projections[axis] == nil {
				continue
			}
			require.Equal(t, []int{0, 1, 2, 3, 4, 5, 6, 7}, parityFilledVertices(prepared, axis),
				`axis %d must project the queried component's vertices and no others`, axis)
		}
	})

	t.Run(`cancellation inside the scan leaves the cache usable`, func(t *testing.T) {
		verts, tris, subset, p := parityBenchMesh(parityBenchCubes)
		require.Greater(t, len(subset), 256,
			`the subset must outrun one context-check block for the cancellation to land inside the scan`)

		// One fresh counting context per arm: it is stateful, so sharing it
		// would cancel the two arms at different facets.
		refCtx := &parityCancelAfterErr{Context: ctx, clean: 1}
		//nolint:contextcheck // the counting wrapper IS the fixture: it derives from ctx and only delays the cancellation.
		_, _, wantErr := referenceMeshParityContext(refCtx, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, context.Canceled, `the reference stops mid-scan`)

		prepared := newParityMesh(verts, tris)
		gotCtx := &parityCancelAfterErr{Context: ctx, clean: 1}
		//nolint:contextcheck // same wrapper, its own counter, so both arms stop at the same facet.
		_, _, gotErr := meshParityPreparedContext(gotCtx, p, prepared, subset)
		require.ErrorIs(t, gotErr, context.Canceled, `the prepared kernel stops mid-scan too`)
		require.Equal(t, wantErr.Error(), gotErr.Error(), `and reports the same error`)
		require.NotEmpty(t, parityFilledVertices(prepared, 0),
			`the facets before the cancellation still filled their projections`)

		// The half-filled cache must still answer correctly.
		requirePreparedMatchesReference(ctx, t, `after cancellation`, prepared, p, subset)
	})
}

// parityBenchMesh builds one vertex/triangle buffer holding that many disjoint
// closed unit cubes spaced along x, plus a query interior to the first.
func parityBenchMesh(cubes int) ([]r3.Vec, [][3]int, []int, xpt) {
	verts := make([]r3.Vec, 0, cubes*8)
	tris := make([][3]int, 0, cubes*12)
	for k := range cubes {
		verts = append(verts, parityCubeVerts(r3.NewVec(float64(3*k), 0, 0))...)
		tris = append(tris, parityCubeTris(8*k)...)
	}
	return verts, tris, parityIdentitySubset(len(tris)), xptOf(r3.NewVec(0.25, 0.375, 0.5625))
}

// requireBenchInside checks one benchmark iteration's answer, so a faster wrong
// answer cannot win an arm.
func requireBenchInside(b *testing.B, inside, onBoundary bool, err error) {
	b.Helper()
	if err != nil {
		b.Fatal(err)
	}
	if !inside || onBoundary {
		b.Fatalf(`want inside, got inside=%t onBoundary=%t`, inside, onBoundary)
	}
}

// BenchmarkParityQueryProjection measures the frozen reference against every
// production entry point on one prebuilt mesh and query, so the projections each
// step removes show up as an allocation difference.
//
// The prepared arms separate the two costs the cache trades between: `prepared
// warm` builds the cache once outside the timing, which is what a real operand
// does across the many queries of one operation, while `prepared cold` rebuilds
// it every iteration so setup cost cannot hide behind reuse.
func BenchmarkParityQueryProjection(b *testing.B) {
	verts, tris, subset, p := parityBenchMesh(parityBenchCubes)
	require.GreaterOrEqual(b, len(tris), 1024, `the benchmark mesh must exercise at least 1024 triangles`)
	ctx := b.Context()

	for _, arm := range []struct {
		name string
		fn   func(context.Context, xpt, []r3.Vec, [][3]int, []int) (bool, bool, error)
	}{
		{`reference`, referenceMeshParityContext},
		{`production`, meshParityContext},
	} {
		b.Run(arm.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				inside, onBoundary, err := arm.fn(ctx, p, verts, tris, subset)
				requireBenchInside(b, inside, onBoundary, err)
			}
		})
	}

	b.Run(`prepared warm`, func(b *testing.B) {
		prepared := newParityMesh(verts, tris)
		b.ReportAllocs()
		for b.Loop() {
			inside, onBoundary, err := meshParityPreparedContext(ctx, p, prepared, subset)
			requireBenchInside(b, inside, onBoundary, err)
		}
	})

	b.Run(`prepared cold`, func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			inside, onBoundary, err := meshParityPreparedContext(ctx, p, newParityMesh(verts, tris), subset)
			requireBenchInside(b, inside, onBoundary, err)
		}
	})
}

// parityScatterVerts is a closed tetrahedron skewed off every axis, so none of
// its four facets projects onto a segment on any of the three planes the rays
// leave. That matters: the axis-aligned fixtures the file already had make the
// very first facet of a scan ambiguous, which ends the scan before a filter can
// skip anything, so they say nothing about how much work the filter removes.
// Every coordinate is a binary fraction and so exactly representable.
func parityScatterVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0.25, 0.125),
		r3.NewVec(0.25, 1, 0.375),
		r3.NewVec(0.375, 0.5, 1),
	}
}

// parityScatterInterior is a point strictly inside the tetrahedron at the
// scatter grid's origin.
func parityScatterInterior() r3.Vec { return r3.NewVec(0.40625, 0.4375, 0.375) }

// parityScatterMesh builds a side×side grid of those tetrahedra spread across
// the plane axis 0 projects onto, so a query near one of them leaves most
// facets provably outside their own projected box — the case the box filter is
// for.
func parityScatterMesh(side int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, side*side*4)
	tris := make([][3]int, 0, side*side*4)
	for i := range side {
		for j := range side {
			base := 4 * (i*side + j)
			for _, v := range parityScatterVerts() {
				verts = append(verts, r3.NewVec(v.X, v.Y+float64(3*i), v.Z+float64(3*j)))
			}
			for _, t := range parityTetraTris() {
				tris = append(tris, [3]int{t[0] + base, t[1] + base, t[2] + base})
			}
		}
	}
	return verts, tris
}

// parityProjectedCorners returns one facet's three projected corners as the
// exact rational pairs the parity kernel classifies against, built the frozen
// reference's way rather than through the cache under test.
func parityProjectedCorners(verts []r3.Vec, tri [3]int, u, v int) [3]xp2 {
	var out [3]xp2
	for k, vi := range tri {
		out[k] = newXP2(mustRatOf(coordOf(verts[vi], u)), mustRatOf(coordOf(verts[vi], v)))
	}
	return out
}

// parityFloatAreaAbstains recomputes buildFacetBox's float area filter, so a
// case can assert that the float leg could NOT decide and the exact leg is what
// answered.
func parityFloatAreaAbstains(verts []r3.Vec, tri [3]int, u, v int) bool {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	au, av := coordOf(a, u), coordOf(a, v)
	bu, bv := coordOf(b, u), coordOf(b, v)
	cu, cv := coordOf(c, u), coordOf(c, v)
	left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
	bound := parityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + parityAreaFloor
	det := left - right
	return !(det > bound || det < -bound)
}

// parityFixture names one vertex/triangle buffer.
type parityFixture struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// parityBoxFixtures are the meshes the box filter is asserted over: the
// axis-aligned ones the file already used, plus the scattered tetrahedra that
// actually make the filter fire.
func parityBoxFixtures() []parityFixture {
	scatterVerts, scatterTris := parityScatterMesh(4)
	farVerts, farTris := parityScatterMesh(3)
	for i, v := range farVerts {
		farVerts[i] = r3.NewVec(v.X+parityFar, v.Y+parityFar, v.Z+parityFar)
	}
	return []parityFixture{
		{`unit cube`, parityCubeVerts(r3.NewVec(0, 0, 0)), parityCubeTris(0)},
		{`unit tetrahedron`, parityTetraVerts(), parityTetraTris()},
		{`triangle soup sharing the origin`, paritySoupVerts(), paritySoupTris()},
		{`scattered tetrahedra`, scatterVerts, scatterTris},
		{`scattered tetrahedra far from the origin`, farVerts, farTris},
	}
}

// TestParityFacetBoxStatesTheProjection pins what a box says about its facet:
// the four bounds are the exact extremes of the projected triangle, and
// nondegenerate is the EXACT area sign, not the float filter's guess.
func TestParityFacetBoxStatesTheProjection(t *testing.T) {
	t.Parallel()

	t.Run(`bounds and area match the exact projection`, func(t *testing.T) {
		for _, fx := range parityBoxFixtures() {
			t.Run(fx.name, func(t *testing.T) {
				pm := newParityMesh(fx.verts, fx.tris)
				for _, ray := range axisRays {
					for ti := range fx.tris {
						box := pm.buildFacetBox(ray.axis, ray.u, ray.v, ti)
						q := parityProjectedCorners(fx.verts, fx.tris[ti], ray.u, ray.v)
						require.True(t, box.built, `a built box must say so`)
						require.Equal(t, math.Min(q[0].fu, math.Min(q[1].fu, q[2].fu)), box.minU,
							`axis %d facet %d: minU is the smallest projected u`, ray.axis, ti)
						require.Equal(t, math.Max(q[0].fu, math.Max(q[1].fu, q[2].fu)), box.maxU,
							`axis %d facet %d: maxU is the largest projected u`, ray.axis, ti)
						require.Equal(t, math.Min(q[0].fv, math.Min(q[1].fv, q[2].fv)), box.minV,
							`axis %d facet %d: minV is the smallest projected v`, ray.axis, ti)
						require.Equal(t, math.Max(q[0].fv, math.Max(q[1].fv, q[2].fv)), box.maxV,
							`axis %d facet %d: maxV is the largest projected v`, ray.axis, ti)
						require.Equal(t, cross2xSign(q[0], q[1], q[2]) != 0, box.nondegenerate,
							`axis %d facet %d: nondegenerate must be the exact area sign`, ray.axis, ti)
					}
				}
			})
		}
	})

	t.Run(`a projection with no area is never usable`, func(t *testing.T) {
		for _, tc := range []struct {
			name  string
			verts []r3.Vec
		}{
			{`three collinear corners`, []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 2), r3.NewVec(0, 3, 6)}},
			{`a repeated corner`, []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 4)}},
			{`a facet flat against the swept axis`, []r3.Vec{
				r3.NewVec(0, 0, 5), r3.NewVec(1, 1, 5), r3.NewVec(2, 4, 5)}},
		} {
			t.Run(tc.name, func(t *testing.T) {
				pm := newParityMesh(tc.verts, [][3]int{{0, 1, 2}})
				box := pm.buildFacetBox(0, 1, 2, 0)
				require.False(t, box.nondegenerate,
					`a projection whose exact area is zero may never license a skip`)
				require.False(t, box.rejects(-1e9, -1e9),
					`an unusable box rejects nothing, however far away the query is`)
			})
		}
	})

	t.Run(`a non-finite coordinate leaves the box unusable`, func(t *testing.T) {
		for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
			verts := []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, bad)}
			pm := newParityMesh(verts, [][3]int{{0, 1, 2}})
			box := pm.buildFacetBox(0, 1, 2, 0)
			require.False(t, box.nondegenerate,
				`a box built over %v bounds nothing`, bad)
			require.False(t, box.rejects(-1e9, -1e9))
		}
	})

	t.Run(`the exact leg decides an area the float filter cannot`, func(t *testing.T) {
		// The projected corners are (0, 0), (1, 1) and (2⁴⁰, 2⁴⁰+1): a true
		// area of 1 against a permanent above 2⁴¹, which is three decades
		// under the float filter's own threshold.
		const big = 1 << 40
		verts := []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 1), r3.NewVec(0, big, big+1)}
		tri := [3]int{0, 1, 2}
		require.True(t, parityFloatAreaAbstains(verts, tri, 1, 2),
			`the case is only meaningful while the float filter abstains`)
		q := parityProjectedCorners(verts, tri, 1, 2)
		require.NotEqual(t, 0, cross2xSign(q[0], q[1], q[2]),
			`and while the exact area is genuinely nonzero`)

		pm := newParityMesh(verts, [][3]int{tri})
		box := pm.buildFacetBox(0, 1, 2, 0)
		require.True(t, box.nondegenerate,
			`the exact leg must recover the area the float filter could not prove`)
		require.True(t, box.rejects(-1, 0), `a query below minU is rejected`)
		require.False(t, box.rejects(0, 0), `a query on the box boundary is not`)
	})
}

// parityBoxQueries returns the query points the box filter is asserted over on
// one fixture: the hand-built cube and tetrahedron cases, plus points placed
// exactly at, one ulp inside and one ulp outside a real facet's own box bounds
// on every axis — the boundary the filter's strict comparison decides.
func parityBoxQueries(fx parityFixture) []parityCase {
	out := append(parityCubeQueries(r3.NewVec(0, 0, 0)), parityTetraQueries()...)
	pm := newParityMesh(fx.verts, fx.tris)
	for _, ray := range axisRays {
		box := pm.buildFacetBox(ray.axis, ray.u, ray.v, 0)
		for _, edge := range []float64{box.minU, box.maxU} {
			for _, at := range []float64{edge,
				math.Nextafter(edge, math.Inf(-1)), math.Nextafter(edge, math.Inf(1))} {
				c := make([]float64, 3)
				c[ray.u] = at
				c[ray.v] = box.minV
				out = append(out, parityCase{
					fmt.Sprintf(`axis %d facet 0 u-bound %v`, ray.axis, at),
					xptOf(r3.NewVec(c[0], c[1], c[2])),
				})
				// The same place reached as an exact rational a hair off the
				// bound, which is what a midpoint or centroid witness is.
				out = append(out, parityCase{
					fmt.Sprintf(`axis %d facet 0 u-bound %v minus 1/3 ulp`, ray.axis, at),
					xptFromRat(
						parityBoxOffsetRat(c[0], ray.u == 0, at),
						parityBoxOffsetRat(c[1], ray.u == 1, at),
						parityBoxOffsetRat(c[2], ray.u == 2, at)),
				})
			}
		}
	}
	return out
}

// parityBoxOffsetRat is coordinate c exactly, except on the swept-plane axis
// the caller marks, where it is the bound minus a third of the gap to the next
// float below it — a value no float64 names, sitting inside the last ulp.
func parityBoxOffsetRat(c float64, offset bool, at float64) *big.Rat {
	r := mustRatOf(c)
	if !offset {
		return r
	}
	gap := new(big.Rat).Sub(mustRatOf(at), mustRatOf(math.Nextafter(at, math.Inf(-1))))
	return new(big.Rat).Sub(r, new(big.Rat).Quo(gap, big.NewRat(3, 1)))
}

// TestParityBoxFilterOnlySkipsStrictlyOutsideFacets asserts the filter's whole
// contract at facet granularity: every facet it skips is one the unfiltered
// classification would have passed over as STRICTLY OUTSIDE. A skip that landed
// on an ambiguous or a counted facet would change the answer, and this test
// catches it on the offending facet rather than on a mesh-wide verdict that
// happens to survive it.
func TestParityBoxFilterOnlySkipsStrictlyOutsideFacets(t *testing.T) {
	t.Parallel()
	skips := 0
	for _, fx := range parityBoxFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			pm := newParityMesh(fx.verts, fx.tris)
			for _, c := range parityBoxQueries(fx) {
				for _, ray := range axisRays {
					pa := newXP2(ratCoordOf(c.p, ray.u), ratCoordOf(c.p, ray.v))
					if !pa.floatFinite {
						continue
					}
					for ti := range fx.tris {
						box := pm.buildFacetBox(ray.axis, ray.u, ray.v, ti)
						if !box.rejects(pa.fu, pa.fv) {
							continue
						}
						skips++
						q := parityProjectedCorners(fx.verts, fx.tris[ti], ray.u, ray.v)
						s1 := cross2xSign(q[0], q[1], pa)
						s2 := cross2xSign(q[1], q[2], pa)
						s3 := cross2xSign(q[2], q[0], pa)
						neg := s1 < 0 || s2 < 0 || s3 < 0
						pos := s1 > 0 || s2 > 0 || s3 > 0
						require.True(t, neg && pos,
							`%s: axis %d facet %d was skipped, but its exact signs (%d, %d, %d) are not the strictly-outside pattern`,
							c.name, ray.axis, ti, s1, s2, s3)
					}
				}
			}
		})
	}
	require.Positive(t, skips,
		`the fixtures must actually exercise the filter, or the property above is vacuous`)
}

// TestParityBoxFilterMatchesTheUnfilteredKernel is the end-to-end differential:
// the same prepared mesh answered with the filter and with it switched off, and
// the frozen pre-filter reference beside both, over hand-built and fixed-seed
// random queries and over subsets of every shape the callers use.
func TestParityBoxFilterMatchesTheUnfilteredKernel(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	verts, tris := parityScatterMesh(4)
	n := len(tris)

	rng := rand.New(rand.NewPCG(0x5ee1, 0xb0c5))
	queries := append(parityBoxQueries(parityFixture{`scattered`, verts, tris}),
		parityCubeQueries(r3.NewVec(0, 3, 3))...)
	for range 64 {
		queries = append(queries, parityCase{
			fmt.Sprintf(`random %d`, len(queries)),
			xptFromRat(
				big.NewRat(rng.Int64N(4000)-500, 100),
				big.NewRat(rng.Int64N(4000)-500, 100),
				big.NewRat(rng.Int64N(4000)-500, 100)),
		})
	}
	// A handful of queries placed exactly on a mesh vertex, which is where the
	// unfiltered kernel reports ambiguity and a boundary.
	for range 8 {
		queries = append(queries, parityCase{
			fmt.Sprintf(`vertex %d`, len(queries)),
			xptOf(verts[rng.IntN(len(verts))]),
		})
	}

	subsets := []parityNamedSubset{
		{`identity`, parityIdentitySubset(n)},
		{`empty`, nil},
		{`singleton`, []int{n / 2}},
		{`stride`, parityStrideSubset(n, 7)},
	}
	for k := range 4 {
		shuffled := parityIdentitySubset(n)
		rng.Shuffle(len(shuffled), func(i, j int) {
			shuffled[i], shuffled[j] = shuffled[j], shuffled[i]
		})
		subsets = append(subsets, parityNamedSubset{
			fmt.Sprintf(`shuffled prefix %d`, k), shuffled[:1+rng.IntN(n)]})
	}

	// ONE cache per arm across every query, so a box built for an earlier query
	// is what a later one reads.
	filtered := newParityMesh(verts, tris)
	plain := newParityMesh(verts, tris)
	plain.unfiltered = true

	for _, subset := range subsets {
		for _, c := range queries {
			arm := subset.name + `/` + c.name
			wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, c.p, verts, tris, subset.facets)
			want := parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr}

			plainIn, plainBoundary, plainErr := meshParityPreparedContext(ctx, c.p, plain, subset.facets)
			requireParityOutcome(t, `unfiltered/`+arm, want,
				parityOutcome{inside: plainIn, onBoundary: plainBoundary, err: plainErr})

			gotIn, gotBoundary, gotErr := meshParityPreparedContext(ctx, c.p, filtered, subset.facets)
			requireParityOutcome(t, `filtered/`+arm, want,
				parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})
		}
	}
}

// TestParityBoxFilterLeavesAnEmptySubsetAlone pins that arming the filter costs
// a query nothing until a facet is actually visited: an empty subset builds no
// box slice, and a canceled context returns before the first one.
func TestParityBoxFilterLeavesAnEmptySubsetAlone(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	verts, tris := parityScatterMesh(2)
	p := xptOf(parityScatterInterior())

	t.Run(`an empty subset builds no boxes`, func(t *testing.T) {
		pm := newParityMesh(verts, tris)
		_, _, err := meshParityPreparedContext(ctx, p, pm, nil)
		require.NoError(t, err)
		require.Equal(t, [3][]parityFacetBox{}, pm.boxes,
			`no facet was visited, so no axis may hold a box slice`)
	})

	t.Run(`a canceled context returns before the first box`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		pm := newParityMesh(verts, tris)
		_, _, err := meshParityPreparedContext(canceled, p, pm, parityIdentitySubset(len(tris)))
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, [3][]parityFacetBox{}, pm.boxes,
			`the context check runs before the first box`)
	})
}

// BenchmarkParityBoxFilter measures the filter against the same kernel with it
// switched off, on a mesh spread across the projection plane — the shape a real
// operand has, and the one the axis-stacked BenchmarkParityQueryProjection mesh
// deliberately does not.
func BenchmarkParityBoxFilter(b *testing.B) {
	verts, tris := parityScatterMesh(12)
	subset := parityIdentitySubset(len(tris))
	require.GreaterOrEqual(b, len(tris), 512, `the benchmark mesh must exercise at least 512 triangles`)
	ctx := b.Context()
	// Interior to the tetrahedron at the grid's origin, so the answer the
	// benchmark checks is `inside` and both arms must agree on it.
	p := xptOf(parityScatterInterior())

	for _, arm := range []struct {
		name       string
		unfiltered bool
	}{{`filtered`, false}, {`unfiltered`, true}} {
		b.Run(arm.name, func(b *testing.B) {
			prepared := newParityMesh(verts, tris)
			prepared.unfiltered = arm.unfiltered
			b.ReportAllocs()
			for b.Loop() {
				inside, onBoundary, err := meshParityPreparedContext(ctx, p, prepared, subset)
				requireBenchInside(b, inside, onBoundary, err)
			}
		})
	}
}
