package decad

import (
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
	rng := rand.New(rand.NewPCG(7, 11))
	for range 4000 {
		a := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
		b := r3.NewVec(rng.Float64()*200-100, rng.Float64()*200-100, rng.Float64()*200-100)
		xa, xb := xptOf(a), xptOf(b)
		// An exact interior point of the exact segment, at a rational parameter
		// no float64 can represent, so its rounding is a genuine approximation.
		tt := big.NewRat(int64(rng.IntN(9999)+1), 10000)
		xp := xlerp(xa, xb, tt)
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
			tt := big.NewRat(int64(rng.IntN(9999)+1), 10000)
			xp := xlerp(xa, xb, tt)
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
