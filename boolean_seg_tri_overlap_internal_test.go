package decad

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceSegTriOverlap2 is the PRE-OPTIMIZATION reference copy of
// segTriOverlap2, captured verbatim before the clip parameter stopped being a
// normalised big.Rat: same branch order, same sign handling, same returns, and
// the same big.Rat.Quo per crossing edge that the unnormalised clipFrac
// removes.
//
// It is frozen on purpose. A later production edit must NEVER be mirrored into
// this function — the whole value of the comparison is that the reference
// still states the behaviour production had before the change. If production
// and this function disagree, production changed a verdict and the change is
// wrong, not this copy.
func referenceSegTriOverlap2(a, b, ta, tb, tc xp2) bool {
	ccw := cross2xSign(ta, tb, tc)
	if ccw == 0 {
		return false
	}
	edges := [3][2]xp2{{ta, tb}, {tb, tc}, {tc, ta}}
	lo, hi := new(big.Rat), new(big.Rat).SetInt64(1)
	for _, e := range edges {
		sa := cross2xSign(e[0], e[1], a)
		sb := cross2xSign(e[0], e[1], b)
		if ccw < 0 {
			sa = -sa
			sb = -sb
		}
		switch {
		case sa >= 0 && sb >= 0:
			continue
		case sa < 0 && sb < 0:
			return false
		default:
			fa := cross2x(e[0], e[1], a)
			fb := cross2x(e[0], e[1], b)
			if ccw < 0 {
				fa.Neg(fa)
				fb.Neg(fb)
			}
			diff := new(big.Rat).Sub(fb, fa)
			t := new(big.Rat).Quo(new(big.Rat).Neg(fa), diff)
			if sa < 0 {
				if t.Cmp(lo) > 0 {
					lo = t
				}
			} else {
				if t.Cmp(hi) < 0 {
					hi = t
				}
			}
		}
	}
	return lo.Cmp(hi) < 0
}

// pt2 is one differential-test point in plain float coordinates. Each case is
// lifted into every xp2 flavour segTriOverlap2 can be handed, so a case tests
// both the homogeneous route through edgeCross2Fracs and the rational
// fallback through cross2x.
type pt2 struct{ x, y float64 }

// xp2Flavour lifts a float coordinate pair into one xp2 representation.
type xp2Flavour struct {
	name string
	lift func(pt2) xp2
}

// xp2Flavours are the three shapes an xp2 reaches segTriOverlap2 in: the
// rational-only form polygon construction builds, the homogeneous form the
// projection caches build from a lifted float vertex (weight a stripped power
// of two), and a homogeneous form carrying a deliberately awkward weight, so
// the shared-factor cancellation in edgeCross2Fracs is exercised against
// weights that differ from point to point.
var xp2Flavours = []xp2Flavour{
	{
		name: "rational",
		lift: func(p pt2) xp2 { return newXP2(mustRatOf(p.x), mustRatOf(p.y)) },
	},
	{
		name: "homogeneous",
		lift: func(p pt2) xp2 { return newXP2FromXpt(xptOf(r3.Vec{X: p.x, Y: p.y}), 0, 1) },
	},
	{
		name: "homogeneous-weighted",
		lift: func(p pt2) xp2 { return newXP2FromXpt(weightedXpt(p, 3), 0, 1) },
	},
}

// weightedXpt respells a point with a chosen positive homogeneous weight, so
// the two segment endpoints and the three triangle corners no longer share one
// denominator. The coordinate is unchanged: numerator and weight are scaled
// together.
func weightedXpt(p pt2, w int64) xpt {
	x, y := mustRatOf(p.x), mustRatOf(p.y)
	den := new(big.Int).Mul(x.Denom(), y.Denom())
	den.Mul(den, big.NewInt(w))
	num := func(r *big.Rat) *big.Int {
		out := new(big.Int).Mul(r.Num(), den)
		return out.Quo(out, r.Denom())
	}
	return xpt{x: num(x), y: num(y), z: big.NewInt(0), w: den}
}

// segTriCase is one segment-versus-triangle configuration.
type segTriCase struct {
	name           string
	a, b           pt2
	ta, tb, tc     pt2
	wantIfDecided  bool
	assertExpected bool
}

// segTriLiftings pairs a flavour for the segment with a flavour for the
// triangle. Production hands segTriOverlap2 six points from one source, so the
// matching pairs are the reachable ones; the mismatched pairs are here because
// edgeCross2Fracs may only take its homogeneous route when all four points it
// reads carry homogeneous coordinates, and nothing else would exercise that
// guard.
func segTriLiftings() []struct {
	name     string
	seg, tri xp2Flavour
} {
	var out []struct {
		name     string
		seg, tri xp2Flavour
	}
	for _, seg := range xp2Flavours {
		for _, tri := range xp2Flavours {
			out = append(out, struct {
				name     string
				seg, tri xp2Flavour
			}{name: seg.name + "-seg/" + tri.name + "-tri", seg: seg, tri: tri})
		}
	}
	return out
}

// requireSegTriParity runs one case through every lifting and asserts the
// candidate matches the frozen reference. Cases that also name the geometric
// verdict assert that too, so a change that broke BOTH implementations the
// same way would still be caught.
func requireSegTriParity(t *testing.T, c segTriCase) {
	t.Helper()
	for _, lifting := range segTriLiftings() {
		a, b := lifting.seg.lift(c.a), lifting.seg.lift(c.b)
		ta, tb, tc := lifting.tri.lift(c.ta), lifting.tri.lift(c.tb), lifting.tri.lift(c.tc)
		want := referenceSegTriOverlap2(a, b, ta, tb, tc)
		got := segTriOverlap2(a, b, ta, tb, tc)
		require.Equal(t, want, got, "%s/%s: verdict must match the pre-optimization reference", c.name, lifting.name)
		if c.assertExpected {
			require.Equal(t, c.wantIfDecided, got, "%s/%s: geometric verdict", c.name, lifting.name)
		}
	}
}

// segTriOverlapCases covers the shapes the clip loop has to get right:
// interior crossings, contact along an edge, contact at a single vertex,
// collinear and degenerate inputs, and both triangle windings.
func segTriOverlapCases() []segTriCase {
	tri := [3]pt2{{0, 0}, {4, 0}, {0, 4}}
	rev := [3]pt2{{0, 0}, {0, 4}, {4, 0}}
	cases := []segTriCase{
		{
			name: "segment crosses the interior",
			a:    pt2{-1, 1}, b: pt2{3, 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "segment lies wholly inside",
			a:    pt2{0.5, 0.5}, b: pt2{1.5, 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "segment misses on the outside",
			a:    pt2{5, 5}, b: pt2{9, 9},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "segment runs along an edge",
			a:    pt2{1, 0}, b: pt2{3, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "segment overhangs an edge at both ends",
			a:    pt2{-2, 0}, b: pt2{6, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "segment touches one vertex only",
			a:    pt2{-2, 2}, b: pt2{0, 4},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "segment ends on a vertex pointing outward",
			a:    pt2{4, 0}, b: pt2{8, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "segment ends on an edge pointing outward",
			a:    pt2{2, 0}, b: pt2{2, -4},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "segment enters through a vertex",
			a:    pt2{-1, -1}, b: pt2{1, 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "segment collinear with an edge but disjoint from it",
			a:    pt2{5, 0}, b: pt2{9, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			// A zero-length segment strictly inside crosses no edge, so the
			// parameter interval stays [0, 1] and both implementations report
			// an overlap even though the segment has no length. That is the
			// behaviour at this commit; the case is here to pin it, not to
			// endorse it.
			name: "zero-length segment inside",
			a:    pt2{1, 1}, b: pt2{1, 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "zero-length segment outside",
			a:    pt2{9, 9}, b: pt2{9, 9},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "degenerate triangle: collinear corners",
			a:    pt2{-1, 1}, b: pt2{3, 1},
			ta: pt2{0, 0}, tb: pt2{2, 2}, tc: pt2{4, 4},
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "degenerate triangle: two corners coincide",
			a:    pt2{-1, 1}, b: pt2{3, 1},
			ta: pt2{0, 0}, tb: pt2{0, 0}, tc: pt2{4, 4},
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "clockwise winding, interior crossing",
			a:    pt2{-1, 1}, b: pt2{3, 1},
			ta: rev[0], tb: rev[1], tc: rev[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "clockwise winding, edge contact",
			a:    pt2{1, 0}, b: pt2{3, 0},
			ta: rev[0], tb: rev[1], tc: rev[2],
			wantIfDecided: true, assertExpected: true,
		},
		{
			name: "clockwise winding, vertex-only contact",
			a:    pt2{-2, 2}, b: pt2{0, 4},
			ta: rev[0], tb: rev[1], tc: rev[2],
			wantIfDecided: false, assertExpected: true,
		},
		{
			name: "reversed segment direction, interior crossing",
			a:    pt2{3, 1}, b: pt2{-1, 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
	}
	// Nearly coincident coordinates: the crossing sits one ulp away from a
	// corner, so the float filter inside cross2xSign cannot decide it and the
	// exact path carries the answer.
	near := math.Nextafter(4, math.Inf(1))
	cases = append(cases,
		segTriCase{
			name: "crossing one ulp inside the far corner",
			a:    pt2{-1, 0}, b: pt2{4, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		segTriCase{
			name: "crossing one ulp outside the far corner",
			a:    pt2{near, 0}, b: pt2{near + 1, 0},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: false, assertExpected: true,
		},
		segTriCase{
			name: "segment endpoints one ulp apart inside",
			a:    pt2{1, 1}, b: pt2{math.Nextafter(1, math.Inf(1)), 1},
			ta: tri[0], tb: tri[1], tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
		segTriCase{
			name: "triangle corner displaced by one ulp",
			a:    pt2{-1, 1}, b: pt2{3, 1},
			ta: tri[0], tb: pt2{near, 0}, tc: tri[2],
			wantIfDecided: true, assertExpected: true,
		},
	)
	// Widely varying finite magnitudes: a huge triangle clipped by a tiny
	// segment, and the reverse, both well inside float64's finite range.
	cases = append(cases,
		segTriCase{
			name: "tiny segment inside a huge triangle",
			a:    pt2{1e-8, 1e-8}, b: pt2{2e-8, 1e-8},
			ta: pt2{-1e12, -1e12}, tb: pt2{1e12, -1e12}, tc: pt2{0, 1e12},
			wantIfDecided: true, assertExpected: true,
		},
		segTriCase{
			name: "huge segment across a tiny triangle",
			a:    pt2{-1e12, 1e-9}, b: pt2{1e12, 1e-9},
			ta: pt2{0, 0}, tb: pt2{1e-8, 0}, tc: pt2{0, 1e-8},
			wantIfDecided: true, assertExpected: true,
		},
		segTriCase{
			name: "huge segment missing a tiny triangle",
			a:    pt2{-1e12, 1e-3}, b: pt2{1e12, 1e-3},
			ta: pt2{0, 0}, tb: pt2{1e-8, 0}, tc: pt2{0, 1e-8},
			wantIfDecided: false, assertExpected: true,
		},
		segTriCase{
			name: "mixed extreme magnitudes",
			a:    pt2{-1e150, 1}, b: pt2{1e150, 1},
			ta: pt2{-1e-150, 0}, tb: pt2{1e150, 0}, tc: pt2{0, 1e150},
			wantIfDecided: true, assertExpected: true,
		},
	)
	return cases
}

func TestSegTriOverlap2MatchesReference(t *testing.T) {
	for _, c := range segTriOverlapCases() {
		t.Run(c.name, func(t *testing.T) {
			requireSegTriParity(t, c)
		})
	}
}

// TestSegTriOverlap2MatchesReferenceRandomized draws configurations from a
// fixed seed, so the case set is identical on every run and on every host. The
// coordinates are drawn from a small integer-and-half lattice on purpose:
// coincident points, collinear triples and exact vertex contact all occur
// often, which is where the clip loop's boundary handling lives, and where
// randomly drawn floats would essentially never land.
func TestSegTriOverlap2MatchesReferenceRandomized(t *testing.T) {
	const (
		draws  = 1024
		scales = 4
	)
	rng := rand.New(rand.NewPCG(0x5eed1, 0x5eed2))
	// One scale per magnitude band, so a single drawn configuration can mix
	// coordinates twelve decades apart.
	scaleOf := [scales]float64{1, 0.5, 1e6, 1e-6}
	draw := func() pt2 {
		coord := func() float64 {
			return float64(rng.IntN(9)-4) / 2 * scaleOf[rng.IntN(scales)]
		}
		return pt2{x: coord(), y: coord()}
	}
	decided := 0
	for i := range draws {
		c := segTriCase{
			name: fmt.Sprintf("draw-%d", i),
			a:    draw(), b: draw(),
			ta: draw(), tb: draw(), tc: draw(),
		}
		requireSegTriParity(t, c)
		if segTriOverlap2(
			xp2Flavours[1].lift(c.a), xp2Flavours[1].lift(c.b),
			xp2Flavours[1].lift(c.ta), xp2Flavours[1].lift(c.tb), xp2Flavours[1].lift(c.tc),
		) {
			decided++
		}
	}
	// A draw that never reports an overlap would compare two implementations
	// on the trivial branch alone and prove nothing about the clip loop.
	require.Greater(t, decided, draws/20, "the random draw must produce overlaps, not only misses")
}

// TestClipFracComparesExactlyAcrossDenominators pins cmpClipFrac's contract
// directly: unnormalised fractions must order exactly as their reduced values
// do, including when one side is far from lowest terms.
func TestClipFracComparesExactlyAcrossDenominators(t *testing.T) {
	fracOf := func(num, den int64) clipFrac {
		return clipFrac{num: big.NewInt(num), den: big.NewInt(den)}
	}
	for _, c := range []struct {
		name string
		x, y clipFrac
		want int
	}{
		{name: "equal in lowest terms", x: fracOf(1, 3), y: fracOf(1, 3), want: 0},
		{name: "equal, one side unreduced", x: fracOf(7, 21), y: fracOf(1, 3), want: 0},
		{name: "equal, both sides unreduced", x: fracOf(1000, 3000), y: fracOf(7, 21), want: 0},
		{name: "below, unreduced", x: fracOf(6, 21), y: fracOf(1, 3), want: -1},
		{name: "above, unreduced", x: fracOf(8, 21), y: fracOf(1, 3), want: 1},
		{name: "zero against a positive", x: fracOf(0, 1), y: fracOf(3, 900000), want: -1},
		{name: "negative against zero", x: fracOf(-1, 900000), y: fracOf(0, 1), want: -1},
		{name: "two negatives", x: fracOf(-2, 3), y: fracOf(-1, 3), want: -1},
	} {
		t.Run(c.name, func(t *testing.T) {
			require.Equal(t, c.want, cmpClipFrac(c.x, c.y))
			require.Equal(t, -c.want, cmpClipFrac(c.y, c.x), "comparison must be antisymmetric")
			ratOf := func(f clipFrac) *big.Rat { return new(big.Rat).SetFrac(f.num, f.den) }
			require.Equal(t, ratOf(c.x).Cmp(ratOf(c.y)), cmpClipFrac(c.x, c.y),
				"unnormalised comparison must match the reduced one")
		})
	}
}

// TestEdgeCross2FracsMatchesCross2xUpToOneSharedFactor proves the contract
// segTriOverlap2 relies on: the two returned fractions are cross2x's exact
// values scaled by ONE common factor, so a ratio between them is unchanged.
// It is the property that lets the homogeneous denominator be dropped.
func TestEdgeCross2FracsMatchesCross2xUpToOneSharedFactor(t *testing.T) {
	pts := []pt2{{0, 0}, {4, 0}, {0, 4}, {1, 1}, {-3, 2.5}, {1e6, -1e-6}, {2, 2}}
	for _, lifting := range segTriLiftings() {
		t.Run(lifting.name, func(t *testing.T) {
			var ratios []*big.Rat
			for i, e0 := range pts {
				for j, e1 := range pts {
					if i == j {
						continue
					}
					a, b := lifting.seg.lift(pts[(i+2)%len(pts)]), lifting.seg.lift(pts[(j+3)%len(pts)])
					le0, le1 := lifting.tri.lift(e0), lifting.tri.lift(e1)
					fa, fb := edgeCross2Fracs(le0, le1, a, b)
					require.Positive(t, fa.den.Sign(), "denominators must stay positive")
					require.Positive(t, fb.den.Sign(), "denominators must stay positive")
					wantA := cross2x(le0, le1, a)
					wantB := cross2x(le0, le1, b)
					gotA := new(big.Rat).SetFrac(fa.num, fa.den)
					gotB := new(big.Rat).SetFrac(fb.num, fb.den)
					// Both fractions carry the same factor, so each matches
					// cross2x once divided by it. Recovering the factor from
					// one side and checking the other proves it is shared.
					if wantA.Sign() == 0 {
						require.Zero(t, gotA.Sign(), "a zero cross product must stay zero")
						continue
					}
					factor := new(big.Rat).Quo(gotA, wantA)
					require.Zero(t, new(big.Rat).Mul(wantB, factor).Cmp(gotB),
						"both fractions must carry the same factor")
					require.Positive(t, factor.Sign(), "the shared factor must be positive")
					ratios = append(ratios, factor)
				}
			}
			require.NotEmpty(t, ratios)
		})
	}
}

// benchSegTriPairs is the segment-and-triangle mix the clip benchmark runs.
// It deliberately holds every branch the loop can take — a full miss decided
// by signs alone, an edge crossing on each side, containment, edge contact and
// vertex contact — because the unnormalised path only changes the cost of the
// crossing branch, and a benchmark of crossings alone would overstate what a
// real audit sees.
func benchSegTriPairs() [][5]xp2 {
	cases := segTriOverlapCases()
	lift := xp2Flavours[1].lift
	out := make([][5]xp2, 0, len(cases))
	for _, c := range cases {
		out = append(out, [5]xp2{lift(c.a), lift(c.b), lift(c.ta), lift(c.tb), lift(c.tc)})
	}
	return out
}

// BenchmarkSegTriOverlap2 compares the shipped clip loop against the frozen
// pre-optimization reference in one process, on one input set, so the two arms
// differ only in whether the clip parameter is normalised.
func BenchmarkSegTriOverlap2(b *testing.B) {
	pairs := benchSegTriPairs()
	for _, arm := range []struct {
		name string
		fn   func(a, b, ta, tb, tc xp2) bool
	}{
		{name: "reference-rat", fn: referenceSegTriOverlap2},
		{name: "clipfrac", fn: segTriOverlap2},
	} {
		b.Run(arm.name, func(b *testing.B) {
			b.ReportAllocs()
			sink := 0
			for b.Loop() {
				for _, p := range pairs {
					if arm.fn(p[0], p[1], p[2], p[3], p[4]) {
						sink++
					}
				}
			}
			b.ReportMetric(float64(sink)/float64(b.N), "overlaps/op")
		})
	}
}
