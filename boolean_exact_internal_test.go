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
}
