package coil_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func requirePoint(t *testing.T, iv coil.Iv, want *big.Rat, msg string) {
	t.Helper()
	require.Zero(t, iv.Lo.Cmp(want), msg)
	require.Zero(t, iv.Hi.Cmp(want), msg)
}

func requireHolds(t *testing.T, iv coil.Iv, want float64, msg string) {
	t.Helper()
	lo, _ := iv.Lo.Float64()
	hi, _ := iv.Hi.Float64()
	slack := 4 * proofbound.UlpOf(want)
	require.LessOrEqual(t, lo, want+slack, msg)
	require.GreaterOrEqual(t, hi, want-slack, msg)
}

// vAxis is the plane's V axis with the profile on +u, so Side is −1 and
// e_r = +u.
func vAxis() coil.Axis {
	zero, one := coil.Point(new(big.Rat)), coil.Point(big.NewRat(1, 1))
	return coil.Axis{AU: zero, AV: zero, DU: zero, DV: one, Side: -1}
}

func TestStationsAndTrig(t *testing.T) {
	n, ok := coil.StationCount(big.NewRat(2, 1), 256)
	require.True(t, ok)
	require.Equal(t, int64(512), n)
	n, ok = coil.StationCount(big.NewRat(13, 10), 256)
	require.True(t, ok)
	require.Equal(t, int64(333), n)
	require.Zero(t, coil.Fraction(big.NewRat(13, 10), 333, 333).Cmp(big.NewRat(13, 10)))

	for q, want := range [][2]int64{{0, 1}, {1, 0}, {0, -1}, {-1, 0}} {
		s, c := coil.TurnSinCos(big.NewRat(int64(q)+8, 4))
		requirePoint(t, s, big.NewRat(want[0], 1), "sin")
		requirePoint(t, c, big.NewRat(want[1], 1), "cos")
	}
	s, c := coil.TurnSinCos(big.NewRat(1, 12))
	requireHolds(t, s, 0.5, "sin 30°")
	requireHolds(t, c, math.Sqrt(3)/2, "cos 30°")
}

func TestCoordsAndMoments(t *testing.T) {
	ax := vAxis()
	rho, zeta := ax.Coords(big.NewRat(3, 1), big.NewRat(1, 1))
	requirePoint(t, rho, big.NewRat(3, 1), "ρ")
	requirePoint(t, zeta, big.NewRat(1, 1), "ζ")
	eu, ev := ax.Radial()
	requirePoint(t, eu, big.NewRat(1, 1), "e_r u")
	requirePoint(t, ev, new(big.Rat), "e_r v")

	us := []*big.Rat{big.NewRat(2, 1), big.NewRat(3, 1), big.NewRat(3, 1), big.NewRat(2, 1)}
	vs := []*big.Rat{new(big.Rat), new(big.Rat), big.NewRat(1, 1), big.NewRat(1, 1)}
	loops := [][]int{{0, 1, 2, 3}}
	area := coil.PolygonArea(us, vs, loops)
	require.Zero(t, area.Cmp(big.NewRat(1, 1)))
	rhos := make([]coil.Iv, 4)
	zetas := make([]coil.Iv, 4)
	for i := range us {
		rhos[i], zetas[i] = ax.Coords(us[i], vs[i])
	}
	m := coil.RegionMoments(rhos, zetas, loops, area, ax.Side)
	requirePoint(t, m.Q, big.NewRat(5, 2), "Q")
	requirePoint(t, m.I, big.NewRat(19, 3), "I")
	requirePoint(t, m.M, big.NewRat(5, 4), "M")

	vol := coil.Volume(m, big.NewRat(2, 1))
	requireHolds(t, vol, 10*math.Pi, "volume")
	r, tr, nn, ok := coil.CentroidCoefficients(m, big.NewRat(3, 2), big.NewRat(2, 1), 1)
	require.True(t, ok)
	requirePoint(t, r, new(big.Rat), "R at whole turns")
	requirePoint(t, tr, new(big.Rat), "T at whole turns")
	requirePoint(t, nn, big.NewRat(2, 1), "N")
}

func TestSegmentAreaAndLengths(t *testing.T) {
	pt := func(x int64) coil.Iv { return coil.Point(big.NewRat(x, 1)) }
	pitch, turns := big.NewRat(3, 2), big.NewRat(2, 1)
	band, ok := coil.SegmentArea(pt(3), pt(0), pt(3), pt(1), pitch, turns)
	require.True(t, ok)
	requireHolds(t, band, 12*math.Pi, "cylindrical band")

	k := 1.5 / (2 * math.Pi)
	F := func(x float64) float64 { return x/2*math.Sqrt(x*x+k*k) + k*k/2*math.Asinh(x/k) }
	ring, ok := coil.SegmentArea(pt(2), pt(0), pt(3), pt(0), pitch, turns)
	require.True(t, ok)
	requireHolds(t, ring, 4*math.Pi*(F(3)-F(2)), "annulus")
	back, ok := coil.SegmentArea(pt(3), pt(1), pt(2), pt(1), pitch, turns)
	require.True(t, ok)
	requireHolds(t, back, 4*math.Pi*(F(3)-F(2)), "annulus walked inward")

	// A cone: ρ 2 → 3 over ζ 0 → 1, L = √2, checked against Simpson.
	cone, ok := coil.SegmentArea(pt(2), pt(0), pt(3), pt(1), pitch, turns)
	require.True(t, ok)
	g := func(l float64) float64 { r := 2 + l; return math.Sqrt(2*r*r + k*k) }
	sum := g(0) + g(1)
	const n = 2000
	for i := 1; i < n; i++ {
		w := 4.0
		if i%2 == 0 {
			w = 2
		}
		sum += w * g(float64(i)/n)
	}
	want := 4 * math.Pi * sum / (3 * n)
	lo, _ := cone.Lo.Float64()
	require.InDelta(t, want, lo, 1e-9)

	hl, ok := coil.HelixLength(pt(3), pitch, turns)
	require.True(t, ok)
	requireHolds(t, hl, 2*math.Hypot(2*math.Pi*3, 1.5), "helix length")

	// ρ·π²·dt²/2 at ρ = 3, dt = 1/256.
	sag, _ := coil.SagUpper(big.NewRat(3, 1), big.NewRat(1, 256)).Float64()
	require.InDelta(t, 3*math.Pi*math.Pi/(2*256*256), sag, 1e-15)
}

func TestLoopsRefusesNonLines(t *testing.T) {
	square := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 2}, End: sectionrecord.Point2{U: 3}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 3}, End: sectionrecord.Point2{U: 3, V: 1}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 2, V: 1}, End: sectionrecord.Point2{U: 3, V: 1}, TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 2, V: 1}, End: sectionrecord.Point2{U: 2}, TStart: 0, TEnd: 1},
	}}
	pts, loops, err := coil.Loops(square, nil)
	require.NoError(t, err)
	require.Len(t, pts, 4)
	require.Equal(t, [][]int{{0, 1, 2, 3}}, loops)
	require.Equal(t, sectionrecord.Point2{U: 3, V: 1}, pts[2])

	trimmed := square
	trimmed.Segments = append([]sectionrecord.CurveSegment(nil), square.Segments...)
	trimmed.Segments[0] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 2}, End: sectionrecord.Point2{U: 3}, TStart: 0, TEnd: 0.5}
	_, _, err = coil.Loops(trimmed, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	arc := square
	arc.Segments = append([]sectionrecord.CurveSegment(nil), square.Segments...)
	arc.Segments[1] = sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: 3, V: 0.5}, Start: sectionrecord.Point2{U: 3}, End: sectionrecord.Point2{U: 3, V: 1}, TStart: 0, TEnd: 1}
	_, _, err = coil.Loops(arc, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
