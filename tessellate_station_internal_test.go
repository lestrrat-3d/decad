package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChordStationBoundEnclosesACircleSegStation(t *testing.T) {
	t.Parallel()
	// A quarter turn of a radius-5 circle, chorded in two: the interior station
	// sits at recorded parameter 1/8, whose turn is exactly rational, so the
	// enclosure comes from proofbound.TurnSinCosInterval and the held pair sits a couple of
	// roundings from it.
	const r = 5.0
	seg := CircleSeg{
		Center: Point2{U: 2, V: -3},
		Radius: units.Millimeters(r),
		CCW:    true,
		TStart: 0,
		TEnd:   0.25,
	}
	th := math.Pi / 4
	heldU, heldV := 2+r*math.Cos(th), -3+r*math.Sin(th)

	got := chordStationBound(seg, 1, 2, heldU, heldV)
	require.True(t, got.Derivable())
	limit := 4 * proofbound.UlpOf(r)
	require.LessOrEqual(t, math.Abs(got.U), limit, `the station's own u gap is a handful of ulps of the radius`)
	require.LessOrEqual(t, math.Abs(got.V), limit, `and so is its v gap`)

	// Falsifier: a held pair displaced by a visible amount is caught, so the
	// small answer above is a reading of this station and not a constant.
	off := chordStationBound(seg, 1, 2, heldU+1e-6, heldV)
	require.Greater(t, off.U, 9e-7)
	require.LessOrEqual(t, math.Abs(off.V), limit)
}

func TestChordStationBoundEnclosesAnArcSegStation(t *testing.T) {
	t.Parallel()
	// An arc states three pinned points and no angle at all, so its station goes
	// through proofbound.Atan2Interval and survey2d.RadSinCosSpan. The enclosure is wider than a
	// circle's, but it is finite and it is positive — never a silent zero.
	seg := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 3, V: 0},
		End:    Point2{U: 0, V: 3},
		TStart: 0,
		TEnd:   1,
	}
	w, err := walkOf(seg, freeform.NewFreeformWork())
	require.NoError(t, err)
	const n = 4
	dth := (w.Th1 - w.Th0) / n
	for k := 1; k < n; k++ {
		th := w.Th0 + float64(k)*dth
		heldU := w.CU + w.Radius*math.Cos(th)
		heldV := w.CV + w.Radius*math.Sin(th)
		got := chordStationBound(seg, k, n, heldU, heldV)
		require.True(t, got.Derivable(), `k=%d`, k)
		require.Positive(t, math.Max(got.U, got.V), `k=%d: an arc station is never held exactly`, k)
		require.Less(t, math.Max(got.U, got.V), 1e-12, `k=%d: and its enclosure stays at coordinate-rounding scale`, k)
	}
}

func TestChordStationBoundRefusesWhatItCannotEnclose(t *testing.T) {
	t.Parallel()
	circle := CircleSeg{Center: Point2{}, Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1}
	line := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}, TStart: 0, TEnd: 1}

	for _, row := range []struct {
		name    string
		seg     CurveSegment
		k, n    int
		heldU   float64
		heldV   float64
		message string
	}{
		{name: "a straight segment has no circular station", seg: line, k: 1, n: 2},
		{name: "the walk start is not an interior station", seg: circle, k: 0, n: 4},
		{name: "the walk end is not an interior station", seg: circle, k: 4, n: 4},
		{name: "an empty chording names no station", seg: circle, k: 1, n: 0},
	} {
		t.Run(row.name, func(t *testing.T) {
			got := chordStationBound(row.seg, row.k, row.n, row.heldU, row.heldV)
			require.False(t, got.Derivable())
			require.True(t, math.IsInf(got.U, 1))
			require.True(t, math.IsInf(got.V, 1))
		})
	}
}

func TestRequireDerivableStoreRefusesAnUnstatedDisplacement(t *testing.T) {
	t.Parallel()
	worst, err := requireDerivableStore([]float64{0, 3e-14, 1e-15})
	require.NoError(t, err)
	require.Equal(t, 3e-14, worst)

	_, err = requireDerivableStore([]float64{0, math.Inf(1)})
	require.ErrorIs(t, err, ErrUnsupported,
		`a sample whose own record states no enclosure refuses rather than publishing an infinite bound`)
}
