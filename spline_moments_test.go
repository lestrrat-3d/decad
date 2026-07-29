package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// closedSplineControls is the fixture both the record and the dense reference
// are built from, so the two answers cannot drift apart.
var closedSplineControls = [][2]float64{{0, 0}, {4, 0}, {5, 3}, {2, 5}, {-1, 3}}

// scaledClosedSplineControls is the same section at three times the size. Its
// exact area is 9·293/18 = 293/2, which IS representable in float64 — so it
// exercises the Exact side of the spline design §3 rounding rule that the base
// fixture (293/18, not representable) exercises the Approximate side of.
func scaledClosedSplineControls() [][2]float64 {
	out := make([][2]float64, len(closedSplineControls))
	for i, control := range closedSplineControls {
		out[i] = [2]float64{control[0] * 3, control[1] * 3}
	}
	return out
}

func recordClosedSplineFrom(t *testing.T, controls [][2]float64) decad.ProfileRecord {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)

	points := make([]*sketch.Point, len(controls))
	for i, control := range controls {
		points[i] = s.CreatePoint(control[0], control[1])
	}
	_, err = s.CreateClosedSpline(points...)
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	require.Len(t, record.Outer.Segments, 1)
	require.IsType(t, decad.ClosedSplineSeg{}, record.Outer.Segments[0])
	return record
}

// densePolylineArea is the falsifier for the exact answer: a 400k-segment
// shoelace over sketch's own sampler. It never replaces the exact integral; it
// only disproves a wrong one.
func densePolylineArea(t *testing.T, controls [][2]float64) float64 {
	t.Helper()
	ring, err := geom.SamplePeriodicCubicBSpline(controls, 400000)
	require.NoError(t, err)
	var twice float64
	n := len(ring) - 1 // drop the repeated closing point
	for i := range n {
		j := (i + 1) % n
		twice += ring[i][0]*ring[j][1] - ring[j][0]*ring[i][1]
	}
	return math.Abs(twice / 2)
}

// The exact integral is 293/18, which float64 cannot represent, so spline
// design §3 requires Approximate with a bound of ONE rounding — not a
// quadrature-sized bound, and not a false Exact.
func TestClosedSplineProfileMomentsRoundOnce(t *testing.T) {
	record := recordClosedSplineFrom(t, closedSplineControls)

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, units.Area, area.Value.Kind())
	require.Equal(t, decad.Approximate, area.Exactness, "293/18 is not representable in float64")

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	// The correctly-rounded 293/18 — and it is what sketch's own arrangement reports.
	require.Equal(t, 293.0/18.0, value)
	require.InDelta(t, densePolylineArea(t, closedSplineControls), value, 1e-6,
		"the exact area survives a dense-sample falsifier")

	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.LessOrEqual(t, bound, math.Nextafter(value, math.Inf(1))-value,
		"the bound is a single rounding of the exact rational, not an estimate")

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 2.0, centroid.Value.X, 1e-15, "∫u dA / A is 293/9 ÷ 293/18 = 2")
	require.InDelta(t, 2.1941383606912619, centroid.Value.Y, 1e-12)
	require.Zero(t, centroid.Value.Z, "a plane-local centroid has no third coordinate")

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	for name, moment := range map[string]decad.Measurement{
		"UU": moments.UU,
		"UV": moments.UV,
		"VV": moments.VV,
	} {
		require.Equal(t, units.SecondMomentOfArea, moment.Value.Kind(), "%s kind", name)
	}
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 89.798354410507187, uu, 1e-9)
}

// The Exact side of the same rule: 293/2 IS representable, so the single
// rounding is no rounding at all and the bound is zero.
func TestClosedSplineProfileAreaExactWhenRepresentable(t *testing.T) {
	controls := scaledClosedSplineControls()
	record := recordClosedSplineFrom(t, controls)

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness, "293/2 is representable in float64")
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, bound, "an Exact measurement carries a zero bound")

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 293.0/2.0, value, "9 x 293/18 exactly")
	require.InDelta(t, densePolylineArea(t, controls), value, 1e-5)
}

// A spline chained with a straight chord exercises the mixed path: the line
// contributes through the existing exact-rational line formulas and the spline
// through the Bézier integrals, into one region.
func TestSplineAndChordProfileMoments(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	c1 := s.CreatePoint(1, 2)
	c2 := s.CreatePoint(3, 2)
	end := s.CreatePoint(4, 0)
	_, err = s.CreateSpline(start, c1, c2, end)
	require.NoError(t, err)
	s.CreateLine(end, start)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Greater(t, value, 0.0, "the region area is a positive magnitude")

	// The dense reference: the hump's sampled area closed by its chord.
	ring, err := geom.SampleCubicBSpline([][2]float64{{0, 0}, {1, 2}, {3, 2}, {4, 0}}, 200000)
	require.NoError(t, err)
	var twice float64
	for i := 0; i+1 < len(ring); i++ {
		twice += ring[i][0]*ring[i+1][1] - ring[i+1][0]*ring[i][1]
	}
	twice += ring[len(ring)-1][0]*ring[0][1] - ring[0][0]*ring[len(ring)-1][1]
	require.InDelta(t, math.Abs(twice/2), value, 1e-6)
}

func TestFreeformProfileRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		segment decad.CurveSegment
		message string
	}{
		{
			name:    "fit spline",
			segment: decad.FitSplineSeg{Fit: []decad.Point2{{}, {U: 4}}, TStart: 0, TEnd: 1},
			message: "interpolation solve",
		},
		{
			name: "elliptical arc",
			segment: decad.EllipticalArcSeg{
				Center:   decad.Point2{},
				Start:    decad.Point2{U: 2},
				End:      decad.Point2{V: 1},
				Rx:       units.Millimeters(2),
				Ry:       units.Millimeters(1),
				Rotation: units.Radians(0),
				TStart:   0,
				TEnd:     1,
			},
			message: "pinned endpoints",
		},
		{
			name: "conic",
			segment: decad.ConicSeg{
				Start: decad.Point2{}, Apex: decad.Point2{U: 1, V: 1}, End: decad.Point2{U: 2},
				Rho: 0.4, TStart: 0, TEnd: 1,
			},
			message: "closed form",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			record := decad.ProfileRecord{
				Outer: decad.LoopRecord{Segments: []decad.CurveSegment{tc.segment}},
			}
			_, err := record.Area()
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), tc.message)
		})
	}
}

// A caller-built record whose spline control points all coincide states no
// boundary. It must refuse rather than integrate a zero-area region into a
// centroid division.
func TestDegenerateSplineRecordRefuses(t *testing.T) {
	same := decad.Point2{U: 3, V: 3}
	record := decad.ProfileRecord{
		Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.ClosedSplineSeg{Control: []decad.Point2{same, same, same}, CCW: true, TStart: 0, TEnd: 1},
		}},
	}
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
