package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func recordOne(t *testing.T, s *sketch.Sketch, pick func(*sketch.Profile) bool) decad.ProfileRecord {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if !pick(profile) {
			continue
		}
		record, _, err := decad.RecordProfile(s, profile)
		require.NoError(t, err)
		return record
	}
	t.Fatal(`no profile matched`)
	return decad.ProfileRecord{}
}

func momentLine(u0, v0, u1, v1 float64) decad.CurveSegment {
	return decad.LineSeg{
		Start: decad.Point2{U: u0, V: v0},
		End:   decad.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1,
	}
}

func momentSquare(u0, v0, u1, v1 float64, clockwise bool) decad.LoopRecord {
	points := [][2]float64{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}
	if clockwise {
		points = [][2]float64{{u0, v0}, {u0, v1}, {u1, v1}, {u1, v0}}
	}
	return decad.LoopRecord{Segments: []decad.CurveSegment{
		momentLine(points[0][0], points[0][1], points[1][0], points[1][1]),
		momentLine(points[1][0], points[1][1], points[2][0], points[2][1]),
		momentLine(points[2][0], points[2][1], points[3][0], points[3][1]),
		momentLine(points[3][0], points[3][1], points[0][0], points[0][1]),
	}}
}

func momentWholeCircle(center decad.Point2, radius float64, counterclockwise bool) decad.LoopRecord {
	segment := decad.CircleSeg{
		Center: center,
		Radius: units.Millimeters(radius),
		CCW:    counterclockwise,
	}
	if counterclockwise {
		segment.TEnd = 1
	} else {
		segment.TStart = 1
	}
	return decad.LoopRecord{Segments: []decad.CurveSegment{segment}}
}

func requireProfileMomentError(t *testing.T, record decad.ProfileRecord, target error) {
	t.Helper()
	_, err := record.Area()
	require.ErrorIs(t, err, target)
	_, err = record.Centroid()
	require.ErrorIs(t, err, target)
	_, err = record.SecondMoments()
	require.ErrorIs(t, err, target)
}

func TestRegionAreaAndCentroidRectangle(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(10, 20, 110, 80)
	s.Fix(rect.A)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Outer) == 4 })

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Zero(t, area.Bound.Mag())
	require.True(t, area.Value.Equal(units.SquareMillimeters(6000), 1e-9))

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, centroid.Exactness)
	require.InDelta(t, 60, centroid.Value.X, 1e-9)
	require.InDelta(t, 50, centroid.Value.Y, 1e-9)
	require.Zero(t, centroid.Value.Z)
}

func TestRegionAreaAndCentroidWithHole(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Holes) == 1 })

	holeArea := math.Pi * 100
	wantArea := 6000 - holeArea
	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, got, 1e-9)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, (6000*50-holeArea*70)/wantArea, centroid.Value.X, 1e-9)
	require.InDelta(t, 30, centroid.Value.Y, 1e-9)
}

func TestRegionAreaWholeCircle(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*49, got, 1e-9)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5, centroid.Value.X, 1e-9)
	require.InDelta(t, -3, centroid.Value.Y, 1e-9)
}

func TestRegionAreaMatchesSketchOnCertifiedFragments(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(95, 30), 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	checked := 0
	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := decad.RecordProfile(s, profile)
		require.NoError(t, err)
		area, err := record.Area()
		require.NoError(t, err)
		got, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, profile.Area, got, 1e-9)
		checked++
	}
	require.GreaterOrEqual(t, checked, 3)
}

func TestRegionMomentsRejectUnsupportedAndEmptyRecords(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateEllipse(center, 20, 10, 0)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })
	requireProfileMomentError(t, record, decad.ErrUnsupported)

	requireProfileMomentError(t, decad.ProfileRecord{}, decad.ErrDegenerate)
}

func TestRegionMomentsPointerVariants(t *testing.T) {
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		&decad.LineSeg{Start: decad.Point2{}, End: decad.Point2{U: 4}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{U: 4}, End: decad.Point2{U: 4, V: 4}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{U: 4, V: 4}, End: decad.Point2{V: 4}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{V: 4}, End: decad.Point2{}, TStart: 0, TEnd: 1},
	}}}
	area, err := record.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(16), 1e-12))

	bad := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{(*decad.LineSeg)(nil)}}}
	requireProfileMomentError(t, bad, decad.ErrDegenerate)
	bad.Outer.Segments[0] = nil
	requireProfileMomentError(t, bad, decad.ErrDegenerate)
}

func TestRegionMomentsRejectMalformedFields(t *testing.T) {
	tests := []struct {
		name   string
		record decad.ProfileRecord
		target error
	}{
		{
			name: "OpenLoop",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				momentLine(0, 0, 1, 0), momentLine(1, 0, 1, 1),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NonFiniteCoordinate",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				momentLine(math.NaN(), 0, 1, 0),
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NonFiniteRange",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.LineSeg{Start: decad.Point2{}, End: decad.Point2{U: 1}, TEnd: math.Inf(1)},
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "WrongRadiusUnit",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{Radius: units.Degrees(1), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrUnitKind,
		},
		{
			name: "NonFiniteRadius",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{Radius: units.Millimeters(math.Inf(1)), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NegativeRadius",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{Radius: units.Millimeters(-1), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrNegativeMagnitude,
		},
		{
			name: "InconsistentArcPins",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.ArcSeg{
					Center: decad.Point2{}, Start: decad.Point2{U: 1}, End: decad.Point2{V: 2}, TEnd: 1,
				},
				momentLine(0, 1, 1, 0),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NearlyFullCircle",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Millimeters(1), CCW: true, TEnd: math.Nextafter(1, 0),
				},
			}}},
			target: decad.ErrDegenerate,
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			requireProfileMomentError(t, test.record, test.target)
		})
	}
}

func TestRegionMomentsRejectMalformedTopology(t *testing.T) {
	tests := []struct {
		name   string
		record decad.ProfileRecord
	}{
		{
			name: "CrossingOuter",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				momentLine(0, 0, 4, 4),
				momentLine(4, 4, 0, 4),
				momentLine(0, 4, 4, 0),
				momentLine(4, 0, 0, 0),
			}}},
		},
		{name: "WrongOuterWinding", record: decad.ProfileRecord{Outer: momentSquare(0, 0, 10, 10, true)}},
		{
			name: "HoleOutsideOuter",
			record: decad.ProfileRecord{
				Outer: momentSquare(0, 0, 10, 10, false),
				Holes: []decad.LoopRecord{
					momentSquare(12, 2, 13, 3, true),
				},
			},
		},
		{
			name: "OverlappingHoles",
			record: decad.ProfileRecord{
				Outer: momentSquare(0, 0, 10, 10, false),
				Holes: []decad.LoopRecord{
					momentSquare(2, 2, 6, 6, true),
					momentSquare(4, 4, 8, 8, true),
				},
			},
		},
		{
			name: "OuterHoleInternalTangency",
			record: decad.ProfileRecord{
				Outer: momentWholeCircle(decad.Point2{}, 10, true),
				Holes: []decad.LoopRecord{
					momentWholeCircle(decad.Point2{U: 5}, 5, false),
				},
			},
		},
		{
			name: "HoleHoleExternalTangency",
			record: decad.ProfileRecord{
				Outer: momentWholeCircle(decad.Point2{}, 10, true),
				Holes: []decad.LoopRecord{
					momentWholeCircle(decad.Point2{U: -2}, 2, false),
					momentWholeCircle(decad.Point2{U: 2}, 2, false),
				},
			},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			requireProfileMomentError(t, test.record, decad.ErrDegenerate)
		})
	}
}

func TestRegionMomentsAcceptThinAnnulus(t *testing.T) {
	const outerRadius = 10.0
	const gap = 1e-10
	holeRadius := outerRadius - gap
	record := decad.ProfileRecord{
		Outer: momentWholeCircle(decad.Point2{}, outerRadius, true),
		Holes: []decad.LoopRecord{
			momentWholeCircle(decad.Point2{}, holeRadius, false),
		},
	}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*(outerRadius*outerRadius-holeRadius*holeRadius), got, 1e-12)
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.NoError(t, err)
}

func TestRegionMomentsAcceptSeparatedWholeCircleHoles(t *testing.T) {
	record := decad.ProfileRecord{
		Outer: momentWholeCircle(decad.Point2{}, 10, true),
		Holes: []decad.LoopRecord{
			momentWholeCircle(decad.Point2{U: -2.0000000001}, 2, false),
			momentWholeCircle(decad.Point2{U: 2.0000000001}, 2, false),
		},
	}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 92*math.Pi, got, 1e-12)
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.NoError(t, err)
}

func TestRegionMomentsRequestedOrderControlsOverflow(t *testing.T) {
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.CircleSeg{
			Center: decad.Point2{V: 1e200},
			Radius: units.Millimeters(1),
			CCW:    true,
			TEnd:   1,
		},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(math.Pi), 1e-12))
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.ErrorIs(t, err, decad.ErrNotFinite)
}

func TestSecondMomentsRectangle(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Outer) == 4 })

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 40.0*40*40*30/3, uu, 1e-6)
	require.InDelta(t, 40.0*40*30*30/4, uv, 1e-6)
	require.InDelta(t, 40.0*30*30*30/3, vv, 1e-6)
}

func TestSecondMomentsOffsetCircle(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	area := math.Pi * 49
	quarter := math.Pi * 7 * 7 * 7 * 7 / 4
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, quarter+25*area, uu, 1e-6)
	require.InDelta(t, -15*area, uv, 1e-6)
	require.InDelta(t, quarter+9*area, vv, 1e-6)
}

func TestArcSegExactQuarterDisk(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	const radius = 20.0
	area, err := record.Area()
	require.NoError(t, err)
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*radius*radius/4, gotArea, 1e-9)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.X, 1e-9)
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.Y, 1e-9)

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, uu, 1e-6)
	require.InDelta(t, math.Pow(radius, 4)/8, uv, 1e-6)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, vv, 1e-6)
}
