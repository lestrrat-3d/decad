package decad_test

import (
	"math"
	"math/big"
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

func requireBoundContainsBig(t *testing.T, got, bound float64, want *big.Float) {
	t.Helper()
	const precision = 256
	diff := new(big.Float).SetPrec(precision).Sub(
		new(big.Float).SetPrec(precision).SetFloat64(got),
		want,
	)
	diff.Abs(diff)
	heldBound := new(big.Float).SetPrec(precision).SetFloat64(bound)
	require.GreaterOrEqual(t, heldBound.Cmp(diff), 0, "error %s exceeds bound %.17g", diff.Text('g', 18), bound)
}

func precisePi(t *testing.T) *big.Float {
	t.Helper()
	const digits = "3.141592653589793238462643383279502884197169399375105820974944592307816406286"
	pi, ok := new(big.Float).SetPrec(256).SetString(digits)
	require.True(t, ok)
	return pi
}

func preciseAtan(t *testing.T, x *big.Rat) *big.Float {
	t.Helper()
	const precision = uint(256)
	xSquared := new(big.Rat).Mul(x, x)
	term := new(big.Rat).Set(x)
	sum := new(big.Rat)
	for n := range 256 {
		addend := new(big.Rat).Quo(term, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, addend)
		} else {
			sum.Sub(sum, addend)
		}
		term.Mul(term, xSquared)
	}
	return new(big.Float).SetPrec(precision).SetRat(sum)
}

func preciseGreenArcArea(t *testing.T, center, start, end decad.Point2) *big.Float {
	t.Helper()
	const precision = uint(256)
	floatRat := func(value float64) *big.Rat {
		return new(big.Rat).SetFloat64(value)
	}
	bigFloatRat := func(value *big.Rat) *big.Float {
		return new(big.Float).SetPrec(precision).SetRat(value)
	}
	bigFloat := func(value float64) *big.Float {
		return new(big.Float).SetPrec(precision).SetFloat64(value)
	}

	dx0 := new(big.Rat).Sub(floatRat(start.U), floatRat(center.U))
	dy0 := new(big.Rat).Sub(floatRat(start.V), floatRat(center.V))
	dx1 := new(big.Rat).Sub(floatRat(end.U), floatRat(center.U))
	dy1 := new(big.Rat).Sub(floatRat(end.V), floatRat(center.V))
	r0Squared := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
	r1Squared := new(big.Rat).Add(new(big.Rat).Mul(dx1, dx1), new(big.Rat).Mul(dy1, dy1))
	r0 := new(big.Float).SetPrec(precision).Sqrt(bigFloatRat(r0Squared))
	r1 := new(big.Float).SetPrec(precision).Sqrt(bigFloatRat(r1Squared))

	angleArg := new(big.Rat).Quo(
		new(big.Rat).Sub(dy1, dx1),
		new(big.Rat).Add(dy1, dx1),
	)
	theta := new(big.Float).SetPrec(precision).Quo(precisePi(t), bigFloat(4))
	theta.Add(theta, preciseAtan(t, angleArg))

	endSin := new(big.Float).SetPrec(precision).Quo(bigFloatRat(dy1), r1)
	endSin.Mul(endSin, r0)
	endCos := new(big.Float).SetPrec(precision).Quo(bigFloatRat(dx1), r1)
	endCos.Mul(endCos, r0)

	arc := new(big.Float).SetPrec(precision).Mul(new(big.Float).SetPrec(precision).Mul(r0, r0), theta)
	centerU, centerV := bigFloat(center.U), bigFloat(center.V)
	arc.Add(arc, new(big.Float).SetPrec(precision).Mul(
		centerU,
		new(big.Float).SetPrec(precision).Sub(endSin, bigFloatRat(dy0)),
	))
	arc.Sub(arc, new(big.Float).SetPrec(precision).Mul(
		centerV,
		new(big.Float).SetPrec(precision).Sub(endCos, bigFloatRat(dx0)),
	))
	arc.Mul(arc, bigFloat(0.5))

	startU, startV := bigFloat(start.U), bigFloat(start.V)
	endU, endV := bigFloat(end.U), bigFloat(end.V)
	line := new(big.Float).SetPrec(precision).Mul(endU, centerV)
	line.Sub(line, new(big.Float).SetPrec(precision).Mul(endV, centerU))
	line.Add(line, new(big.Float).SetPrec(precision).Mul(centerU, startV))
	line.Sub(line, new(big.Float).SetPrec(precision).Mul(centerV, startU))
	line.Mul(line, bigFloat(0.5))
	return arc.Add(arc, line)
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
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*49, got, 1e-9)
	wantArea := new(big.Float).SetPrec(256).Mul(
		precisePi(t),
		new(big.Float).SetPrec(256).SetFloat64(49),
	)
	requireBoundContainsBig(t, got, area.Bound.Base(), wantArea)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	require.InDelta(t, 5.0, centroid.Value.X, 1e-9)
	require.InDelta(t, -3.0, centroid.Value.Y, 1e-9)
	require.LessOrEqual(t, math.Hypot(centroid.Value.X-5, centroid.Value.Y+3), centroid.Bound.Base())
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

func TestCircleSegMomentRadiusErrorsUseDecadSentinels(t *testing.T) {
	calls := []struct {
		name string
		call func(decad.ProfileRecord) error
	}{
		{
			name: "area",
			call: func(rec decad.ProfileRecord) error {
				_, err := rec.Area()
				return err
			},
		},
		{
			name: "centroid",
			call: func(rec decad.ProfileRecord) error {
				_, err := rec.Centroid()
				return err
			},
		},
		{
			name: "second moments",
			call: func(rec decad.ProfileRecord) error {
				_, err := rec.SecondMoments()
				return err
			},
		},
	}
	radii := []struct {
		name       string
		radius     units.Value
		want       error
		dependency error
	}{
		{
			name:       "wrong kind",
			radius:     units.Degrees(1),
			want:       decad.ErrUnitKind,
			dependency: units.ErrIncompatible,
		},
		{
			name:       "NaN",
			radius:     units.Millimeters(math.NaN()),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
		{
			name:       "infinite",
			radius:     units.Millimeters(math.Inf(1)),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
		{
			name:       "conversion overflow",
			radius:     units.Meters(math.MaxFloat64),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
	}
	forms := []struct {
		name string
		seg  func(units.Value) decad.CurveSegment
	}{
		{
			name: "value",
			seg: func(radius units.Value) decad.CurveSegment {
				return decad.CircleSeg{Radius: radius, CCW: true, TEnd: 1}
			},
		},
		{
			name: "pointer",
			seg: func(radius units.Value) decad.CurveSegment {
				return &decad.CircleSeg{Radius: radius, CCW: true, TEnd: 1}
			},
		},
	}

	for _, radius := range radii {
		for _, form := range forms {
			rec := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				form.seg(radius.radius),
			}}}
			for _, call := range calls {
				t.Run(radius.name+"/"+form.name+"/"+call.name, func(t *testing.T) {
					err := call.call(rec)
					require.ErrorIs(t, err, radius.want)
					require.NotErrorIs(t, err, radius.dependency,
						`mass properties expose decad's sentinel, not the units dependency error`)
				})
			}
		}
	}
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
	require.Equal(t, decad.Exact, moments.UU.Exactness)
	require.Equal(t, decad.Exact, moments.UV.Exactness)
	require.Equal(t, decad.Exact, moments.VV.Exactness)
	require.Zero(t, moments.UU.Bound.Base())
	require.Zero(t, moments.UV.Bound.Base())
	require.Zero(t, moments.VV.Bound.Base())
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
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	require.Equal(t, decad.Approximate, moments.UV.Exactness)
	require.Equal(t, decad.Approximate, moments.VV.Exactness)
	require.Positive(t, moments.UU.Bound.Base())
	require.Positive(t, moments.UV.Bound.Base())
	require.Positive(t, moments.VV.Bound.Base())
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

func TestLineRationalRoundingIsBounded(t *testing.T) {
	rec := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 1, V: 0}, End: decad.Point2{U: 0, V: 1}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 0, V: 1}, End: decad.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}

	area, err := rec.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Zero(t, area.Bound.Base())

	centroid, err := rec.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	want := 1.0 / 3
	require.LessOrEqual(
		t,
		math.Hypot(centroid.Value.X-want, centroid.Value.Y-want),
		centroid.Bound.Base(),
	)

	moments, err := rec.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	got, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got-1.0/12), moments.UU.Bound.Base())
}

// The underflow reading is not free-form-specific: the line path integrates to
// exact rationals too, so a square of four LineSegs whose exact area is strictly
// positive and below the smallest float64 owes the same bounded zero — value 0
// with the rounding that produced it as the bound — rather than a refusal for
// enclosing no positive area.
func TestUnderflowingLineRegionAreaPublishesBoundedZero(t *testing.T) {
	const side = 1e-163
	record := decad.ProfileRecord{Outer: momentSquare(0, 0, side, side, false)}

	area, err := record.Area()
	require.NoError(t, err, "the exact rational area is side², which is strictly positive")
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, value, "no float64 holds 1e-326")
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base(), "the bound is the rounding that produced the zero")
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
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.X, 1e-9)
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.Y, 1e-9)
	wantArea := new(big.Float).SetPrec(256).Mul(
		precisePi(t),
		new(big.Float).SetPrec(256).SetFloat64(radius*radius/4),
	)
	requireBoundContainsBig(t, gotArea, area.Bound.Base(), wantArea)

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	wantCentroid := 4 * radius / (3 * math.Pi)
	require.LessOrEqual(t, math.Hypot(centroid.Value.X-wantCentroid, centroid.Value.Y-wantCentroid), centroid.Bound.Base())
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	require.Equal(t, decad.Approximate, moments.UV.Exactness)
	require.Equal(t, decad.Approximate, moments.VV.Exactness)
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, uu, 1e-6)
	require.InDelta(t, math.Pow(radius, 4)/8, uv, 1e-6)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, vv, 1e-6)
	require.LessOrEqual(t, math.Abs(uu-math.Pi*math.Pow(radius, 4)/16), moments.UU.Bound.Base())
	require.LessOrEqual(t, math.Abs(uv-math.Pow(radius, 4)/8), moments.UV.Bound.Base())
	require.LessOrEqual(t, math.Abs(vv-math.Pi*math.Pow(radius, 4)/16), moments.VV.Bound.Base())
}

func TestRegionMomentsAcceptsGeneratedArcEndpointDrift(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(1.23456789, -2.34567891)
	s.Fix(center)
	start := s.CreatePoint(17.89101112, 4.32109876)
	s.Fix(start)
	end := s.CreatePoint(3.33333333, 18.7654321)
	s.CreateLine(center, start)
	s.CreateArc(center, start, end)
	s.CreateLine(end, center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := decad.RecordProfile(s, profile)
		require.NoError(t, err)
		_, err = record.Centroid()
		require.NoError(t, err)
		return
	}
	t.Fatal("no valid profile generated")
}

func TestRegionAreaBoundContainsArcEndpointDriftGreenIntegral(t *testing.T) {
	center := decad.Point2{U: 5, V: -3}
	start := decad.Point2{U: 6, V: -3}
	driftedRadius := math.Nextafter(1, math.Inf(1))
	end := decad.Point2{
		U: 5 + 0.6*driftedRadius,
		V: -3 + 0.8*driftedRadius,
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: end, End: center, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: center, End: start, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	want := preciseGreenArcArea(t, center, start, end)
	requireBoundContainsBig(t, got, area.Bound.Base(), want)
}
