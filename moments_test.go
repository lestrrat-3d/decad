package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// recordOne solves the sketch and records the one profile matching pick.
func recordOne(t *testing.T, s *sketch.Sketch, pick func(*sketch.Profile) bool) decad.ProfileRecord {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if !pick(p) {
			continue
		}
		rec, _, err := decad.RecordProfile(s, p)
		require.NoError(t, err)
		return rec
	}
	t.Fatal(`no profile matched`)
	return decad.ProfileRecord{}
}

func TestRegionAreaAndCentroidRectangle(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(10, 20, 110, 80) // 100 × 60, off the origin
	s.Fix(rect.A)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return len(p.Outer) == 4 })

	area, err := rec.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness, `closed-form area is exact`)
	require.Zero(t, area.Bound.Mag(), `an exact measurement carries a zero bound`)
	require.True(t, area.Value.Equal(units.SquareMillimeters(6000), 1e-9), `a 100×60 plate is 6000 mm², got %s`, area.Value)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, c.Exactness)
	require.InDelta(t, 60.0, c.Value.X, 1e-9)
	require.InDelta(t, 50.0, c.Value.Y, 1e-9)
	require.Zero(t, c.Value.Z, `the centroid is plane-local: (u, v, 0)`)
}

func TestRegionAreaAndCentroidWithHole(t *testing.T) {
	// An off-center Ø20 hole: the composite centroid shifts away from it.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return len(p.Holes) == 1 })

	hole := math.Pi * 100
	wantArea := 6000 - hole
	area, err := rec.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, got, 1e-9, `net area subtracts the hole exactly`)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, (6000*50-hole*70)/wantArea, c.Value.X, 1e-9, `the centroid shifts away from the off-center hole`)
	require.InDelta(t, 30.0, c.Value.Y, 1e-9)
}

func TestRegionAreaWholeCircle(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return true })

	area, err := rec.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*49, got, 1e-9)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, -3.0, c.Value.Y, 1e-9)
}

func TestRegionAreaMatchesSketchOnCertifiedFragments(t *testing.T) {
	// A circle straddling a rectangle edge: every region's boundary mixes
	// whole edges with certified line and circle fragments. decad's
	// closed-form area must agree with sketch's own exact answer on every
	// region — the §1 falsifier consistency, exercised as a test.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(95, 30), 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	checked := 0
	for _, p := range s.Profiles() {
		if !p.Valid {
			continue
		}
		rec, _, err := decad.RecordProfile(s, p)
		require.NoError(t, err)
		area, err := rec.Area()
		require.NoError(t, err)
		got, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, p.Area, got, 1e-9, `decad's closed form should reproduce sketch's exact area`)
		checked++
	}
	require.GreaterOrEqual(t, checked, 3, `the arrangement should yield at least three regions`)
}

func TestRegionMomentsRejects(t *testing.T) {
	// A free-form boundary kind has no closed form here yet: ErrUnsupported,
	// never an approximation.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateEllipse(center, 20, 10, 0)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return true })
	_, err = rec.Area()
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = rec.Centroid()
	require.ErrorIs(t, err, decad.ErrUnsupported)

	// A zero-value record has no outer boundary, so it is not a region.
	var empty decad.ProfileRecord
	_, err = empty.Area()
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = empty.Centroid()
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = empty.SecondMoments()
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestRegionMomentsPointerVariants(t *testing.T) {
	// Pointer variants integrate exactly like their values — the same
	// normalization the codec applies — and a nil pointer is rejected.
	square := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		&decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 4, V: 0}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{U: 4, V: 0}, End: decad.Point2{U: 4, V: 4}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{U: 4, V: 4}, End: decad.Point2{U: 0, V: 4}, TStart: 0, TEnd: 1},
		&decad.LineSeg{Start: decad.Point2{U: 0, V: 4}, End: decad.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	area, err := square.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(16), 1e-12), `pointer segments integrate like values, got %s`, area.Value)
	centroid, err := square.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(2, 2, 0), centroid.Value)
	_, err = square.SecondMoments()
	require.NoError(t, err)

	bad := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{(*decad.LineSeg)(nil)}}}
	_, err = bad.Area()
	require.ErrorIs(t, err, decad.ErrDegenerate, `a nil segment pointer names no curve to integrate`)

	bad = decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{nil}}}
	requireProfileMomentError(t, bad, decad.ErrDegenerate)
}

func TestRegionMomentsRejectMalformedRecords(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) decad.CurveSegment {
		return decad.LineSeg{
			Start:  decad.Point2{U: u0, V: v0},
			End:    decad.Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}

	tests := []struct {
		name   string
		record decad.ProfileRecord
		target error
	}{
		{
			name: "OpenLoop",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(1, 0, 2, 0),
				line(2, 0, 2, 2),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "OpenLoopAtLargeCoordinate",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(1e200, 0, 1e200, 1),
				line(1e200, 2, 1e200, 0),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NonFiniteCoordinate",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(math.NaN(), 0, 1, 0),
				line(1, 0, 0, 1),
				line(0, 1, math.NaN(), 0),
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NonFiniteRange",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.LineSeg{
					Start:  decad.Point2{U: 0, V: 0},
					End:    decad.Point2{U: 1, V: 0},
					TStart: 0,
					TEnd:   math.Inf(1),
				},
				line(1, 0, 0, 1),
				line(0, 1, 0, 0),
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "FiniteArithmeticOverflow",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(0, 0, 1e200, 0),
				line(1e200, 0, 0, 1e200),
				line(0, 1e200, 0, 0),
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "WrongRadiusUnit",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Degrees(1),
					CCW:    true,
					TStart: 0,
					TEnd:   1,
				},
			}}},
			target: decad.ErrUnitKind,
		},
		{
			name: "NonFiniteRadius",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Millimeters(math.Inf(1)),
					CCW:    true,
					TStart: 0,
					TEnd:   1,
				},
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NegativeRadius",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Millimeters(-1),
					CCW:    true,
					TStart: 0,
					TEnd:   1,
				},
			}}},
			target: decad.ErrNegativeMagnitude,
		},
		{
			name: "InconsistentArcPins",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.ArcSeg{
					Center: decad.Point2{},
					Start:  decad.Point2{U: 1},
					End:    decad.Point2{V: 2},
					TStart: 0,
					TEnd:   1,
				},
				line(0, 1, 1, 0),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NearlyFullOpenCircle",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.CircleSeg{
					Radius: units.Millimeters(1e20),
					CCW:    true,
					TStart: 0,
					TEnd:   1 - 1e-13,
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

func TestRegionMomentsRejectMalformedArrangement(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) decad.CurveSegment {
		return decad.LineSeg{
			Start:  decad.Point2{U: u0, V: v0},
			End:    decad.Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	square := func(u0, v0, u1, v1 float64, clockwise bool) decad.LoopRecord {
		points := [][2]float64{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}
		if clockwise {
			points = [][2]float64{{u0, v0}, {u0, v1}, {u1, v1}, {u1, v0}}
		}
		return decad.LoopRecord{Segments: []decad.CurveSegment{
			line(points[0][0], points[0][1], points[1][0], points[1][1]),
			line(points[1][0], points[1][1], points[2][0], points[2][1]),
			line(points[2][0], points[2][1], points[3][0], points[3][1]),
			line(points[3][0], points[3][1], points[0][0], points[0][1]),
		}}
	}

	t.Run("WrongOuterWinding", func(t *testing.T) {
		record := decad.ProfileRecord{Outer: square(0, 0, 10, 10, true)}
		requireProfileMomentError(t, record, decad.ErrDegenerate)
	})

	tests := []struct {
		name  string
		holes []decad.LoopRecord
	}{
		{name: "WrongHoleWinding", holes: []decad.LoopRecord{square(2, 2, 3, 3, false)}},
		{name: "HoleOutsideOuter", holes: []decad.LoopRecord{square(12, 2, 13, 3, true)}},
		{name: "CrossingHole", holes: []decad.LoopRecord{square(-1, 2, 1, 3, true)}},
		{
			name: "NestedHoles",
			holes: []decad.LoopRecord{
				square(2, 2, 8, 8, true),
				square(3, 3, 4, 4, true),
			},
		},
		{
			name: "OverlappingHoles",
			holes: []decad.LoopRecord{
				square(2, 2, 6, 6, true),
				square(4, 4, 8, 8, true),
			},
		},
		{
			name: "DuplicateHoles",
			holes: []decad.LoopRecord{
				square(2, 2, 4, 4, true),
				square(2, 2, 4, 4, true),
			},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			record := decad.ProfileRecord{
				Outer: square(0, 0, 10, 10, false),
				Holes: test.holes,
			}
			requireProfileMomentError(t, record, decad.ErrDegenerate)
		})
	}
}

func TestRegionMomentsRejectCrossingAndOpenRecordsAtExtremeScales(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) decad.CurveSegment {
		return decad.LineSeg{
			Start:  decad.Point2{U: u0, V: v0},
			End:    decad.Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}

	const translatedU = 1e15
	tests := []struct {
		name   string
		record decad.ProfileRecord
	}{
		{
			name: "LargeCoordinateOpenGap",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(translatedU, 0, translatedU+10, 0),
				line(translatedU+10, 0, translatedU+10, 10),
				line(translatedU+10, 10, translatedU, 10),
				line(translatedU, 10, translatedU+1, 0),
			}}},
		},
		{
			name: "SmallCoordinateSelfCrossing",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(0, 0, 3e-6, 3e-6),
				line(3e-6, 3e-6, 0, 2e-6),
				line(0, 2e-6, 2e-6, 0),
				line(2e-6, 0, 0, 0),
			}}},
		},
		{
			name: "NearEndpointLineCrossing",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(0, 0, 1, 0),
				line(1, 0, 0.5, -1e-8),
				line(0.5, -1e-8, 0.5, 1),
				line(0.5, 1, 0, 0),
			}}},
		},
		{
			name: "NearVertexCircleCrossing",
			record: decad.ProfileRecord{
				Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
					line(-2, 0, 0, 0),
					line(0, 0, 2, 0),
					line(2, 0, 2, 3),
					line(2, 3, -2, 3),
					line(-2, 3, -2, 0),
				}},
				Holes: []decad.LoopRecord{{Segments: []decad.CurveSegment{
					decad.CircleSeg{
						Center: decad.Point2{U: 0, V: 1},
						Radius: units.Millimeters(1.000000000000001),
						TStart: 1,
						TEnd:   0,
					},
				}}},
			},
		},
		{
			name: "AdjacentLineArcSecondIntersection",
			record: decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				line(-2, 0.5, 1, 0),
				decad.ArcSeg{
					Center: decad.Point2{},
					Start:  decad.Point2{U: 1},
					End:    decad.Point2{U: -1},
					TStart: 0,
					TEnd:   1,
				},
				line(-1, 0, -2, 0.5),
			}}},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			requireProfileMomentError(t, test.record, decad.ErrDegenerate)
		})
	}
}

func TestRegionMomentsAcceptTranslatedNestedLoops(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) decad.CurveSegment {
		return decad.LineSeg{
			Start:  decad.Point2{U: u0, V: v0},
			End:    decad.Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	square := func(u0, v0, u1, v1 float64, clockwise bool) decad.LoopRecord {
		points := [][2]float64{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}
		if clockwise {
			points = [][2]float64{{u0, v0}, {u0, v1}, {u1, v1}, {u1, v0}}
		}
		return decad.LoopRecord{Segments: []decad.CurveSegment{
			line(points[0][0], points[0][1], points[1][0], points[1][1]),
			line(points[1][0], points[1][1], points[2][0], points[2][1]),
			line(points[2][0], points[2][1], points[3][0], points[3][1]),
			line(points[3][0], points[3][1], points[0][0], points[0][1]),
		}}
	}

	const translatedU = 1e15
	record := decad.ProfileRecord{
		Outer: square(translatedU, 0, translatedU+10, 10, false),
		Holes: []decad.LoopRecord{square(translatedU+4, 4, translatedU+6, 6, true)},
	}

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.True(t, area.Value.Equal(units.SquareMillimeters(96), 1e-12))

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, centroid.Exactness)
	require.False(t, math.IsNaN(centroid.Value.X))
	require.False(t, math.IsInf(centroid.Value.X, 0))
	require.InDelta(t, 5, centroid.Value.Y, 1e-12)
	require.Zero(t, centroid.Value.Z)

	second, err := record.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, second.UU.Exactness)
	require.Positive(t, second.UU.Value.Base())
	require.Positive(t, second.UV.Value.Base())
	require.Positive(t, second.VV.Value.Base())
}

func TestRegionMomentsAllowBoundaryTangency(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) decad.CurveSegment {
		return decad.LineSeg{
			Start:  decad.Point2{U: u0, V: v0},
			End:    decad.Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	record := decad.ProfileRecord{
		Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			line(0, 0, 10, 0),
			line(10, 0, 10, 10),
			line(10, 10, 0, 10),
			line(0, 10, 0, 0),
		}},
		Holes: []decad.LoopRecord{{Segments: []decad.CurveSegment{
			decad.CircleSeg{
				Center: decad.Point2{U: 1, V: 5},
				Radius: units.Millimeters(1),
				TStart: 1,
				TEnd:   0,
			},
		}}},
	}

	area, err := record.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(100-math.Pi), 1e-12))
}

func TestRegionMomentsAllowArcPinRoundoff(t *testing.T) {
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.ArcSeg{
			Center: decad.Point2{},
			Start:  decad.Point2{U: 1},
			End:    decad.Point2{V: math.Nextafter(1, 2)},
			TStart: 0,
			TEnd:   1,
		},
		decad.LineSeg{
			Start:  decad.Point2{V: 1},
			End:    decad.Point2{U: 1},
			TStart: 0,
			TEnd:   1,
		},
	}}}

	_, err := record.Area()
	require.NoError(t, err, `one ULP of pin roundoff must not disprove the arc record`)
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

func TestSecondMomentsRectangle(t *testing.T) {
	// For the axis-aligned rectangle [0,a]×[0,b] about the origin:
	// ∫u²dA = a³b/3, ∫uv dA = a²b²/4, ∫v²dA = ab³/3.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return len(p.Outer) == 4 })
	m, err := rec.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, m.UU.Exactness)
	uu, err := m.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := m.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := m.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 40.0*40*40*30/3, uu, 1e-6)
	require.InDelta(t, 40.0*40*30*30/4, uv, 1e-6)
	require.InDelta(t, 40.0*30*30*30/3, vv, 1e-6)
}

func TestSecondMomentsOffsetCircle(t *testing.T) {
	// Parallel-axis check on a Ø14 disk centered at (5, -3):
	// ∫u²dA = πr⁴/4 + c_u²·πr², ∫uv = c_u·c_v·πr², ∫v²dA = πr⁴/4 + c_v²·πr².
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)

	rec := recordOne(t, s, func(p *sketch.Profile) bool { return true })
	m, err := rec.SecondMoments()
	require.NoError(t, err)
	area := math.Pi * 49
	quarter := math.Pi * 7 * 7 * 7 * 7 / 4
	uu, err := m.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := m.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := m.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, quarter+25*area, uu, 1e-6)
	require.InDelta(t, 5.0*(-3)*area, uv, 1e-6)
	require.InDelta(t, quarter+9*area, vv, 1e-6)
}

func TestArcSegExactQuarterDisk(t *testing.T) {
	// The dedicated ArcSeg exact-value coverage: a quarter disk of radius 20
	// in the first quadrant, bounded by two lines and a real sketch arc.
	// Exact values: A = πR²/4, centroid (4R/3π, 4R/3π),
	// ∫u²dA = ∫v²dA = πR⁴/16, ∫uv dA = R⁴/8.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py) // CCW from (R,0) to (0,R): the quarter circle
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	rec, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)

	const R = 20.0
	area, err := rec.Area()
	require.NoError(t, err)
	gotA, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*R*R/4, gotA, 1e-9)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 4*R/(3*math.Pi), c.Value.X, 1e-9)
	require.InDelta(t, 4*R/(3*math.Pi), c.Value.Y, 1e-9)

	m, err := rec.SecondMoments()
	require.NoError(t, err)
	uu, err := m.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := m.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := m.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*R*R*R*R/16, uu, 1e-6)
	require.InDelta(t, R*R*R*R/8, uv, 1e-6)
	require.InDelta(t, math.Pi*R*R*R*R/16, vv, 1e-6)
}
