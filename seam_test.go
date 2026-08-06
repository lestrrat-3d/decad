package decad_test

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRecordProfileRectangle(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	rec, plane, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err, `a clean rectangle should record`)

	// The plane is the XY datum, recorded as vectors.
	require.Equal(t, r3.NewVec(0, 0, 0), plane.Origin)
	require.Equal(t, r3.NewVec(1, 0, 0), plane.U)
	require.Equal(t, r3.NewVec(0, 1, 0), plane.V)

	// Four whole line edges, each spanning its full domain; every recorded
	// coordinate is the entity's own.
	require.Len(t, rec.Outer.Segments, 4)
	require.Empty(t, rec.Holes)
	corners := map[decad.Point2]int{}
	for _, seg := range rec.Outer.Segments {
		line, ok := seg.(decad.LineSeg)
		require.True(t, ok, `a rectangle boundary records as LineSegs, got %T`, seg)
		// A whole edge spans the full domain; the walk may run it backwards,
		// which the range order (not the fields) carries.
		lo, hi := line.TStart, line.TEnd
		if lo > hi {
			lo, hi = hi, lo
		}
		require.Equal(t, 0.0, lo)
		require.Equal(t, 1.0, hi)
		corners[line.Start]++
		corners[line.End]++
	}
	// Four corners, each shared by exactly two of the recorded lines.
	require.Len(t, corners, 4)
	for c, n := range corners {
		require.Equal(t, 2, n, `corner %v should be shared by exactly two lines`, c)
	}
	require.Contains(t, corners, decad.Point2{U: 0, V: 0})
	require.Contains(t, corners, decad.Point2{U: 100, V: 60})

	// The whole record is a value that round-trips.
	buf, err := json.Marshal(rec)
	require.NoError(t, err)
	var got decad.ProfileRecord
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, rec, got)
}

func TestRecordProfileCircleHole(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	center := s.CreatePoint(50, 30)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// Find the rectangle-minus-circle region: four lines outside, one hole.
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-hole region should exist`)

	rec, _, err := decad.RecordProfile(s, prof)
	require.NoError(t, err)
	require.Len(t, rec.Holes, 1)
	require.Len(t, rec.Holes[0].Segments, 1, `a circle bounds a hole loop on its own`)

	hole, ok := rec.Holes[0].Segments[0].(decad.CircleSeg)
	require.True(t, ok, `the hole should record as a CircleSeg, got %T`, rec.Holes[0].Segments[0])
	require.Equal(t, decad.Point2{U: 50, V: 30}, hole.Center)
	require.True(t, hole.Radius.Equal(units.Millimeters(10), 1e-9), `the hole radius should be the entity's own 10 mm, got %s`, hole.Radius)
	// A hole is walked clockwise — against the circle's natural CCW — so the
	// walk is baked in: CCW false, range order reversed.
	require.False(t, hole.CCW, `a hole walks clockwise`)
	require.Greater(t, hole.TStart, hole.TEnd, `a reversed walk carries TStart > TEnd`)
}

func TestRecordProfileCertifiedFragments(t *testing.T) {
	// A circle crossed by a rectangle: every cut is a line-involved
	// line/circle crossing, which sketch's closed-form kernel certifies —
	// the fragments record, each through the falsifier.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	center := s.CreatePoint(95, 30) // straddling the rectangle's right edge
	s.CreateCircle(center, 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// Every region of this arrangement is recordable; at least one boundary
	// must carry a certified circle fragment (the circle is split by the
	// rectangle's edge into an inside and an outside piece).
	profiles := s.Profiles()
	require.NotEmpty(t, profiles)
	fragments := 0
	for _, p := range profiles {
		if !p.Valid {
			continue
		}
		rec, _, err := decad.RecordProfile(s, p)
		require.NoError(t, err, `every certified-cut region should record`)
		for _, loop := range append([]decad.LoopRecord{rec.Outer}, rec.Holes...) {
			for _, seg := range loop.Segments {
				c, ok := seg.(decad.CircleSeg)
				if !ok {
					continue
				}
				lo, hi := c.TStart, c.TEnd
				if lo > hi {
					lo, hi = hi, lo
				}
				if lo > 0 || hi < 1 {
					fragments++
					require.Greater(t, hi, lo, `a fragment's range is non-degenerate`)
				}
			}
		}
	}
	require.NotZero(t, fragments, `the crossed circle should record at least one certified fragment`)
}

func TestRecordProfileRejectsSampledCuts(t *testing.T) {
	// A spline crossing a circle: free-form intersection has no closed form
	// upstream (docs/spline-design.md §9 ask 3), so sketch reports the cut
	// sampled (TExact false) and decad refuses to record the fragment — never
	// approximately, never as the whole curve.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	_, err = s.CreateSpline(
		s.CreatePoint(-30, -30), s.CreatePoint(-10, 30),
		s.CreatePoint(10, -30), s.CreatePoint(30, 30),
	)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.NotEmpty(t, profiles)
	rejected := 0
	for _, p := range profiles {
		if !p.Valid {
			continue
		}
		_, _, err := decad.RecordProfile(s, p)
		if err != nil {
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
			rejected++
		}
	}
	require.NotZero(t, rejected, `regions bounded by sampled spline cuts should be rejected`)
}

func TestRecordProfileRejectsWholeSketchTExactWithholding(t *testing.T) {
	// A distant spline withholds certification but leaves the analytic
	// circle/circle fragments and their ranges unchanged.
	newProfiles := func(withSpline bool) (*sketch.Sketch, []*sketch.Profile) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		s.CreateCircle(s.CreatePoint(0, 0), 5)
		s.CreateCircle(s.CreatePoint(6, 0), 5)
		if withSpline {
			_, err = s.CreateSpline(
				s.CreatePoint(40, 0), s.CreatePoint(42, 4),
				s.CreatePoint(46, 4), s.CreatePoint(48, 0),
			)
			require.NoError(t, err)
		}
		return s, s.Profiles()
	}

	certifiedSketch, certified := newProfiles(false)
	withheldSketch, withheld := newProfiles(true)
	require.Len(t, certified, 3, `the circles should form two caps and one lens`)
	require.Len(t, withheld, len(certified), `the distant spline should not change the profile set`)
	for i, certifiedProfile := range certified {
		withheldProfile := withheld[i]
		require.Len(t, withheldProfile.Outer, len(certifiedProfile.Outer))
		for j, certifiedEdge := range certifiedProfile.Outer {
			withheldEdge := withheldProfile.Outer[j]
			require.True(t, certifiedEdge.Partial)
			require.True(t, certifiedEdge.TExact, `the circle/circle bound should be certified without the spline`)
			require.True(t, withheldEdge.Partial)
			require.False(t, withheldEdge.TExact, `the whole-sketch gate should withhold certification`)
			require.Equal(t, certifiedEdge.TStart, withheldEdge.TStart, `the gate must not change the start parameter`)
			require.Equal(t, certifiedEdge.TEnd, withheldEdge.TEnd, `the gate must not change the end parameter`)
		}
		_, _, err := decad.RecordProfile(certifiedSketch, certifiedProfile)
		require.NoError(t, err, `the certified circle profile should record`)
		_, _, err = decad.RecordProfile(withheldSketch, withheldProfile)
		require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	}
}

func TestRecordProfileReportsOuterEdgeIndex(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	_, err = s.CreateSpline(
		s.CreatePoint(-30, -5), s.CreatePoint(-5, 15),
		s.CreatePoint(5, -15), s.CreatePoint(30, 5),
	)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	// The reported index is the FIRST rejected edge, so a fixture whose whole
	// edge comes before its sampled one proves the index is walked, not zero.
	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !candidate.Valid || len(candidate.Outer) != 4 {
			continue
		}
		if !candidate.Outer[0].Partial && candidate.Outer[1].Partial {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-cut-by-a-spline region should exist`)
	require.True(t, prof.Outer[1].Partial)
	require.False(t, prof.Outer[1].TExact)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.ErrorContains(t, err, `outer edge 1`)
}

func TestRecordProfileReportsHoleAndEdgeIndices(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-80, -50, 80, 50)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(-50, 0), 8)
	s.CreateCircle(s.CreatePoint(20, 0), 20)
	_, err = s.CreateClosedSpline(
		s.CreatePoint(45, -18), s.CreatePoint(60, 0),
		s.CreatePoint(45, 18), s.CreatePoint(30, 0),
	)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 4 && len(candidate.Holes) == 2 {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-two-holes region should exist`)
	// Hole 0 is a whole circle, which records from the entity's own data and
	// never consults TExact. Hole 1 is cut by a spline, so it is sampled.
	require.False(t, prof.Holes[0][0].Partial)
	require.True(t, prof.Holes[1][0].Partial)
	require.False(t, prof.Holes[1][0].TExact)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.ErrorContains(t, err, `hole 1 edge 0`)
}

func TestRecordProfileGates(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]

	// Foreign: the profile's plane-local coordinates belong to its own sketch.
	other, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	_, _, err = decad.RecordProfile(other, prof)
	require.ErrorIs(t, err, decad.ErrForeignProfile)

	// Nil input is degenerate, not a panic.
	_, _, err = decad.RecordProfile(s, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RecordProfile(nil, prof)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// Stale: move the geometry after the snapshot — the profile still holds
	// the old boundary, and recording it would build the wrong part.
	s.AddConstraint(sketch.NewDistance(rect.A, rect.B, 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, prof.IsStale(), `the solve should have moved the sketch under the profile`)
	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrStaleProfile)

	// A fresh profile records again.
	fresh := s.Profiles()[0]
	_, _, err = decad.RecordProfile(s, fresh)
	require.NoError(t, err)
}

func TestRecordProfileRejectsForeignBoundaryEntity(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]

	foreign, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	foreignRect := foreign.CreateRectangle(100, 100, 110, 110)
	foreign.Fix(foreignRect.A)
	_, err = foreign.Solve(t.Context())
	require.NoError(t, err)

	// The replacement has the same area, so the later area falsifier cannot
	// distinguish it from the profile that came from s.
	prof.Outer = foreign.Profiles()[0].Outer
	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrForeignProfile)
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecordProfileRejectsForeignHoleEntity(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 5), 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof)

	foreign, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := foreign.CreatePoint(100, 100)
	foreign.Fix(center)
	foreign.CreateCircle(center, 2)
	_, err = foreign.Solve(t.Context())
	require.NoError(t, err)
	prof.Holes[0] = foreign.Profiles()[0].Outer

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrForeignProfile)
}

func TestRecordProfileRejectsChangedCurrentBoundary(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	first := s.CreateRectangle(0, 0, 10, 10)
	second := s.CreateRectangle(20, 0, 30, 10)
	s.Fix(first.A)
	s.Fix(second.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	profiles[0].Outer = profiles[1].Outer

	_, _, err = decad.RecordProfile(s, profiles[0])
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}

func TestRecordProfileRejectsTypedNilBoundaryEntity(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	prof := s.Profiles()[0]
	prof.Outer[0].Entity = (*sketch.Line)(nil)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}

// nearMissTriangle builds the triangle (0,0), (10,0), (0,10) whose closing line
// STARTS at (du, 10+dv) instead of at (0, 10) — the corner the previous line
// actually ends at. sketch admits the region on its own proximity threshold, so
// the profile is valid and reads an area of 50 mm2 whatever the offset.
func nearMissTriangle(t *testing.T, du, dv float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	missed := s.CreatePoint(du, 10+dv)
	s.CreateLine(origin, right)
	s.CreateLine(right, top)
	s.CreateLine(missed, origin)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid, `sketch should still admit the near-miss region`)
	return s, profiles[0]
}

func TestRecordProfileRejectsUnclosedLoop(t *testing.T) {
	// A loop whose recorded segments do not meet bounds no region at all, so
	// there is nothing for an Exact, zero-bound area to be exact ABOUT. sketch
	// snapped the gap away and reports one valid region; decad records each
	// entity's own points verbatim, which is where the gap is visible, and
	// refuses (docs/sketch-seam-design.md §3).
	for _, tc := range []struct {
		name   string
		du, dv float64
	}{
		{name: "collinear, 3e-13 mm", dv: 3e-13},
		{name: "collinear, 1e-8 mm", dv: 1e-8},
		{name: "off the line, 1e-6 mm", du: 1e-6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := nearMissTriangle(t, tc.du, tc.dv)
			require.Equal(t, 50.0, p.Area, `sketch reports the snapped region's area`)
			for _, e := range p.Outer {
				require.False(t, e.Partial, `the near miss is snapped, so every edge is whole`)
			}

			_, _, err := decad.RecordProfile(s, p)
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
			require.Contains(t, err.Error(), `does not close`)

			// The same refusal reaches the solid evaluator, which used to
			// publish a 100 mm3 Exact volume over the same open loop.
			_, err = decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
		})
	}
}

func TestRecordProfileRejectsUnclosedConstrainedLoop(t *testing.T) {
	// The same rule, reached the way a caller most likely reaches it: two
	// distinct points driven together by a coincidence constraint. The solver
	// converges to within its own residual, not to the same float, so the
	// recorded loop does not close and no measurement is published over it.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	right := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 10)
	mate := s.CreatePoint(0.001, 10.002)
	s.CreateLine(origin, right)
	s.CreateLine(right, top)
	s.CreateLine(mate, origin)
	s.AddConstraint(sketch.NewCoincident(top, mate))
	s.Fix(origin)
	s.Fix(right)
	solved, err := s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, solved.Converged)
	require.NotEqual(t, [2]float64{top.X(), top.Y()}, [2]float64{mate.X(), mate.Y()},
		`the solver converges within a residual, so the two points stay distinct`)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	_, _, err = decad.RecordProfile(s, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
}

func TestRecordProfileRecordsSnapThresholdTrim(t *testing.T) {
	// The same shape one order of magnitude further out: past sketch's snap
	// threshold it TRIMS the two lines at their crossing instead, so the
	// recorded loop closes on sketch's own cut and the region measures. This is
	// the boundary the refusal above must not overrun.
	s, p := nearMissTriangle(t, 1e-5, 0)
	partial := 0
	for _, e := range p.Outer {
		if e.Partial {
			require.True(t, e.TExact, `an analytic line/line cut is certified`)
			partial++
		}
	}
	require.Equal(t, 2, partial, `the two lines that miss should arrive trimmed`)

	rec, _, err := decad.RecordProfile(s, p)
	require.NoError(t, err, `a loop closed on sketch's own cut records`)
	require.Len(t, rec.Outer.Segments, 3)

	area, err := rec.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 49.99995, value, 1e-7)
	require.Equal(t, decad.Approximate, area.Exactness, `a trimmed section is bounded, never Exact`)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.Less(t, bound, 1e-12)
}

func TestProfileRecordAreaRejectsUnclosedLoop(t *testing.T) {
	// A decoded or caller-built record reaches the same verdict through the
	// moments validator: the reconstruction authenticates each candidate region
	// through RecordProfile, which now refuses an open loop, so no candidate
	// matches and the record is reported as bounding no closed region.
	triangle := func(gap float64) decad.ProfileRecord {
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 10, V: 0}, TEnd: 1},
			decad.LineSeg{Start: decad.Point2{U: 10, V: 0}, End: decad.Point2{U: 0, V: 10}, TEnd: 1},
			decad.LineSeg{Start: decad.Point2{U: 0, V: 10 + gap}, End: decad.Point2{U: 0, V: 0}, TEnd: 1},
		}}}
	}

	_, err := triangle(3e-13).Area()
	require.ErrorIs(t, err, decad.ErrDegenerate)

	area, err := triangle(0).Area()
	require.NoError(t, err, `the closed record still measures`)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 50.0, value)
	require.Equal(t, decad.Exact, area.Exactness)
}
