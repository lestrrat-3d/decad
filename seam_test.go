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
	// Two overlapping circles: a curve/curve crossing has no closed form, so
	// sketch reports the cut sampled (TExact false) and decad refuses to
	// record the fragment — never approximately, never as the whole curve.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c1 := s.CreatePoint(0, 0)
	s.Fix(c1)
	s.CreateCircle(c1, 20)
	c2 := s.CreatePoint(25, 0)
	s.CreateCircle(c2, 20)
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
	require.NotZero(t, rejected, `regions bounded by sampled circle/circle cuts should be rejected`)
}

func TestRecordProfileReportsOuterEdgeIndex(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(-8, 0), 20)
	s.CreateCircle(s.CreatePoint(8, 0), 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Outer) == 5 && !candidate.Outer[0].Partial {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-and-overlapping-circles region should exist`)
	require.True(t, prof.Outer[2].Partial)
	require.False(t, prof.Outer[2].TExact)

	_, _, err = decad.RecordProfile(s, prof)
	require.ErrorIs(t, err, decad.ErrUnrecordableProfile)
	require.ErrorContains(t, err, `outer edge 2`)
}

func TestRecordProfileReportsHoleAndEdgeIndices(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-80, -50, 80, 50)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(-50, 0), 8)
	s.CreateCircle(s.CreatePoint(10, 0), 20)
	s.CreateCircle(s.CreatePoint(26, 0), 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Outer) == 4 && len(candidate.Holes) == 2 {
			prof = candidate
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-two-holes region should exist`)
	require.False(t, prof.Holes[0][0].Partial)
	require.True(t, prof.Holes[0][0].TExact)
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
