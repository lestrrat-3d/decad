package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
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
	require.True(t, area.Equal(units.SquareMillimeters(6000), 1e-9), `a 100×60 plate is 6000 mm², got %s`, area)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 60.0, c.U, 1e-9)
	require.InDelta(t, 50.0, c.V, 1e-9)
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
	got, err := area.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, got, 1e-9, `net area subtracts the hole exactly`)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, (6000*50-hole*70)/wantArea, c.U, 1e-9, `the centroid shifts away from the off-center hole`)
	require.InDelta(t, 30.0, c.V, 1e-9)
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
	got, err := area.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*49, got, 1e-9)

	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.0, c.U, 1e-9)
	require.InDelta(t, -3.0, c.V, 1e-9)
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
		got, err := area.In(units.SquareMillimeter)
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

	// A zero-value record has zero net area, so it has no centroid.
	var empty decad.ProfileRecord
	area, err := empty.Area()
	require.NoError(t, err)
	require.Zero(t, area.Mag())
	_, err = empty.Centroid()
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
	require.True(t, area.Equal(units.SquareMillimeters(16), 1e-12), `pointer segments integrate like values, got %s`, area)

	bad := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{(*decad.LineSeg)(nil)}}}
	_, err = bad.Area()
	require.Error(t, err, `a nil segment pointer names no curve to integrate`)
}
