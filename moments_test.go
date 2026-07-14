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

	// A zero-value record has zero net area, so it has no centroid.
	var empty decad.ProfileRecord
	area, err := empty.Area()
	require.NoError(t, err)
	require.Zero(t, area.Value.Mag())
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
	require.True(t, area.Value.Equal(units.SquareMillimeters(16), 1e-12), `pointer segments integrate like values, got %s`, area.Value)

	bad := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{(*decad.LineSeg)(nil)}}}
	_, err = bad.Area()
	require.Error(t, err, `a nil segment pointer names no curve to integrate`)
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
