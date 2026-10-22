package axialsection_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
)

func record(t *testing.T, s *sketch.Sketch, p *sketch.Profile) (momentinput.Profile, float64) {
	t.Helper()
	prof, _, area, err := sectioncert.RecordProfile(s, p)
	require.NoError(t, err)
	return prof, area
}

func rectSection(t *testing.T, x0, y0, x1, y1 float64) (momentinput.Profile, float64) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return record(t, s, s.Profiles()[0])
}

func discSection(t *testing.T, r float64) (momentinput.Profile, float64) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return record(t, s, s.Profiles()[0])
}

// lobeSection draws one region closed by a line, a fit spline, an arc, a fit spline, a
// line and a root arc on the circle of radius r, turned by rot.
func lobeSection(t *testing.T, r, rot, half float64, fit int) (momentinput.Profile, float64) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.Origin()
	pt := func(rad, a float64) *sketch.Point {
		p := s.CreatePoint(rad*math.Cos(a+rot), rad*math.Sin(a+rot))
		s.Fix(p)
		return p
	}
	rr, rl := pt(r, -half), pt(r, half)
	right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
	for i := range fit {
		f := float64(i) / float64(fit-1)
		rad := r + 0.35 + f*0.65
		a := half * (1 - 0.4*f)
		right[i], left[i] = pt(rad, -a), pt(rad, a)
	}
	s.CreateLine(rr, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(c, right[fit-1], left[fit-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rl)
	s.CreateArc(c, rr, rl)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return record(t, s, s.Profiles()[0])
}

// TestUnionOfAPlateAndARibIsTheirOutline joins two rectangles, whose section has no
// free-form segment, so only the 2D rule is at work.
func TestUnionOfAPlateAndARibIsTheirOutline(t *testing.T) {
	t.Parallel()
	plate, pa := rectSection(t, 0, 0, 10, 4)
	rib, ra := rectSection(t, 4, 2, 6, 8)
	u, area, ok, err := axialsection.UnionArea(t.Context(), []momentinput.Profile{plate, rib})
	require.NoError(t, err)
	require.True(t, ok)
	overlap := 2.0 * 2.0 // the rib's 2 mm width over the plate's upper 2 mm
	require.InDelta(t, pa+ra-overlap, area, 1e-9)
	require.Len(t, u.Outer.Segments, 8, "the plate's bottom and sides, the rib's top and sides, and the two split top edges")
	require.Empty(t, u.Holes)
}

func TestUnionOfAnLAndATIsOneRegion(t *testing.T) {
	t.Parallel()
	a, aa := rectSection(t, 0, 0, 6, 2)
	b, ba := rectSection(t, 0, 1, 2, 6)
	c, ca := rectSection(t, 4, 1, 6, 6)
	_, area, ok, err := axialsection.UnionArea(t.Context(), []momentinput.Profile{a, b, c})
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, aa+ba+ca-2*2*1, area, 1e-9, "b and c each overlap a in a 2 by 1 patch")
}

// TestUnionOfFlushRectanglesIsAMiss pins a case sketch does not certify: two
// rectangles whose edges lie on one line over only part of their length, so the
// union would have to split a line.
func TestUnionOfFlushRectanglesIsAMiss(t *testing.T) {
	t.Parallel()
	plate, _ := rectSection(t, 0, 0, 10, 4)
	rib, _ := rectSection(t, 4, 4, 6, 8)
	_, ok, err := axialsection.Union(t.Context(), []momentinput.Profile{plate, rib})
	require.NoError(t, err)
	require.False(t, ok)
}

func TestUnionOfADiscAndLobesOnItsCircle(t *testing.T) {
	t.Parallel()
	for _, n := range []int{8, 12, 17} {
		const r = 3.0
		disc, da := discSection(t, r)
		sections := []momentinput.Profile{disc}
		sum := da
		for k := range n {
			lobe, la := lobeSection(t, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), 4)
			sections = append(sections, lobe)
			sum += la
		}
		u, area, ok, err := axialsection.UnionArea(t.Context(), sections)
		require.NoError(t, err, "n=%d", n)
		require.True(t, ok)
		require.InDelta(t, sum, area, 1e-9*sum, "n=%d", n)
		require.Len(t, u.Outer.Segments, 6*n, "five whole edges and one circle fragment per lobe")
	}
}

func TestUnionMissesWhatItCannotRead(t *testing.T) {
	t.Parallel()
	a, _ := rectSection(t, 0, 0, 4, 4)
	b, _ := rectSection(t, 2, 2, 6, 6)
	t.Run("one section", func(t *testing.T) {
		t.Parallel()
		_, ok, err := axialsection.Union(t.Context(), []momentinput.Profile{a})
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a section with a hole", func(t *testing.T) {
		t.Parallel()
		holed := a
		holed.Holes = append(holed.Holes, b.Outer)
		_, ok, err := axialsection.Union(t.Context(), []momentinput.Profile{holed, b})
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a trimmed segment", func(t *testing.T) {
		t.Parallel()
		// The union of a disc and a lobe has a circle fragment, which is a trimmed
		// segment of the circle: a second union over it is a miss.
		disc, _ := discSection(t, 3)
		lobe, _ := lobeSection(t, 3, 0, 0.1, 4)
		joined, ok, err := axialsection.Union(t.Context(), []momentinput.Profile{disc, lobe})
		require.NoError(t, err)
		require.True(t, ok)
		other, _ := lobeSection(t, 3, 2, 0.1, 4)
		_, ok, err = axialsection.Union(t.Context(), []momentinput.Profile{joined, other})
		require.NoError(t, err)
		require.False(t, ok)
	})
}
