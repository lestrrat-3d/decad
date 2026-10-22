package apitest_test

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// crossingLoft draws a lobe whose flanks are fit splines that start inside the circle of
// radius r and cross it on their way out, closed by an arc inside the circle and an arc at
// the tip, and lofts it between two identical planes.
func crossingLoftAt(t *testing.T, doc *decad.Document, r, half, height, rot float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pt := func(rad, a float64) *sketch.Point {
			p := s.CreatePoint(rad*math.Cos(a+rot), rad*math.Sin(a+rot))
			s.Fix(p)
			return p
		}
		const fit = 6
		right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
		for i := range fit {
			f := float64(i) / float64(fit-1)
			rad, a := r-0.5+1.9*f, half*(1-0.4*f)
			right[i], left[i] = pt(rad, -a), pt(rad, a)
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateArc(s.Origin(), right[0], left[0])
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(w.XY())
	s1, p1 := section(splitTopPlane(t, w, height))
	b, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return b
}

func crossingLoft(t *testing.T, doc *decad.Document, r, half, height float64) *decad.Body {
	t.Helper()
	return crossingLoftAt(t, doc, r, half, height, 0)
}

// TestPatternJoinFlanksCrossTheBase joins patterned lobes whose spline flanks cross the
// base circle, so the union cuts splines against a circle. The volume is the disc and the
// lobes less the part of each lobe inside the disc, which the same lobes drawn at their
// own angles also have.
func TestPatternJoinFlanksCrossTheBase(t *testing.T) {
	t.Parallel()
	const r, height = 3.0, 2.0
	for _, n := range []int{8, 12, 24} {
		t.Run(strconv.Itoa(n)+" copies", func(t *testing.T) {
			t.Parallel()
			half := 0.4 * math.Pi / float64(n)
			doc := decad.New()
			disc := circleBodyAtZ(t, doc, 0, r, 0, height)
			joined, err := decad.PatternJoin(t.Context(), disc, crossingLoft(t, doc, r, half, height),
				decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
			require.NoError(t, err)
			require.True(t, joined.IsSolid())

			// The same lobes, each drawn at its own angle.
			ref := decad.New()
			rdisc := circleBodyAtZ(t, ref, 0, r, 0, height)
			lobes := make([]*decad.Body, n)
			for k := range lobes {
				lobes[k] = crossingLoftAt(t, ref, r, half, height, 2*math.Pi*float64(k)/float64(n))
			}
			drawn, err := decad.Union(t.Context(), rdisc, lobes...)
			require.NoError(t, err)
			gv, err := joined.Volume()
			require.NoError(t, err)
			wv, err := drawn.Volume()
			require.NoError(t, err)
			require.InDelta(t, wv.Value.Base(), gv.Value.Base(), wv.Bound.Base()+gv.Bound.Base())
		})
	}
}

// TestPatternJoinFlanksCrossTheBaseThenFilletBoreChamfer takes the joined part through the
// finishing order: a concave fillet where each flank meets the base circle, a through-bore
// Cut and a cap Chamfer.
func TestPatternJoinFlanksCrossTheBaseThenFilletBoreChamfer(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 12
	half := 0.4 * math.Pi / n
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := crossingLoft(t, doc, r, half, height)
	copies, err := decad.PatternCopies(t.Context(), lobe, decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: n})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{lobe}, copies...)...)
	require.NoError(t, err)
	v0, err := joined.Volume()
	require.NoError(t, err)

	// Where a flank leaves the base circle the surface turns inward: a concave axial edge.
	selection := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave()).Exactly(2 * n)
	filleted, err := decad.Fillet(t.Context(), joined, selection, units.Millimeters(0.05))
	require.NoError(t, err)
	v1, err := filleted.Volume()
	require.NoError(t, err)
	require.Greater(t, v1.Value.Base(), v0.Value.Base(), "a concave fillet adds material")

	bore := circleBodyAtZ(t, doc, 0, 0.8, -1, height+2)
	bored, err := decad.Cut(t.Context(), filleted, bore)
	require.NoError(t, err)
	caps := decad.Edges(decad.OuterLoopOf(decad.CapStart(bored))).Or(decad.OuterLoopOf(decad.CapEnd(bored)))
	chamfered, err := decad.Chamfer(t.Context(), bored, caps, units.Millimeters(0.02))
	require.NoError(t, err)
	require.True(t, chamfered.IsSolid())
}
