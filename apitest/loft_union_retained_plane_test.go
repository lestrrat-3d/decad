package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// chordedFrustum lofts a circle of radius r0 at z 0, centred on (cx, 0), to a
// circle of radius r1 at height h. The loft chords the wall: each wall facet
// stands for the cone patch it chords, not for a plane.
func chordedFrustum(t *testing.T, doc *decad.Document, cx, r0, r1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), h)
	require.NoError(t, err)
	circle := func(plane *sketch.Plane, r float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		c := s.CreatePoint(cx, 0)
		s.Fix(c)
		s.CreateCircle(c, r)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	s0, p0 := circle(w.XY(), r0)
	s1, p1 := circle(top, r1)
	body, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return body
}

// frustumRim measures the rim a plane cut x >= cutX leaves on the wall of the
// frustum of chordedFrustum(0, 10, 8, 20): the vertices on the cut plane at
// wall height. It returns the largest claimed vertex bound, and the largest
// distance of such a vertex from the true rim, the cone's own intersection
// with the plane.
func frustumRim(t *testing.T, body *decad.Body, cutX float64) (claimed, actual float64, count int) {
	t.Helper()
	// The true wall radius at height z is 10 - z/10, so the true rim at z has
	// y = sqrt(r(z)^2 - cutX^2).
	rim := func(z float64) (float64, bool) {
		r := 10 - z/10
		if r*r <= cutX*cutX {
			return 0, false
		}
		return math.Sqrt(r*r - cutX*cutX), true
	}
	for _, v := range body.Vertices() {
		p := v.Position()
		if math.Abs(p.Value.X-cutX) > 1e-9 || p.Value.Z < 2.5 || p.Value.Z > 13.5 {
			continue
		}
		// Distance to the rim curve (z, +-y(z)), searched on a fine grid of z.
		best := math.Inf(1)
		for k := 0; k <= 20000; k++ {
			z := 2 + 12*float64(k)/20000
			y, ok := rim(z)
			if !ok {
				continue
			}
			best = math.Min(best, math.Hypot(math.Abs(p.Value.Y)-y, p.Value.Z-z))
		}
		if best > 1 {
			continue
		}
		count++
		claimed = math.Max(claimed, p.Bound.Mag())
		actual = math.Max(actual, best)
	}
	return claimed, actual, count
}

// Unioning chorded lofts must not make a later Cut claim a rim bound smaller
// than the fresh loft's: a chorded wall facet holds the cone patch it chords
// only within the loft's chord bound, so it is no exact plane.
func TestChordedLoftUnionKeepsRimBoundOnLaterCut(t *testing.T) {
	t.Parallel()
	const cutX = 8.8
	doc := decad.New()
	a := chordedFrustum(t, doc, 0, 10, 8, 20)
	b := chordedFrustum(t, doc, 40, 10, 8, 20)
	tool := boxBodyAtZ(t, doc, cutX, -20, 20, 20, 2, 14)
	freshTool, err := decad.Copy(t.Context(), tool)
	require.NoError(t, err)
	freshLoft, err := decad.Copy(t.Context(), a)
	require.NoError(t, err)

	fresh, err := decad.Cut(t.Context(), freshLoft, freshTool)
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), union, tool)
	require.NoError(t, err)

	freshClaimed, freshActual, freshCount := frustumRim(t, fresh, cutX)
	cutClaimed, cutActual, cutCount := frustumRim(t, cut, cutX)
	require.Positive(t, freshCount)
	require.Positive(t, cutCount)
	// Each rim vertex lies within its claimed bound of the true rim.
	require.LessOrEqual(t, freshActual, freshClaimed)
	require.LessOrEqual(t, cutActual, cutClaimed, "the unioned loft's rim vertices stand farther from the true rim than they claim")
	// And the unioned loft's rim states at least the fresh loft's bound.
	require.GreaterOrEqual(t, cutClaimed, freshClaimed)
}
