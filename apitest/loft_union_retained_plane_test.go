package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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

// frustumRim measures the rim a plane cut x >= cutX leaves on the wall of
// chordedFrustum(cx, 10, 8, 20) standing on height z0: the vertices on the cut
// plane at wall height. It returns the largest claimed vertex bound and the
// largest distance of such a vertex from the true rim, the cone's own
// intersection with the plane.
func frustumRim(t *testing.T, body *decad.Body, cx, z0, cutX float64) (claimed, actual float64, count int) {
	t.Helper()
	// The true wall radius at height z is 10 - (z-z0)/10, so the true rim at z
	// has y = sqrt(r(z)^2 - (cutX-cx)^2).
	rim := func(z float64) (float64, bool) {
		r := 10 - (z-z0)/10
		d := cutX - cx
		if r*r <= d*d {
			return 0, false
		}
		return math.Sqrt(r*r - d*d), true
	}
	for _, v := range body.Vertices() {
		p := v.Position()
		if math.Abs(p.Value.X-cutX) > 1e-9 || p.Value.Z < z0+2.5 || p.Value.Z > z0+13.5 {
			continue
		}
		// Distance to the rim curve (z, +-y(z)), searched on a fine grid of z.
		best := math.Inf(1)
		for k := range 20001 {
			z := z0 + 2 + 12*float64(k)/20000
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

// A chorded wall facet holds the cone patch it chords only within the loft's
// chord bound, so it is no exact plane: a Cut of a Union of chorded lofts must
// state a rim bound that covers the vertices' real distance from the true rim,
// and no smaller a bound than the fresh loft's own Cut.
func TestChordedLoftUnionKeepsRimBoundOnLaterCut(t *testing.T) {
	t.Parallel()
	const cutX = 8.8
	// Every case builds its own bodies: a Union and a Cut retire their operands.
	build := func(t *testing.T, unions int, placedBy float64) (*decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		target := chordedFrustum(t, doc, 0, 10, 8, 20)
		for k := 1; k <= unions; k++ {
			var err error
			target, err = decad.Union(target, chordedFrustum(t, doc, 40*float64(k), 10, 8, 20)).Do(t.Context())
			require.NoError(t, err)
		}
		if placedBy != 0 {
			move, err := r3.Translation(r3.Vec{Z: placedBy})
			require.NoError(t, err)
			target, err = decad.Copy(t.Context(), target, decad.WithCopyPlacement(move))
			require.NoError(t, err)
		}
		return target, boxBodyAtZ(t, doc, cutX, -20, 20, 20, 2+placedBy, 14)
	}

	target, tool := build(t, 0, 0)
	fresh, err := decad.Cut(target, tool).Do(t.Context())
	require.NoError(t, err)
	freshClaimed, freshActual, freshCount := frustumRim(t, fresh, 0, 0, cutX)
	require.Positive(t, freshCount)
	require.LessOrEqual(t, freshActual, freshClaimed)

	for _, tc := range []struct {
		name     string
		unions   int
		placedBy float64
		// refusable admits the evaluator's refusal as the sound outcome: the
		// chorded wall's facet bound exceeds this pair's chord tolerance.
		refusable bool
	}{
		{"one union", 1, 0, false},
		// The next union reads the chorded walls from a retained result.
		{"union of a union", 2, 0, true},
		// A placed result keeps stating the chord bound.
		{"placed union", 2, 5, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			target, tool := build(t, tc.unions, tc.placedBy)
			cut, err := decad.Cut(target, tool).Do(t.Context())
			if tc.refusable && err != nil {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				return
			}
			require.NoError(t, err)
			claimed, actual, count := frustumRim(t, cut, 0, tc.placedBy, cutX)
			require.Positive(t, count)
			require.LessOrEqual(t, actual, claimed,
				"the rim vertices stand farther from the true rim than they claim")
			require.GreaterOrEqual(t, claimed, freshClaimed)
		})
	}
}
