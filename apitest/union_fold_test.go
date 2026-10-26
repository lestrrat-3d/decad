package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/union-fold-design.md's public-surface tests for the
// many-body Union that no certified route takes as a set: two revolved frusta, 10 mm long
// and overlapping from x = 6 to 10, and a block through their axis.

// foldCone revolves a trapezoid, radius 5 at x0 and 3 ten millimetres on, about the X axis.
func foldCone(t *testing.T, doc *decad.Document, x0 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b, c, d := s.CreatePoint(x0, 0), s.CreatePoint(x0+10, 0), s.CreatePoint(x0+10, 3), s.CreatePoint(x0, 5)
	for _, p := range []*sketch.Point{a, b, c, d} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// foldVolume integrates the cross-section of the union along x: at each x the larger of the
// two frusta's discs, joined, where the block lies there (x 3 to 9, |y| ≤ 2, |z| ≤ 1), by the
// block's rectangle.
func foldVolume() float64 {
	const steps, ysteps = 4000, 4000
	total := 0.0
	for i := range steps {
		x := (float64(i) + 0.5) * 16 / steps
		r := 0.0
		if x <= 10 {
			r = 5 - 0.2*x
		}
		if x >= 6 {
			r = math.Max(r, 5-0.2*(x-6))
		}
		area := math.Pi * r * r
		if x >= 3 && x <= 9 {
			reach := math.Max(r, 2)
			area = 0
			for j := range ysteps {
				y := -reach + (float64(j)+0.5)*2*reach/ysteps
				h := math.Sqrt(math.Max(r*r-y*y, 0))
				if math.Abs(y) <= 2 {
					h = math.Max(h, 1)
				}
				area += 2 * h * 2 * reach / ysteps
			}
		}
		total += area * 16 / steps
	}
	return total
}

func TestUnionOfSeveralBodiesFoldsThemTwoAtATime(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := foldCone(t, doc, 0), foldCone(t, doc, 6)
	c := boxBodyAtZ(t, doc, 3, -2, 9, 2, -1, 2)
	out, err := decad.Union(t.Context(), a, b, c)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == out)
	v, err := out.Volume()
	require.NoError(t, err)
	want := foldVolume()
	require.InDelta(t, want, v.Value.Base(), 0.005*want+v.Bound.Base())
}

func TestUnionFoldIsTheSameWhateverTheOrder(t *testing.T) {
	t.Parallel()
	volume := func(order [3]int) float64 {
		doc := decad.New()
		set := []*decad.Body{foldCone(t, doc, 0), foldCone(t, doc, 6), boxBodyAtZ(t, doc, 3, -2, 9, 2, -1, 2)}
		out, err := decad.Union(t.Context(), set[order[0]], set[order[1]], set[order[2]])
		require.NoError(t, err)
		v, err := out.Volume()
		require.NoError(t, err)
		return v.Value.Base()
	}
	base := volume([3]int{0, 1, 2})
	for _, order := range [][3]int{{2, 0, 1}, {1, 2, 0}} {
		require.InDelta(t, base, volume(order), 0.5)
	}
}

func TestUnionFoldRefusalsLeaveEveryBodyLive(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := foldCone(t, doc, 0), foldCone(t, doc, 6)
	c := boxBodyAtZ(t, doc, 3, -2, 9, 2, -1, 2)
	_, err := decad.Union(t.Context(), a, b, a)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a repeated body")
	require.Len(t, doc.Bodies(), 3)
	_, err = decad.Union(t.Context(), a, b, nil)
	require.Error(t, err)
	require.Len(t, doc.Bodies(), 3, "a refusal leaves the three bodies and no copy")
	require.True(t, c.Kind() == decad.BodySolid)
}
