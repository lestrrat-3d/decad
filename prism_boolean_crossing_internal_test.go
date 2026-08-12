package decad

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §14 PR3's own required property
// test (§15): the crossing sub-case's analytic answer measured against the
// mesh path's own answer, on the same randomized pairs. It runs internally so
// the mesh path can be forced directly through evaluateBoolean, rather than
// raced against the analytic dispatch performBoolean already prefers.

// crossingDiscBody extrudes one coplanar disc of radius r centered at
// (cx, 0), swept by e. Two calls with different z-extents are this file's
// own fixture for a genuinely crossing (not nested) coplanar pair whose caps
// stay clear of each other — see TestPrismCrossingDiscPairsMatchMeshAnswer's
// own doc comment for why that clearance matters.
func crossingDiscBody(t *testing.T, doc *Document, cx, r float64, e Extent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], e)
	require.NoError(t, err)
	return body
}

// crossingDiscPairFixture is one randomized trial's own parameters, drawn
// once and replayed identically against both the mesh path and the analytic
// path (and against both Cut and Intersect), so the two answers are about
// the exact same geometry.
type crossingDiscPairFixture struct {
	r1, r2, offset, targetH, toolHalf float64
}

// randomCrossingDiscPairFixtures draws n trials from a fixed seed: two radii
// in [5, 20] mm and a center offset in (|r1-r2|, r1+r2), margined by 0.5 mm on
// each side to stay clear of a near-tangent or near-nested crossing, so every
// drawn trial is a genuine two-point crossing this classifier's own scope
// covers. targetH/toolHalf follow this file's own cap-clearance shape.
func randomCrossingDiscPairFixtures(n int) []crossingDiscPairFixture {
	r := rand.New(rand.NewSource(1))
	fixtures := make([]crossingDiscPairFixture, 0, n)
	for len(fixtures) < n {
		r1 := 5 + r.Float64()*15
		r2 := 5 + r.Float64()*15
		lo := math.Abs(r1-r2) + 0.5
		hi := r1 + r2 - 0.5
		if hi <= lo {
			continue
		}
		offset := lo + r.Float64()*(hi-lo)
		targetH := 5 + r.Float64()*10
		toolHalf := targetH*1.5 + r.Float64()*5
		fixtures = append(fixtures, crossingDiscPairFixture{r1: r1, r2: r2, offset: offset, targetH: targetH, toolHalf: toolHalf})
	}
	return fixtures
}

// TestPrismCrossingDiscPairsMatchMeshAnswer is §14 PR3's own required
// property test: over a randomized set of crossing (not coincident-carrier)
// coplanar prism pairs, Cut and Intersect's analytic answer agrees with the
// mesh path's own answer within the mesh path's bound, and the analytic
// bound is the tighter of the two.
//
// Each pair's target sweeps [0, targetH] and its tool sweeps
// [-toolHalf, +toolHalf] with toolHalf > targetH, so the tool's own caps
// clear the target's on both sides — this pair's ONLY coplanar contact is the
// footprints' own genuine crossing, never a coincident cap. That is what lets
// evaluateBoolean below answer an ordinary chorded intersection instead of
// refusing the pair outright the way a coincident-cap pair would (§1's own
// "coplanar contact refuses outright" consequence, still standing for any
// pair outside this design's admitted class).
func TestPrismCrossingDiscPairsMatchMeshAnswer(t *testing.T) {
	for i, fx := range randomCrossingDiscPairFixtures(8) {
		for _, op := range []OpKind{OpIntersect, OpCut} {
			doc := New()
			target := crossingDiscBody(t, doc, 0, fx.r1, Distance{D: units.Millimeters(fx.targetH), Dir: Along})
			tool := crossingDiscBody(t, doc, fx.offset, fx.r2, Symmetric{D: units.Millimeters(fx.toolHalf)})
			eval, err := evaluateBoolean(t.Context(), op, target, tool)
			require.NoErrorf(t, err, "trial %d op %v: mesh path", i, op)
			meshVol := eval.volume

			doc2 := New()
			target2 := crossingDiscBody(t, doc2, 0, fx.r1, Distance{D: units.Millimeters(fx.targetH), Dir: Along})
			tool2 := crossingDiscBody(t, doc2, fx.offset, fx.r2, Symmetric{D: units.Millimeters(fx.toolHalf)})
			pp, ok, err := tryPrismBoolean(t.Context(), op, target2, tool2)
			require.NoErrorf(t, err, "trial %d op %v: analytic path", i, op)
			require.Truef(t, ok, "trial %d op %v: the crossing classifier must admit this pair", i, op)
			analyticBody, err := evalPrismContext(t.Context(), doc2, doc2.nextStepRef(), pp, newFreeformWork())
			require.NoError(t, err)
			analyticVol, err := analyticBody.Volume()
			require.NoError(t, err)

			diff := math.Abs(analyticVol.Value.Base() - meshVol.Value.Base())
			require.LessOrEqualf(t, diff, meshVol.Bound.Base(),
				"trial %d op %v: analytic %v and mesh %v (+-%v) disagree beyond the mesh bound",
				i, op, analyticVol.Value.Base(), meshVol.Value.Base(), meshVol.Bound.Base())
			require.Lessf(t, analyticVol.Bound.Base(), meshVol.Bound.Base(),
				"trial %d op %v: the analytic bound %v must be tighter than the mesh bound %v",
				i, op, analyticVol.Bound.Base(), meshVol.Bound.Base())
		}
	}
}
