package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternEmbeddedJoinKeepsAxialRootsAndCapLoops(t *testing.T) {
	t.Parallel()
	const (
		teeth = 24
		root  = 5.2
		tip   = 6.2
	)
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.Origin()
	const samples = 5
	right, left := make([]*sketch.Point, samples), make([]*sketch.Point, samples)
	for i := range samples {
		fraction := float64(i) / float64(samples-1)
		radius := root + fraction*(tip-root)
		half := 0.08 - fraction*0.04
		right[i] = s.CreatePoint(radius*math.Cos(-half), radius*math.Sin(-half))
		left[i] = s.CreatePoint(radius*math.Cos(half), radius*math.Sin(half))
		s.Fix(right[i])
		s.Fix(left[i])
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[samples-1], left[samples-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateArc(center, right[0], left[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	profile := s.Profiles()[0]
	require.True(t, profile.Valid)
	tooth, err := decad.Extrude(t.Context(), doc, s, profile,
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	disc := circleBodyAtZ(t, doc, 0, 5.375, 0, 2)
	copies, err := decad.PatternCopies(t.Context(), tooth,
		decad.CircularPattern{Axis: r3.Vec{Z: 1}, Count: teeth})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{tooth}, copies...)...)
	require.NoError(t, err)
	require.False(t, tookMeshPath(joined), "the freeform axial-section join must run before the generic prism fold")

	axial, err := decad.Edges(decad.ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var roots int
	for _, edge := range axial {
		if !edge.IsConvex() {
			roots++
		}
	}
	require.Equal(t, 2*teeth, roots)

	concave, err := decad.Edges(decad.ParallelTo(r3.Vec{Z: 1}), decad.Concave()).Exactly(2 * teeth).SelectEdges(joined)
	require.NoError(t, err)
	require.Len(t, concave, 2*teeth)
	for _, capRef := range []decad.FeatureRef{decad.CapStart(joined), decad.CapEnd(joined)} {
		outer, err := decad.Edges(decad.OuterLoopOf(capRef)).SelectEdges(joined)
		require.NoError(t, err)
		require.NotEmpty(t, outer)
	}
}
