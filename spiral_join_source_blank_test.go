package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func spiralTestRevolveBlank(t *testing.T, section []planePoint) *Body {
	t.Helper()
	doc := New()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(section))
	for i, point := range section {
		points[i] = s.CreatePoint(point.U, point.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	blank, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return blank
}

func TestSpiralBlankUsesAuthenticatedRevolveSection(t *testing.T) {
	section := []planePoint{{U: 4.9}, {U: 6}, {U: 6, V: 3.5},
		{U: 5.7, V: 2.28}, {U: 4.7, V: 1.88}, {U: 4.9, V: 1.5}}
	blank := spiralTestRevolveBlank(t, section)
	candidates := revolveRootCandidates(blank)
	require.NotEmpty(t, candidates)
	source, ok := exactSpiralBlank(blank)
	require.True(t, ok)
	require.Same(t, blank, source.section.blank)
	rootCandidates := 0
	for _, candidate := range candidates {
		if candidate.rootStart != section[4] || candidate.rootEnd != section[3] {
			continue
		}
		rootCandidates++
		require.Equal(t, candidate.rootSpan, source.section.rootSpan)
		require.Same(t, candidate.spans[candidate.rootSpan].face, source.rootFace)
		require.Equal(t, candidate.coneGap, source.section.coneGap)
	}
	require.Equal(t, 1, rootCandidates)
	require.Equal(t, section[4], source.toeRoot)
	require.Equal(t, section[3], source.ded)
	for edge, index := range source.edgeIndex {
		found := false
		for _, span := range source.section.spans {
			if len(span.sourceEdges) == 1 && span.sourceEdges[0] == index {
				found = true
				break
			}
		}
		require.True(t, found, "legacy edge %d is not an authenticated source edge", edge)
	}
}

func TestSpiralBlankRefusesSevenSideRevolveSection(t *testing.T) {
	section := []planePoint{{U: 4.9}, {U: 6}, {U: 6, V: 3.5},
		{U: 5.9, V: 3.4}, {U: 5.7, V: 2.28},
		{U: 4.7, V: 1.88}, {U: 4.9, V: 1.5}}
	blank := spiralTestRevolveBlank(t, section)
	candidates := revolveRootCandidates(blank)
	require.NotEmpty(t, candidates)
	for _, candidate := range candidates {
		require.Len(t, candidate.spans, 7)
	}
	_, admitted := exactSpiralBlank(blank)
	require.False(t, admitted)
}
