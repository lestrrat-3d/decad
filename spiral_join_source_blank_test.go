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
	require.Same(t, blank, bodyHandle(source.Section.Blank))
	rootCandidates := 0
	for _, candidate := range candidates {
		if candidate.RootStart != section[4] || candidate.RootEnd != section[3] {
			continue
		}
		rootCandidates++
		require.Equal(t, candidate.RootSpan, source.Section.RootSpan)
		require.Same(t, candidate.Spans[candidate.RootSpan].Face, source.RootFace)
		require.Equal(t, candidate.ConeGap, source.Section.ConeGap)
	}
	require.Equal(t, 1, rootCandidates)
	require.Equal(t, section[4], source.ToeRoot)
	require.Equal(t, section[3], source.Ded)
	for edge, index := range source.EdgeIndex {
		found := false
		for _, span := range source.Section.Spans {
			if len(span.SourceEdges) == 1 && span.SourceEdges[0] == index {
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
		require.Len(t, candidate.Spans, 7)
	}
	_, admitted := exactSpiralBlank(blank)
	require.False(t, admitted)
}
