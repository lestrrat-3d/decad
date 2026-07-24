package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func coedgeBox(t *testing.T) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestLoopCoEdgesMatchEdgesCompatibilityView(t *testing.T) {
	body := coedgeBox(t)

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			undirected := loop.Edges()
			require.Len(t, directed, len(undirected))
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, undirected[i], use.Edge())
			}
		}
	}
}

func TestLoopCoEdgesFollowDirectedBoundary(t *testing.T) {
	body := coedgeBox(t)
	uses := make(map[*decad.Edge][]decad.CoEdge)
	sawReverse := false

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, use.End(), directed[(i+1)%len(directed)].Start())
				if use.IsForward() {
					require.Same(t, use.Edge().Start(), use.Start())
					require.Same(t, use.Edge().End(), use.End())
				} else {
					sawReverse = true
					require.Same(t, use.Edge().End(), use.Start())
					require.Same(t, use.Edge().Start(), use.End())
				}
				uses[use.Edge()] = append(uses[use.Edge()], use)
			}
		}
	}

	require.True(t, sawReverse, `the box exposes edge uses that oppose global edge orientation`)
	require.Len(t, uses, len(body.Edges()))
	for _, edge := range body.Edges() {
		require.Len(t, uses[edge], 2, `each manifold edge has two loop uses`)
	}
}

func TestLoopCoEdgesReturnImmutableView(t *testing.T) {
	loop := coedgeBox(t).Faces()[0].Loops()[0]
	directed := loop.CoEdges()
	undirected := loop.Edges()
	require.NotEmpty(t, directed)

	wantEdge := directed[0].Edge()
	wantStart := directed[0].Start()
	wantEnd := directed[0].End()
	wantForward := directed[0].IsForward()
	directed[0] = decad.CoEdge{}
	undirected[0] = nil

	got := loop.CoEdges()[0]
	require.Same(t, wantEdge, got.Edge())
	require.Same(t, wantStart, got.Start())
	require.Same(t, wantEnd, got.End())
	require.Equal(t, wantForward, got.IsForward())
	require.Same(t, wantEdge, loop.Edges()[0])
}
