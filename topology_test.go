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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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

// TestNURBSSurfaceReportsKindNURBS pins docs/spline-design.md §7: a
// NURBSSurface is a tagged, opaque Surface variant reporting the existing
// KindNURBS discriminant, carrying no exported geometry of its own.
func TestNURBSSurfaceReportsKindNURBS(t *testing.T) {
	t.Parallel()
	var s decad.Surface = decad.NURBSSurface{}
	require.Equal(t, decad.KindNURBS, s.Kind())
}

// TestNURBSCurveSealsIntoCurve pins docs/spline-design.md §7: a NURBSCurve is
// NURBSSurface's one-dimensional analog. Curve is sealed by its marker method
// alone and declares no Kind, so assigning a NURBSCurve to a Curve variable
// is the whole of what the variant publishes.
func TestNURBSCurveSealsIntoCurve(t *testing.T) {
	t.Parallel()
	var c decad.Curve = decad.NURBSCurve{}
	require.NotNil(t, c)
}
