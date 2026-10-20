package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/stretchr/testify/require"
)

// TestBlendEdgesGates chamfers box edges through route E and breaks one fact
// of the selection at a time. Each broken fact is refused with the Table SB
// row that owns it.
func TestBlendEdgesGates(t *testing.T) {
	t.Parallel()

	run := func(t *testing.T, pick func(edges []*topo.Edge, body *topo.Body) []*topo.Edge) (*topo.Body, error) {
		t.Helper()
		body, bp, boxEdges := allEdgeChamferFixture(t)
		req := brepmodify.Request{
			Op: "chamfers", Sel: "test selector", Edges: pick(boxEdges, body), CurveName: publicCurveName,
			Blend: &brepmodify.BlendOp{Kind: "chamfer", Corner: func(loop cornerLoop, _, ci int, _ *topo.Edge) (*cornerBlend, error) {
				return computeChamfer(loop, ci, 1, 1)
			}},
		}
		return brepmodify.BlendEdges(t.Context(), envOf(body.Doc, body.Doc.NextProducer), bp, req)
	}
	first := func(edges []*topo.Edge, _ *topo.Body) []*topo.Edge { return edges[:1] }

	t.Run("one straight edge", func(t *testing.T) {
		t.Parallel()
		got, err := run(t, first)
		require.NoError(t, err)
		require.Greater(t, len(topobuild.Faces(got)), 7)
		require.Positive(t, got.Volume.Value.Base())
	})

	refusals := []struct {
		name string
		want string
		pick func(edges []*topo.Edge, body *topo.Body) []*topo.Edge
	}{
		{"circle edge", "SB4", func(_ []*topo.Edge, body *topo.Body) []*topo.Edge {
			for _, f := range topobuild.Faces(body) {
				if _, ok := f.Surface.(surfacegeom.Cylinder); ok {
					return []*topo.Edge{f.Loops[0].CoEdges[0].Edge}
				}
			}
			return nil
		}},
		{"edge that is no line", "SB4", func(edges []*topo.Edge, _ *topo.Body) []*topo.Edge {
			odd := *edges[0]
			odd.Curve = surfacegeom.Ellipse3{}
			return []*topo.Edge{&odd}
		}},
		{"edge with no vertices", "SB4", func(edges []*topo.Edge, _ *topo.Body) []*topo.Edge {
			odd := *edges[0]
			odd.Start = nil
			return []*topo.Edge{&odd}
		}},
		{"edge of no record edge", "SB6", func(edges []*topo.Edge, _ *topo.Body) []*topo.Edge {
			moved := *edges[0]
			start, end := *moved.Start, *moved.End
			start.Position.X += 1
			end.Position.X += 1
			moved.Start, moved.End = &start, &end
			return []*topo.Edge{&moved}
		}},
		{"two edges at one vertex", "SB5", func(edges []*topo.Edge, _ *topo.Body) []*topo.Edge {
			for _, other := range edges[1:] {
				if other.Start == edges[0].Start || other.Start == edges[0].End ||
					other.End == edges[0].Start || other.End == edges[0].End {
					return []*topo.Edge{edges[0], other}
				}
			}
			return nil
		}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := run(t, tc.pick)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
