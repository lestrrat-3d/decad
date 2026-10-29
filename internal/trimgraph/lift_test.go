package trimgraph_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/trimgraph"
	"github.com/lestrrat-3d/decad/internal/trimtest"
	"github.com/stretchr/testify/require"
)

func TestLiftBevelledPlate(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	g, err := trimgraph.Lift(t.Context(), b, fp)
	require.NoError(t, err)
	require.Len(t, g.Faces, 7)
	require.Len(t, g.Edges, 15)
	require.Len(t, g.Vertices, 10)
	for i, e := range g.Edges {
		require.True(t, e.Convex, "edge %d", i)
		require.NotEqual(t, e.Faces[0], e.Faces[1])
		require.NotEqual(t, e.Start, e.End)
	}
	nodes := topobuild.Faces(b)
	for i, f := range g.Faces {
		require.Same(t, nodes[i], f.Node)
		require.Len(t, f.Loops, 1)
		// The group is the one fp.Src names for the face's facets.
		for k, fi := range fp.FaceOf {
			if fi == i {
				require.Equal(t, fp.Src[k], f.Group)
			}
		}
	}
	for i, v := range g.Vertices {
		require.Len(t, v.Edges, 3, "vertex %d", i)
		require.Len(t, v.Faces, 3, "vertex %d", i)
		// Each face of the cyclic order shares an edge at v with the next.
		for k, f := range v.Faces {
			next := v.Faces[(k+1)%3]
			shared := 0
			for _, ei := range v.Edges {
				e := g.Edges[ei]
				if (e.Faces[0] == f && e.Faces[1] == next) || (e.Faces[1] == f && e.Faces[0] == next) {
					shared++
				}
			}
			require.Equal(t, 1, shared, "vertex %d faces %d and %d", i, f, next)
		}
	}
	// The walk of every outer loop is a closed chain of its own edges.
	for i, f := range g.Faces {
		for _, u := range f.Loops[0] {
			e := g.Edges[u.Edge]
			require.True(t, e.Faces[0] == i || e.Faces[1] == i)
			require.Equal(t, u.Forward, e.Faces[0] == i, "Faces[0] walks the edge forward")
		}
	}
}

func TestLiftConcaveEdgeIsFlagged(t *testing.T) {
	b, fp := trimtest.LSection(t)
	g, err := trimgraph.Lift(t.Context(), b, fp)
	require.NoError(t, err)
	concave := 0
	for _, e := range g.Edges {
		if !e.Convex {
			concave++
		}
	}
	require.Equal(t, 1, concave)
}

func TestLiftDeclines(t *testing.T) {
	tests := []struct {
		name string
		edit func(*facetedrecord.Payload)
		code string
	}{
		{"a stated normal bound", func(fp *facetedrecord.Payload) { fp.Groups[0].NormalBound = 1e-9 }, "(L2)"},
		{"a stated axial delta", func(fp *facetedrecord.Payload) {
			fp.Groups[0].HasAxialDelta, fp.Groups[0].AxialDelta = true, 1e-9
		}, "(L2)"},
		{"a flipped Reversed", func(fp *facetedrecord.Payload) { fp.Groups[6].Reversed = true }, "(L6)"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			_, fp := trimtest.BevelledPlate(t)
			fp.Groups = append([]facetedrecord.Group(nil), fp.Groups...)
			tc.edit(&fp)
			b, built := trimtest.BuildPayload(t, fp)
			_, err := trimgraph.Lift(t.Context(), b, built)
			require.ErrorIs(t, err, trimgraph.ErrNotLiftable)
			require.ErrorContains(t, err, tc.code)
		})
	}
}

func TestLiftDeclinesValenceFour(t *testing.T) {
	b, fp := trimtest.Pyramid(t)
	_, err := trimgraph.Lift(t.Context(), b, fp)
	require.ErrorIs(t, err, trimgraph.ErrNotLiftable)
	require.ErrorContains(t, err, "(L5)")
}

func TestLiftHonoursCancellation(t *testing.T) {
	b, fp := trimtest.BevelledPlate(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := trimgraph.Lift(ctx, b, fp)
	require.ErrorIs(t, err, context.Canceled)
	require.NotErrorIs(t, err, trimgraph.ErrNotLiftable)
}
