package meshbool

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCutFacetBatchesPreserveGeometryAndOrder(t *testing.T) {
	identity := func(p r3.Vec) r3.Vec { return p }
	refine := func(m *BoolMesh) *BoolMesh {
		verts := append([]r3.Vec(nil), m.Verts...)
		tris, sources := m.Tris, m.Src
		for range 2 {
			midpoints := map[[2]int]int{}
			midpoint := func(a, b int) int {
				key := [2]int{min(a, b), max(a, b)}
				if v, ok := midpoints[key]; ok {
					return v
				}
				v := len(verts)
				verts = append(verts, verts[a].Add(verts[b]).Scale(0.5))
				midpoints[key] = v
				return v
			}
			var next [][3]int
			var nextSources []int
			for i, tri := range tris {
				a, b, c := midpoint(tri[0], tri[1]), midpoint(tri[1], tri[2]), midpoint(tri[2], tri[0])
				next = append(next, [3]int{tri[0], a, c}, [3]int{a, tri[1], b},
					[3]int{c, b, tri[2]}, [3]int{a, b, c})
				nextSources = append(nextSources, sources[i], sources[i], sources[i], sources[i])
			}
			tris, sources = next, nextSources
		}
		out, err := PrepContext(t.Context(), verts, tris, sources)
		require.NoError(t, err)
		bounds := make([]float64, len(verts))
		for i := range bounds {
			bounds[i] = float64(i+1) * 1e-9
		}
		out.SetVertexBounds(bounds)
		return out
	}
	run := func(workers int, op OperationKind) ([]KeptFacet, float64) {
		a := refine(boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{}))
		b := refine(boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{X: 0.31, Y: 0.43, Z: 0.57}))
		memo := NewContactMemo(a, b)
		kept, rim, err := MeshBoolean(orderedwork.WithWorkers(t.Context(), workers), op, a, b, memo)
		require.NoError(t, err)
		// Count actual classifier results to ensure both operands exceed the
		// subdivision threshold; a small box alone would take the serial path.
		for operand := range 2 {
			cut := map[int]struct{}{}
			for i := range a.Tris {
				for j := range b.Tris {
					contact, ok := memo.Lookup(i, j)
					if !ok || contact.Kind != ContactSegment || contact.EdgeA >= 0 || contact.EdgeB >= 0 {
						continue
					}
					facet := i
					if operand == 1 {
						facet = j
					}
					cut[facet] = struct{}{}
				}
			}
			require.GreaterOrEqual(t, len(cut), 32, "operand %d", operand)
		}
		return kept, rim
	}
	for _, op := range []OperationKind{OpUnion, OpCut, OpIntersect} {
		want, wantRim := run(1, op)
		for _, workers := range []int{2, 4} {
			got, gotRim := run(workers, op)
			require.Equal(t, wantRim, gotRim)
			require.NotEmpty(t, got)
			require.Len(t, got, len(want))
			for i := range want {
				require.Equal(t, want[i].Src, got[i].Src, "facet %d", i)
				require.Equal(t, want[i].Beta, got[i].Beta, "facet %d", i)
				for k := range 3 {
					require.Equal(t, want[i].V[k].Key(), got[i].V[k].Key(), "facet %d corner %d", i, k)
				}
			}
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := subdivideCutFacets(ctx, &BoolMesh{}, nil, make([]int, 32))
	require.ErrorIs(t, err, context.Canceled)
}

func TestCutFacetBatchesKeepEarlierClassificationError(t *testing.T) {
	verts := []r3.Vec{{}, {X: 4}, {Y: 4}}
	tris := make([][3]int, 32)
	for i := range tris {
		tris[i] = [3]int{0, 1, 2}
	}
	m, err := PrepContext(t.Context(), verts, tris, make([]int, len(tris)))
	require.NoError(t, err)
	partner, err := PrepContext(t.Context(), []r3.Vec{{X: 1, Y: -1, Z: -1},
		{X: 1, Y: 5, Z: -1}, {X: 1, Y: -1, Z: 5}}, [][3]int{{0, 1, 2}}, []int{0})
	require.NoError(t, err)
	contact, err := TriTriClassify(TriCorners(m, 0), TriCorners(partner, 0),
		XtriCorners(m, 0), XtriCorners(partner, 0), m.Norms[0], partner.Norms[0])
	require.NoError(t, err)
	require.Equal(t, ContactSegment, contact.Kind)
	require.True(t, contact.P0OnA && contact.P1OnA)
	cuts := map[int][]Xseg{}
	for i := range tris {
		cuts[i] = []Xseg{{A: contact.P0, B: contact.P1, AOnEdge: true, BOnEdge: true,
			Partner: TriCorners(partner, 0)}}
	}
	// Deliberately invalid metadata makes the first facet's classification
	// fail, while a later facet has a dangling chain that fails subdivision.
	cuts[0][0].Partner = TriCorners(m, 0)
	cuts[31] = []Xseg{{A: m.Xverts[0], B: proof.XptOf(r3.Vec{X: 1, Y: 1}), AOnEdge: true}}
	for _, workers := range []int{1, 4} {
		_, err := keepSide(orderedwork.WithWorkers(t.Context(), workers), m, partner, cuts, nil, nil, false, false, newKeyMemo())
		require.ErrorContains(t, err, "a region probe landed on its contact plane")
	}
}
