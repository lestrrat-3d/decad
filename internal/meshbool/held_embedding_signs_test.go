package meshbool

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPairSeparatedReusesExactClassifierSigns(t *testing.T) {
	base := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0}}
	cases := map[string]struct {
		other [3]r3.Vec
		tri   [3]int
	}{
		"crossing": {
			other: [3]r3.Vec{{X: 0.5, Y: 0.5, Z: -1}, {X: 0.5, Y: 0.5, Z: 1}, {X: 1, Y: 1, Z: 0}},
			tri:   [3]int{3, 4, 5},
		},
		"shared vertex": {
			other: [3]r3.Vec{{}, {X: 1, Y: 1, Z: 1}, {X: 1, Y: 1, Z: -1}},
			tri:   [3]int{0, 4, 5},
		},
		"coplanar overlap": {
			other: [3]r3.Vec{{X: 0.5, Y: 0.5, Z: 0}, {X: 1.5, Y: 0.5, Z: 0}, {X: 0.5, Y: 1.5, Z: 0}},
			tri:   [3]int{3, 4, 5},
		},
	}
	for name, test := range cases {
		t.Run(name, func(t *testing.T) {
			h := HeldRounding{Verts: append(append([]r3.Vec{}, base...), test.other[:]...),
				Tris: [][3]int{{0, 1, 2}, test.tri}}
			e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			var sharedA, sharedB []int
			for a, va := range h.Tris[0] {
				for b, vb := range h.Tris[1] {
					if va == vb {
						sharedA, sharedB = append(sharedA, a), append(sharedB, b)
					}
				}
			}
			_, decided, signs := e.pairSeparated(0, 1, sharedA, sharedB)
			require.False(t, decided)
			fa, xa, na := e.facet(0)
			fb, xb, nb := e.facet(1)
			for k := range 3 {
				require.Equal(t, proof.OrientSignPrepared(fb[0], fb[1], fb[2], fa[k], xb[0], xa[k], nb), signs.a[k])
				require.Equal(t, proof.OrientSignPrepared(fa[0], fa[1], fa[2], fb[k], xa[0], xb[k], na), signs.b[k])
			}
			got, gotErr := TriTriClassifyCore(fa, fb, xa, xb, na, nb, nil, nil,
				&signs.a, &signs.b, true, e.fnorm[0], e.fnorm[1], true)
			want, wantErr := TriTriClassifyPrepared(fa, fb, xa, xb, na, nb, e.fnorm[0], e.fnorm[1])
			require.Equal(t, wantErr, gotErr)
			require.Equal(t, want, got)
		})
	}
}
