package meshbool

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func reuseTestBox(t *testing.T, lo, hi r3.Vec) *BoolMesh {
	t.Helper()
	verts := make([]r3.Vec, 8)
	for k := range verts {
		v := lo
		if k&1 != 0 {
			v.X = hi.X
		}
		if k&2 != 0 {
			v.Y = hi.Y
		}
		if k&4 != 0 {
			v.Z = hi.Z
		}
		verts[k] = v
	}
	quads := [][4]int{{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}}
	tris := make([][3]int, 0, 12)
	for _, q := range quads {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	bm, err := PrepContext(context.Background(), verts, tris, nil)
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	return bm
}

func TestBroadContactsMatchTightDepthWitness(t *testing.T) {
	t.Parallel()
	ctx := context.Background()
	const broadSlack = 0.2
	const tightSlack = 0.02
	a := reuseTestBox(t, r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1))
	b := reuseTestBox(t, r3.NewVec(0.2, 0.2, 0.95), r3.NewVec(0.8, 0.8, 1.95))
	allA, allB := AllFacets(a), AllFacets(b)
	memo := NewContactMemo(a, b)

	broad, deferred, err := GatherNearContacts(ctx, a, allA, b, allB, broadSlack, memo)
	require.NoError(t, err)
	require.False(t, deferred)
	tight, deferred, err := GatherNearContacts(ctx, a, allA, b, allB, tightSlack, memo)
	require.NoError(t, err)
	require.False(t, deferred)
	require.NotEmpty(t, tight.CloseA)
	require.Less(t, len(tight.CloseA)+len(tight.CloseB)+len(tight.Spans),
		len(broad.CloseA)+len(broad.CloseB)+len(broad.Spans),
		"the test must compare a strict subset against its broader contact set")

	gotBroad, err := facesNearMissFromContacts(ctx, a, b, broad, tightSlack)
	require.NoError(t, err)
	gotTight, err := facesNearMissFromContacts(ctx, a, b, tight, tightSlack)
	require.NoError(t, err)
	require.Equal(t, gotTight, gotBroad)
	require.False(t, gotBroad, "the boxes penetrate deeper than the tightened slack")
}

func TestDepthWitnessFitsRejectsOverCapContacts(t *testing.T) {
	t.Parallel()
	// Each over-cap dimension selects RefuseUndecidableProximity's original
	// staged depth checks instead of reusing the contact set.
	require.True(t, depthWitnessFits(NearContacts{
		CloseA: make([]int, MaxDepthWitnessFacets),
		CloseB: make([]int, MaxDepthWitnessFacets),
		Spans:  make([]ContactSpan, MaxDepthWitnessSpans),
	}))
	for name, contacts := range map[string]NearContacts{
		"first facets":  {CloseA: make([]int, MaxDepthWitnessFacets+1)},
		"second facets": {CloseB: make([]int, MaxDepthWitnessFacets+1)},
		"spans":         {Spans: make([]ContactSpan, MaxDepthWitnessSpans+1)},
	} {
		t.Run(name, func(t *testing.T) {
			require.False(t, depthWitnessFits(contacts))
		})
	}
}
