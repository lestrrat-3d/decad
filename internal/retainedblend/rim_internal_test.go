package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// rimWall is a wall face (face 0) of the rim of radius 3 about the origin in the
// plane z = 0: the facet (a, b, c) holds the rim edge ab and leaves it to c, and
// the facet (b, d, e) holds the rim edge bd and leaves it to e. A further facet
// (c, e, f) holds no rim edge, nor does (c, e, g).
func rimWall(c, e, f, g r3.Vec) (facetedrecord.Payload, rim) {
	fp := facetedrecord.Payload{
		Verts:  []r3.Vec{r3.NewVec(3, 0, 0), r3.NewVec(0, 3, 0), c, r3.NewVec(-3, 0, 0), e, f, g},
		Tris:   [][3]int{{0, 1, 2}, {1, 3, 4}, {2, 4, 5}, {2, 4, 6}},
		FaceOf: []int{0, 0, 0, 0},
	}
	fp.VertexBound = []float64{0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001}
	return fp, rim{center: r3.Vec{}, n: r3.NewVec(0, 0, 1), radius: 3, scale: 1}
}

// TestRimLocalSide reads the side of a wall that lies on both sides of the rim's
// plane from the facets along the rim alone (§3.1): it takes the side the
// deciding facets agree on, and refuses when none decides or two disagree.
func TestRimLocalSide(t *testing.T) {
	t.Parallel()
	refuses := func(t *testing.T, err error) {
		t.Helper()
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the rim's wall does not lie on one side of its plane")
	}
	edge := &topo.Edge{}

	t.Run("the facets along the rim agree", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, 4), r3.NewVec(-3, 0, 4), r3.NewVec(0, 0, -4), r3.NewVec(0, 0, -5))
		require.NoError(t, r.readSide(edge, fp, 0, true))
		require.True(t, r.twoSided)
		require.Equal(t, 1.0, r.side)
		require.False(t, r.convex)
	})
	t.Run("the other side", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, -4), r3.NewVec(-3, 0, -4), r3.NewVec(0, 0, 4), r3.NewVec(0, 0, 5))
		convex := &topo.Edge{Convex: true}
		require.NoError(t, r.readSide(convex, fp, 0, true))
		require.True(t, r.twoSided)
		require.Equal(t, -1.0, r.side)
	})
	t.Run("the facets along the rim disagree", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, 4), r3.NewVec(-3, 0, -4), r3.NewVec(0, 0, 4), r3.NewVec(0, 0, 5))
		refuses(t, r.readSide(edge, fp, 0, true))
	})
	t.Run("every facet along the rim lies within its bound of the plane", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, 0.0005), r3.NewVec(-3, 0, 0.0005), r3.NewVec(0, 0, -4), r3.NewVec(0, 0, 4))
		refuses(t, r.readSide(edge, fp, 0, true))
	})
	t.Run("a closed rim keeps the whole face", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, 4), r3.NewVec(-3, 0, 4), r3.NewVec(0, 0, -4), r3.NewVec(0, 0, -5))
		refuses(t, r.readSide(edge, fp, 0, false))
		require.False(t, r.twoSided)
	})
	t.Run("a wall on one side reads as before", func(t *testing.T) {
		t.Parallel()
		fp, r := rimWall(r3.NewVec(3, 0, 4), r3.NewVec(-3, 0, 4), r3.NewVec(0, 0, 4), r3.NewVec(0, 0, 5))
		require.NoError(t, r.readSide(edge, fp, 0, true))
		require.False(t, r.twoSided)
		require.Equal(t, 1.0, r.side)
	})
}

// TestBandChordMeets tests a wall boundary chord against the band widened by the
// chord's bound: a chord through the band, or within the widening of it, meets
// it, and one outside does not.
func TestBandChordMeets(t *testing.T) {
	t.Parallel()
	// The band 0 ≤ x ≤ 1, |y| ≤ 1 in the plane z = 0, as two exact planes each.
	side := func(n r3.Vec, o r3.Vec, sign float64) *bandEnd { return &bandEnd{g: plane{n: n, o: o}, sign: sign} }
	b := &wallBand{
		main: side(r3.NewVec(1, 0, 0), r3.NewVec(1, 0, 0), -1),
		others: []*bandEnd{
			side(r3.NewVec(1, 0, 0), r3.Vec{}, 1),
			side(r3.NewVec(0, 1, 0), r3.NewVec(0, 1, 0), -1),
			side(r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0), 1),
		},
	}
	require.True(t, b.chordMeets(r3.NewVec(0.5, -3, 0), r3.NewVec(0.5, 3, 0), 0))
	require.True(t, b.chordMeets(r3.NewVec(0.5, 0.5, 0), r3.NewVec(0.6, 0.6, 0), 0))
	require.False(t, b.chordMeets(r3.NewVec(1.5, -3, 0), r3.NewVec(1.5, 3, 0), 0.1))
	require.True(t, b.chordMeets(r3.NewVec(1.05, -3, 0), r3.NewVec(1.05, 3, 0), 0.1))
	require.False(t, b.chordMeets(r3.NewVec(-0.5, 0.5, 0), r3.NewVec(-0.5, 0.6, 0), 0.1))
	require.False(t, b.chordMeets(r3.NewVec(0.5, 1.5, 0), r3.NewVec(3, 1.5, 0), 0.1))
}
