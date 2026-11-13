package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestExplodeHiddenExact shows Hidden reads the boxes' exact projections: two cubes standing diagonally
// apart have overlapping bounding rectangles along (1, 1, 1) but their projections are apart.
func TestExplodeHiddenExact(t *testing.T) {
	doc := decad.New()
	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: "a", Kind: parts.Manufactured, Body: block(t, doc, 0, 0, 10, 10, 0, 10)},
		parts.Part{Name: "b", Kind: parts.Manufactured, Body: block(t, doc, -11, 11, -1, 21, 0, 10)})
	ex, err := s.Explode(t.Context(), parts.WithExplodeGround("a"))
	require.NoError(t, err)

	hidden, err := ex.Hidden(r3.Vec{X: 1, Y: 1, Z: 1})
	require.NoError(t, err)
	require.Empty(t, hidden, "the exact projections are apart")

	hidden, err = ex.Hidden(r3.Vec{X: 1, Y: -1})
	require.NoError(t, err)
	require.Len(t, hidden, 1, "along (1, -1, 0) one cube's shadow meets the other's")
	require.Greater(t, hidden[0].Covered.Base(), 0.0)

	hidden, err = ex.Hidden(r3.Vec{Z: -1})
	require.NoError(t, err)
	require.Empty(t, hidden, "side by side along -Z")
}
