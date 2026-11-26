package decad

// CI: isolated-tests

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// TestPairDigestIgnoresContainers builds one pair in the root container of a document and the same geometry as a
// pair split across two child containers of another. The pair digests must be equal and IntersectionVolume, which
// skips the container gate, must answer both pairs alike.
func TestPairDigestIgnoresContainers(t *testing.T) {
	t.Parallel()
	one := New()
	a1 := ivBox(t, one, 0, 0, 10, 10, 0)
	b1 := ivBox(t, one, 5, 5, 15, 15, 5)

	split := New()
	a2 := ivBox(t, split, 0, 0, 10, 10, 0)
	b2 := ivBox(t, split, 5, 5, 15, 15, 5)
	left, err := split.Root().NewChild("left")
	require.NoError(t, err)
	right, err := split.Root().NewChild("right")
	require.NoError(t, err)
	require.NoError(t, left.AddBody(a2))
	require.NoError(t, right.AddBody(b2))

	d1, err := one.PairDigest(t.Context(), a1, b1)
	require.NoError(t, err)
	d2, err := split.PairDigest(t.Context(), a2, b2)
	require.NoError(t, err)
	require.Equal(t, d1, d2)

	v1, err1 := one.IntersectionVolume(t.Context(), a1, b1)
	v2, err2 := split.IntersectionVolume(t.Context(), a2, b2)
	require.NoError(t, err1)
	require.NoError(t, err2)
	require.Equal(t, v1, v2)
}
