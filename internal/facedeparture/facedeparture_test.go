package facedeparture_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/stretchr/testify/require"
)

// TestStatedWithoutReader runs Stated in a binary that never loads the root
// package, so Reader is unset: every value reads as stating a departure, and
// a reader that only refuses on it refuses. Shown to fail with Stated
// returning false when Reader is nil.
func TestStatedWithoutReader(t *testing.T) {
	t.Parallel()
	require.Nil(t, facedeparture.Reader)
	require.True(t, facedeparture.Stated(struct{}{}))
}
