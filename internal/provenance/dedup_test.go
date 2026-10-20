package provenance_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/stretchr/testify/require"
)

func TestDedupProducers(t *testing.T) {
	t.Parallel()

	require.Nil(t, provenance.DedupProducers(nil))
	require.Nil(t, provenance.DedupProducers([]provenance.Producer{}))
	require.Equal(t, []provenance.Producer{3, 1, 2}, provenance.DedupProducers([]provenance.Producer{3, 1, 3, 2, 1, 2}))
	input := []provenance.Producer{5, 5}
	require.Equal(t, []provenance.Producer{5}, provenance.DedupProducers(input))
	require.Equal(t, []provenance.Producer{5, 5}, input, "the input is not edited")
}
