package proof_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestDyMaxDyMin(t *testing.T) {
	t.Parallel()

	small, large := proof.MustDyOf(-2.5), proof.MustDyOf(7)
	for _, pair := range [][2]proof.Dyadic{{small, large}, {large, small}} {
		require.Zero(t, proof.DyCmp(large, proof.DyMax(pair[0], pair[1])))
		require.Zero(t, proof.DyCmp(small, proof.DyMin(pair[0], pair[1])))
	}
	require.Zero(t, proof.DyCmp(small, proof.DyMax(small, small)))
	require.Zero(t, proof.DyCmp(large, proof.DyMin(large, large)))
}
