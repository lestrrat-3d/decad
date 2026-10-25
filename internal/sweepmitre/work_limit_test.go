package sweepmitre_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/stretchr/testify/require"
)

func TestMitredSweepWorkLimitDetails(t *testing.T) {
	err := sweepmitre.Preflight(nil, 3, 2)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "sweep_spans", detail.Resource)
	require.Equal(t, uint64(3), detail.Observed)
	require.Equal(t, uint64(2), detail.Maximum)
	err = sweepmitre.Preflight([][]int{make([]int, 1000)}, 2, 256)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "facet_pairs", detail.Resource)
	require.Equal(t, uint64(17_973_010), detail.Observed)
	require.Equal(t, uint64(8_000_000), detail.Maximum)
}
