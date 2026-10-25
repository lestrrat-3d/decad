package prismcells_test

import (
	"errors"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/stretchr/testify/require"
)

func TestWorkLimitErrorPreservesRefusalAndCounts(t *testing.T) {
	original := fmt.Errorf("%w: arrangement is too large", decaderr.ErrUnsupported)
	err := prismcells.WithWorkLimit(original, prismcells.MaxArrangementSegments+1)
	var limit *prismcells.WorkLimitError
	require.ErrorAs(t, err, &limit)
	require.Equal(t, prismcells.MaxArrangementSegments+1, limit.Observed)
	require.Equal(t, prismcells.MaxArrangementSegments, limit.Maximum)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.True(t, errors.Is(err, original))
}
