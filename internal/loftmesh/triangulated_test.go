package loftmesh_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/stretchr/testify/require"
)

func TestErrPointUnrepresentable(t *testing.T) {
	err := loftmesh.ErrPointUnrepresentable("x")
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the loft's x runs past the representable float64 range")
}
