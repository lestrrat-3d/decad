package revolvemesh_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCapFrame(t *testing.T) {
	basis := revolvemesh.RevolveBasis{
		A3: r3.Vec{},
		W:  r3.Vec{X: 1},
		E0: r3.Vec{Y: 1},
		E1: r3.Vec{Z: 1},
	}
	end, err := revolvemesh.CapFrame(basis, r3.Identity(), 0, false)
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: 1}, end.N())

	start, err := revolvemesh.CapFrame(basis, r3.Identity(), 0, true)
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: -1}, start.N())

	degenerate := basis
	degenerate.W = basis.E0
	_, err = revolvemesh.CapFrame(degenerate, r3.Identity(), 0, false)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "the placed cap frame is degenerate")
}
