package capedge_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestP8TopFrontEdge(t *testing.T) {
	a, b := r3.NewVec(3, 0, 20), r3.NewVec(37, 0, 20)
	require.True(t, capedge.P8TopFrontEdge(a, b))
	require.True(t, capedge.P8TopFrontEdge(b, a))
	require.False(t, capedge.P8TopFrontEdge(a, a))
	require.False(t, capedge.P8TopFrontEdge(a, r3.NewVec(37, 0, 19)))
	require.False(t, capedge.P8TopFrontEdge(r3.NewVec(3, 1, 20), b))
}
