package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferEdgeOrdersEndpoints(t *testing.T) {
	require.Equal(t, [2]int{3, 5}, loftcylinder.ChamferEdge(5, 3))
	require.Equal(t, [2]int{3, 5}, loftcylinder.ChamferEdge(3, 5))
}

func TestChamferContactOnSide(t *testing.T) {
	xverts := []proofarith.Xpt{
		proofarith.XptOf(r3.Vec{X: 0, Y: 0, Z: 0}),
		proofarith.XptOf(r3.Vec{X: 2, Y: 0, Z: 0}),
		proofarith.XptOf(r3.Vec{X: 0, Y: 2, Z: 0}),
	}
	tri := [3]int{0, 1, 2}
	at := func(x, y float64) proofarith.Xpt { return proofarith.XptOf(r3.Vec{X: x, Y: y}) }
	first := map[[2]int]struct{}{loftcylinder.ChamferEdge(0, 1): {}}
	second := map[[2]int]struct{}{loftcylinder.ChamferEdge(2, 1): {}}
	for _, tc := range []struct {
		name  string
		p     proofarith.Xpt
		sides map[[2]int]struct{}
		want  bool
	}{
		{"on the recorded edge", at(1, 0), first, true},
		{"collinear beyond the edge", at(3, 0), first, false},
		{"on an unrecorded edge", at(1, 1), first, false},
		{"on the reversed recorded edge", at(1, 1), second, true},
		{"no recorded edges", at(1, 0), nil, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, loftcylinder.ChamferContactOnSide(tc.p, tri, xverts, tc.sides))
		})
	}
}
