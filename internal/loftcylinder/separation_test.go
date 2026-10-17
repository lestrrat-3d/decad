package loftcylinder

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMovingSeparateChecksTrianglesAndWork(t *testing.T) {
	a := SeparationTooth{
		Bottom: [2]float64{-0.05, 0.05}, Top: [2]float64{-0.05, 0.05},
		Verts: []r3.Vec{{X: 1, Y: -0.1}, {X: 1, Y: 0.1}, {X: 1, Z: 1}},
		Tris:  [][3]int{{0, 1, 2}}, Z0: 0, Z1: 1,
	}
	b := SeparationTooth{
		Bottom: [2]float64{math.Pi - 0.05, math.Pi + 0.05},
		Top:    [2]float64{math.Pi - 0.05, math.Pi + 0.05},
		Verts:  []r3.Vec{{X: -1, Y: -0.1}, {X: -1, Y: 0.1}, {X: -1, Z: 1}},
		Tris:   [][3]int{{0, 1, 2}}, Z0: 0, Z1: 1,
	}
	remaining := 12
	require.True(t, movingSeparate(a, b, &remaining))
	require.Zero(t, remaining)

	remaining = 11
	require.False(t, movingSeparate(a, b, &remaining))
	require.Equal(t, 11, remaining)

	b.Verts[2].X = 0
	remaining = 12
	require.False(t, movingSeparate(a, b, &remaining))
}
