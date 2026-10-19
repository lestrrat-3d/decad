package loftcylinder

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRingPlaneRejectPreservesExactDistance(t *testing.T) {
	wall := RingWall{Points: [3]RingVec{
		RingVertex(r3.Vec{X: 0, Y: 0, Z: 0}),
		RingVertex(r3.Vec{X: 1, Y: 0, Z: 0}),
		RingVertex(r3.Vec{X: 0, Y: 1, Z: 0}),
	}}
	wall.Normal = RingCross(RingSub(wall.Points[1], wall.Points[0]),
		RingSub(wall.Points[2], wall.Points[0]))
	wall.Normal2 = RingDot(wall.Normal, wall.Normal)
	wall.Plane0 = RingDot(wall.Points[0], wall.Normal)
	bound2 := big.NewRat(1, 4)
	rejected := 0
	for _, heights := range [][2]float64{
		{1, 1}, {-1, -1}, {0.25, 0.25}, {-1, 1}, {0.5, 0.5},
	} {
		a := RingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: heights[0]})
		b := RingVertex(r3.Vec{X: 0.5, Y: 0.25, Z: heights[1]})
		if !ringPlaneApart(ringSegmentPlaneOf(a, b, bound2), &wall) {
			continue
		}
		rejected++
		gap2 := ringSegmentTriangle2(a, b, wall.Points)
		if gap2 == nil || gap2.Cmp(bound2) <= 0 {
			t.Fatalf("plane rejected a segment within the bound at heights %v", heights)
		}
	}
	if rejected != 2 {
		t.Fatalf("plane rejected %d segments, want 2 strictly separated segments", rejected)
	}
	below := RingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: -1})
	above := RingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: 1})
	if ringPlaneApart(ringSegmentPlaneOf(below, above, bound2), &wall) {
		t.Fatal("plane rejected a crossing segment")
	}
}

func TestRingCycle(t *testing.T) {
	pt := func(x, y, z int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(z), W: big.NewInt(1)}
	}
	edge := func(a, b proofarith.Xpt) [2]proofarith.Xpt { return [2]proofarith.Xpt{a, b} }
	square := [][2]proofarith.Xpt{
		edge(pt(1, 1, 2), pt(0, 1, 2)),
		edge(pt(0, 0, 2), pt(1, 0, 2)),
		edge(pt(0, 1, 2), pt(0, 0, 2)),
		edge(pt(1, 0, 2), pt(1, 1, 2)),
	}
	t.Run("square in any order", func(t *testing.T) {
		level, ok := RingCycle(square)
		require.True(t, ok)
		require.Zero(t, level.Cmp(big.NewRat(2, 1)))
	})
	t.Run("two disjoint triangles", func(t *testing.T) {
		edges := [][2]proofarith.Xpt{
			edge(pt(0, 0, 2), pt(1, 0, 2)), edge(pt(1, 0, 2), pt(0, 1, 2)), edge(pt(0, 1, 2), pt(0, 0, 2)),
			edge(pt(5, 5, 2), pt(6, 5, 2)), edge(pt(6, 5, 2), pt(5, 6, 2)), edge(pt(5, 6, 2), pt(5, 5, 2)),
		}
		_, ok := RingCycle(edges)
		require.False(t, ok)
	})
	t.Run("repeated edge", func(t *testing.T) {
		edges := append([][2]proofarith.Xpt{}, square...)
		edges[3] = edge(pt(1, 0, 2), pt(0, 0, 2))
		_, ok := RingCycle(edges)
		require.False(t, ok)
	})
	t.Run("edge at another height", func(t *testing.T) {
		edges := append([][2]proofarith.Xpt{}, square...)
		edges[3] = edge(pt(1, 0, 3), pt(1, 1, 3))
		_, ok := RingCycle(edges)
		require.False(t, ok)
	})
	t.Run("too few edges", func(t *testing.T) {
		_, ok := RingCycle(square[:2])
		require.False(t, ok)
	})
}
