package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
)

func TestRingPlaneRejectPreservesExactDistance(t *testing.T) {
	wall := loftCylinderRingWall{points: [3]loftCylinderRingVec{
		loftCylinderRingVertex(r3.Vec{X: 0, Y: 0, Z: 0}),
		loftCylinderRingVertex(r3.Vec{X: 1, Y: 0, Z: 0}),
		loftCylinderRingVertex(r3.Vec{X: 0, Y: 1, Z: 0}),
	}}
	wall.normal = loftCylinderRingCross(loftCylinderRingSub(wall.points[1], wall.points[0]),
		loftCylinderRingSub(wall.points[2], wall.points[0]))
	wall.normal2 = loftCylinderRingDot(wall.normal, wall.normal)
	wall.plane0 = loftCylinderRingDot(wall.points[0], wall.normal)
	bound2 := big.NewRat(1, 4)
	rejected := 0
	for _, heights := range [][2]float64{
		{1, 1}, {-1, -1}, {0.25, 0.25}, {-1, 1}, {0.5, 0.5},
	} {
		a := loftCylinderRingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: heights[0]})
		b := loftCylinderRingVertex(r3.Vec{X: 0.5, Y: 0.25, Z: heights[1]})
		if !loftCylinderRingPlaneApart(a, b, wall, bound2) {
			continue
		}
		rejected++
		gap2 := loftCylinderRingSegmentTriangle2(a, b, wall.points)
		if gap2 == nil || gap2.Cmp(bound2) <= 0 {
			t.Fatalf("plane rejected a segment within the bound at heights %v", heights)
		}
	}
	if rejected != 2 {
		t.Fatalf("plane rejected %d segments, want 2 strictly separated segments", rejected)
	}
	below := loftCylinderRingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: -1})
	above := loftCylinderRingVertex(r3.Vec{X: 0.25, Y: 0.25, Z: 1})
	if loftCylinderRingPlaneApart(below, above, wall, bound2) {
		t.Fatal("plane rejected a crossing segment")
	}
}
