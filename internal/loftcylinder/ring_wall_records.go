package loftcylinder

import (
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/r3"
)

// RingToothWalls builds the exposed and fitted wall records of one Loft tooth.
// The caller has already checked that its wall displacement is finite.
func RingToothWalls(lp loftrecord.Payload, bound float64) ([]RingWall, []RingWall, bool) {
	n := len(lp.Verts) / 2
	root := -1
	for j, segment := range lp.Segment[0] {
		if segment == 5 {
			root = j
			break
		}
	}
	if root < 3 || lp.Walls != 2*n {
		return nil, nil, false
	}
	exterior := make([]RingWall, 0, 2*root)
	fitted := make([]RingWall, 0, 2*(root-2))
	for cell := range n {
		for half := range 2 {
			indices := lp.Tris[2*cell+half]
			tri := [3]r3.Vec{lp.Verts[indices[0]], lp.Verts[indices[1]], lp.Verts[indices[2]]}
			wall := RingWall{Box: RingTriangleBox(tri, bound)}
			for k, p := range tri {
				wall.Points[k] = RingVertex(p)
			}
			wall.Normal = RingCross(RingSub(wall.Points[1], wall.Points[0]),
				RingSub(wall.Points[2], wall.Points[0]))
			wall.Normal2 = RingDot(wall.Normal, wall.Normal)
			wall.Plane0 = RingDot(wall.Points[0], wall.Normal)
			if cell < root {
				exterior = append(exterior, wall)
			}
			if cell > 0 && cell < root-1 {
				fitted = append(fitted, wall)
			}
		}
	}
	return exterior, fitted, true
}
