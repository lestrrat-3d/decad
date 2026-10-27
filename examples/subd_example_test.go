package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

// SubDMesh smooths a polygon cage, the way a Form workspace sculpts a shape by
// moving the vertices of a coarse box. Here the 2 mm cube is refined three times:
// its corners and edges round off and the volume falls from 8 mm^3 toward the
// smooth limit.
func Example_subd_mesh() {
	cage := decad.SubDCage{
		Vertices: []r3.Vec{
			{X: -1, Y: -1, Z: -1}, {X: 1, Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: -1, Y: 1, Z: -1},
			{X: -1, Y: -1, Z: 1}, {X: 1, Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: -1, Y: 1, Z: 1},
		},
		// Six quads, each wound counter-clockwise seen from outside.
		Faces: [][]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}},
	}
	smooth, err := decad.SubDMesh(context.Background(), cage, 3)
	if err != nil {
		fmt.Printf("failed to subdivide: %s\n", err)
		return
	}
	volume, err := smooth.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("closed: %t, triangles: %d, volume: %.2f mm^3\n",
		smooth.Report().Closed, smooth.Report().Triangles, volume.Value.Base())
	// Output:
	// closed: true, triangles: 768, volume: 2.66 mm^3
}
