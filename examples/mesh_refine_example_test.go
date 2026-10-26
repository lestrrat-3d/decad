package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// RefineMesh splits the long edges of a mesh body so that none is longer than a
// limit, without moving any vertex: the surface, its area and its volume stay as
// they were, and only the triangles get smaller. Use it before a step that needs
// short edges. Here a 10 mm cube whose face diagonals are 14 mm long is cut down
// to edges of at most 5 mm.
func Example_mesh_refine() {
	const cube = `v 0 0 0
v 10 0 0
v 10 10 0
v 0 10 0
v 0 0 10
v 10 0 10
v 10 10 10
v 0 10 10
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
`
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(cube))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	refined, err := decad.RefineMesh(context.Background(), mesh, units.Millimeters(5))
	if err != nil {
		fmt.Printf("failed to refine: %s\n", err)
		return
	}
	volume, err := refined.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d -> %d, closed: %t\n", mesh.Report().Triangles, refined.Report().Triangles, refined.Report().Closed)
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// triangles: 12 -> 96, closed: true
	// volume: 1000 mm^3
}
