package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// ReduceMesh thins a dense mesh by clustering its vertices into cubic cells: the
// vertices in a cell become one at their mean. Use it to shrink a scan or a fine
// tessellation before checking or exporting it. The report gives the greatest
// distance a vertex moved. Here a 4 by 4 grid of unit squares is thinned with
// 2 mm cells.
func Example_mesh_reduce() {
	var src strings.Builder
	for y := range 5 {
		for x := range 5 {
			fmt.Fprintf(&src, "v %d %d 0\n", x, y)
		}
	}
	at := func(x, y int) int { return y*5 + x + 1 }
	for y := range 4 {
		for x := range 4 {
			fmt.Fprintf(&src, "f %d %d %d\nf %d %d %d\n",
				at(x, y), at(x+1, y), at(x+1, y+1), at(x, y), at(x+1, y+1), at(x, y+1))
		}
	}
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src.String()))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	reduced, report, err := decad.ReduceMesh(context.Background(), mesh, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to reduce: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d -> %d\n", mesh.Report().Triangles, reduced.Report().Triangles)
	fmt.Printf("vertices moved at most %.2f mm\n", report.MaxVertexMove.Base())
	// Output:
	// triangles: 32 -> 8
	// vertices moved at most 0.71 mm
}
