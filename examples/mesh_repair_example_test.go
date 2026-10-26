package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// RepairMesh mends a mesh file that is nearly a solid: it welds vertices that
// should coincide, winds every face the same way and closes a hole. Read the
// result's Report to see whether it closed. Here a cube's bottom face is missing
// and one side face is wound backwards.
func Example_mesh_repair() {
	src := `v 0 0 0
v 2 0 0
v 2 2 0
v 0 2 0
v 0 0 2
v 2 0 2
v 2 2 2
v 0 2 2
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 3 4 8
f 3 7 8
f 2 3 7
f 2 7 6
f 4 1 5
f 4 5 8
`
	broken, err := decad.ReadOBJ(context.Background(), strings.NewReader(src))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	report := broken.Report()
	fmt.Printf("before: closed %t, %d open edges, %d misoriented\n", report.Closed, report.BoundaryEdges, report.MisorientedEdges)

	fixed, repair, err := decad.RepairMesh(context.Background(), broken,
		decad.WithMeshWeld(units.Millimeters(1e-6)), decad.WithMeshOrient(), decad.WithMeshFillHoles())
	if err != nil {
		fmt.Printf("failed to repair: %s\n", err)
		return
	}
	volume, err := fixed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("after: closed %t, %d triangle(s) flipped, %d loop(s) filled, volume %.0f mm^3\n",
		fixed.Report().Closed, repair.FlippedTriangles, repair.FilledLoops, volume.Value.Base())
	// Output:
	// before: closed false, 4 open edges, 3 misoriented
	// after: closed true, 1 triangle(s) flipped, 1 loop(s) filled, volume 8 mm^3
}
