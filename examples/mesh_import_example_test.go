package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
)

// ReadOBJ and ReadSTL load a triangle mesh made elsewhere into a MeshBody.
// A MeshBody is not a solid Body: no feature takes it until SolidFromMesh turns it
// into one (Example_solid_from_mesh). Use it to ask what a
// mesh file holds before building around it: whether its triangles close into
// a surface, how big it is and how much volume it encloses.
func Example_mesh_import() {
	// A 2 mm cube as a Wavefront OBJ file, twelve triangles over eight corners.
	src := `v 0 0 0
v 2 0 0
v 2 2 0
v 0 2 0
v 0 0 2
v 2 0 2
v 2 2 2
v 0 2 2
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 3 4 8
f 3 8 7
f 2 3 7
f 2 7 6
f 4 1 5
f 4 5 8
`
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	report := mesh.Report()
	fmt.Printf("closed: %t, shells: %d, triangles: %d\n", report.Closed, report.Shells, report.Triangles)
	volume, err := mesh.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// closed: true, shells: 1, triangles: 12
	// volume: 8 mm^3
}
