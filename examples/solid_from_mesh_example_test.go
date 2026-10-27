package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
)

// SolidFromMesh turns a closed mesh read from a file into a solid Body, after
// proving the mesh does not cross itself. The body then takes part in Booleans
// and measurement as any other. Here a 2 mm cube read from OBJ becomes a solid of
// six faces whose volume is exact, because the mesh is the solid.
func Example_solid_from_mesh() {
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
	cube, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	volume, err := cube.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d, volume: %.0f mm^3, bound: %.0f\n", len(cube.Faces()), volume.Value.Base(), volume.Bound.Base())
	// Output:
	// faces: 6, volume: 8 mm^3, bound: 0
}
