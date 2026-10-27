package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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

// A solid made from a mesh keeps an exact plane for every face square to an axis,
// so Fillet and Chamfer work on its edges like those of a built part. Here the
// cube of Example_solid_from_mesh has one edge rounded with a 0.5 mm fillet.
func Example_solid_from_mesh_fillet() {
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
	// The edge along z at the (2, 2) corner.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(2, 2, 0)))
	rounded, err := decad.Fillet(context.Background(), cube, edge, units.Millimeters(0.5))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	volume, err := rounded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The fillet removes (1 - pi/4) x 0.25 x 2 = 0.107 mm^3 of the cube's 8.
	fmt.Printf("volume: %.2f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 7.89 mm^3
}
