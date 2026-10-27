package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Shell takes a solid read from a mesh file when its faces all square to an axis, such
// as a box from STL or OBJ: it opens the faces you name and lines the rest with a wall,
// the corners mitered as the planes of a CAD shell are. Here a 10 x 8 x 6 mm box read
// from OBJ is opened at its top with a 1 mm wall.
func Example_shell_mesh_solid() {
	src := `v 0 0 0
v 10 0 0
v 10 8 0
v 0 8 0
v 0 0 6
v 10 0 6
v 10 8 6
v 0 8 6
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
	box, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	cup, err := decad.Shell(context.Background(), box, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	volume, err := cup.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The box holds 480 mm^3 and the cavity 8 x 6 x 5 = 240.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 240 mm^3
}
