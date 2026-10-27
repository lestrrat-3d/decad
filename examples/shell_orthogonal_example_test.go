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

// MoveFace press-pulls a face of such a solid: it moves the face along its normal
// and the walls beside it follow. A positive distance grows the body, a negative one
// cuts it. Here the top of a 10 x 8 x 6 mm box read from OBJ is pulled down 2 mm.
func Example_move_face() {
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
	lower, err := decad.MoveFace(context.Background(), box, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(-2))
	if err != nil {
		fmt.Printf("failed to move the face: %s\n", err)
		return
	}
	volume, err := lower.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 320 mm^3
}

// Stretch scales a faceted solid by a separate factor along each world axis about
// an anchor, which ScaledAt cannot do. Here a 10 x 8 x 6 mm box read from OBJ is
// stretched to twice its length and half its width.
func Example_stretch() {
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
	longer, err := decad.Stretch(context.Background(), box, r3.NewVec(0, 0, 0), units.Scalar(2), units.Scalar(0.5), units.Scalar(1))
	if err != nil {
		fmt.Printf("failed to stretch: %s\n", err)
		return
	}
	volume, err := longer.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 480 mm^3
}

// A Centered shell puts half the wall on each side of the original surface. Here a
// 10 x 8 x 6 mm box read from OBJ is wrapped by a closed 2 mm wall: 1 mm outside its
// surface and 1 mm inside.
func Example_shell_centered() {
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
	wall, err := decad.Shell(context.Background(), box, nil, units.Millimeters(2),
		decad.WithNoOpenings(), decad.WithShellSense(decad.Centered))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	volume, err := wall.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 768 mm^3
}
