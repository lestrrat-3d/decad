package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// boxOBJ is a 10 x 8 x 6 mm box as OBJ text.
const boxOBJ = `v 0 0 0
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

// Shell takes a solid read from a mesh file when its faces all square to an axis, such
// as a box from STL or OBJ: it opens the faces you name and lines the rest with a wall,
// the corners mitered as the planes of a CAD shell are. Here a 10 x 8 x 6 mm box read
// from OBJ is opened at its top with a 1 mm wall.
func Example_shell_mesh_solid() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
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
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
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
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
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
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
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

// Two faceted solids every face of which squares to an axis, such as parts read from STL,
// combine exactly under Union, Cut and Intersect, whatever the contact between them. Here
// two 10 mm cubes that touch at a face are joined into one bar.
func Example_orthogonal_union() {
	cube := func(x float64) string {
		var src strings.Builder
		for _, c := range [][3]float64{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}} {
			fmt.Fprintf(&src, "v %g %g %g\n", x+10*c[0], 10*c[1], 10*c[2])
		}
		src.WriteString("f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 3 4 8\nf 3 8 7\nf 2 3 7\nf 2 7 6\nf 4 1 5\nf 4 5 8\n")
		return src.String()
	}
	doc := decad.New()
	var solids [2]*decad.Body
	for i, x := range []float64{0, 10} {
		mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(cube(x)))
		if err != nil {
			fmt.Printf("failed to read mesh: %s\n", err)
			return
		}
		if solids[i], err = decad.SolidFromMesh(context.Background(), doc, mesh); err != nil {
			fmt.Printf("failed to make a solid: %s\n", err)
			return
		}
	}
	bar, err := decad.Union(context.Background(), solids[0], solids[1])
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	volume, err := bar.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3, faces: %d\n", volume.Value.Base(), len(bar.Faces()))
	// Output:
	// volume: 2000 mm^3, faces: 6
}

// Section cuts a solid by a plane into a sketch that can enter another feature. A
// faceted solid whose faces all square to an axis, such as a part read from STL,
// sections exactly by a plane square to an axis. Here a 10 x 8 x 6 mm block read from OBJ
// is cut at mid height and the section's area is read.
func Example_section_mesh_solid() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	box, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 3), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		fmt.Printf("failed to make a frame: %s\n", err)
		return
	}
	section, profiles, err := box.Section(context.Background(), frame)
	if err != nil {
		fmt.Printf("failed to section: %s\n", err)
		return
	}
	measured, err := decad.MeasureProfile(section, profiles[0])
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to read the area: %s\n", err)
		return
	}
	fmt.Printf("section area: %.0f mm^2\n", area.Value.Base())
	// Output:
	// section area: 80 mm^2
}

// Offset moves every face of a solid along its normal. For a faceted solid whose faces all
// square to an axis, such as a part read from STL, the faces meet in sharp corners, and the
// result is a second solid beside the first. Here a 10 x 8 x 6 mm block read from OBJ grows by 2 mm.
func Example_offset_solid() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	box, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	grown, err := decad.Offset(context.Background(), box, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to offset: %s\n", err)
		return
	}
	volume, err := grown.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1680 mm^3
}

// QuarterTurn turns a faceted solid by whole quarter turns about a world axis. It exchanges coordinates
// instead of multiplying by a rotation, so the turned solid is exact and stays on the exact routes. Here a
// 10 x 8 x 6 mm block read from OBJ is turned a quarter about +Z through the origin, so that its x extent
// becomes the y extent.
func Example_quarter_turn() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	box, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	turned, err := decad.QuarterTurn(context.Background(), box, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 0), 1)
	if err != nil {
		fmt.Printf("failed to turn: %s\n", err)
		return
	}
	bounds, err := turned.Bounds()
	if err != nil {
		fmt.Printf("failed to read the bounds: %s\n", err)
		return
	}
	fmt.Printf("x: %.0f to %.0f, y: %.0f to %.0f\n", bounds.Min.X, bounds.Max.X, bounds.Min.Y, bounds.Max.Y)
	// Output:
	// x: -8 to 0, y: 0 to 10
}

// WithShellFaceThickness gives the walls that face one direction a thickness of their own, so a
// shelled part can have a heavy floor and thin sides. Here the 10 x 8 x 6 mm block read from OBJ is shelled
// open at the top with 1 mm walls and a 3 mm floor.
func Example_shell_face_thickness() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(boxOBJ))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	box, err := decad.SolidFromMesh(context.Background(), decad.New(), mesh)
	if err != nil {
		fmt.Printf("failed to make a solid: %s\n", err)
		return
	}
	cup, err := decad.Shell(context.Background(), box, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1),
		decad.WithShellFaceThickness(r3.NewVec(0, 0, -1), units.Millimeters(3)))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	volume, err := cup.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The cavity is 8 x 6 x 3 mm, so the walls are 480 - 144 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 336 mm^3
}
