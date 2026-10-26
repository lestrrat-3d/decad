package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ScaleMesh stretches a mesh body by a separate factor along each axis about a
// point, which a solid body cannot do: ScaledAt is uniform. A negative factor
// mirrors the mesh. Here a 10 mm cube is stretched to 20 by 10 by 5 mm.
func Example_mesh_scale() {
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
	stretched, err := decad.ScaleMesh(context.Background(), mesh, r3.Vec{}, units.Scalar(2), units.Scalar(1), units.Scalar(0.5))
	if err != nil {
		fmt.Printf("failed to scale: %s\n", err)
		return
	}
	volume, err := stretched.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	size := stretched.Bounds().Max
	fmt.Printf("size: %g x %g x %g mm\n", size.X, size.Y, size.Z)
	fmt.Printf("volume: %g mm^3\n", volume.Value.Base())
	// Output:
	// size: 20 x 10 x 5 mm
	// volume: 1000 mm^3
}
