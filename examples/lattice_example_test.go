package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// GyroidLattice fills a box with a gyroid, the smooth triply periodic wall used
// for light, strong print infill, as a closed mesh body. Level sets the wall's
// thickness and the box's faces cap it, so the mesh is watertight and its volume
// reads. Here a 20 mm cube holds a lattice repeating every 10 mm.
func Example_gyroid_lattice() {
	lattice, err := decad.GyroidLattice(context.Background(), decad.LatticeSpec{
		Min:        r3.Vec{},
		Max:        r3.Vec{X: 20, Y: 20, Z: 20},
		Cell:       units.Millimeters(10),
		Level:      units.Scalar(0.8),
		Resolution: 16,
	})
	if err != nil {
		fmt.Printf("failed to build the lattice: %s\n", err)
		return
	}
	volume, err := lattice.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fraction := volume.Value.Base() / 8000
	fmt.Printf("closed: %t, solid fraction: %.2f\n", lattice.Report().Closed, fraction)
	// Output:
	// closed: true, solid fraction: 0.53
}
