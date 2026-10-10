package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A rectangular boss on a plate has a rounded cavity where the two slabs
// meet. The top opening is the boss cap from the Union result.
func Example_decad_shell_stacked_boss() {
	ctx := context.Background()
	doc := decad.New()
	box := func(x0, y0, x1, y1, z0, height float64) (*decad.Body, error) {
		world := sketch.NewWorld()
		plane, err := world.CreateOffsetPlane(world.XY(), z0)
		if err != nil {
			return nil, err
		}
		s, err := world.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		r := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(r.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		//nolint:contextcheck // Extrude has no context parameter.
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	plate, err := box(-10, -10, 10, 10, 0, 5)
	if err != nil {
		fmt.Println(err)
		return
	}
	boss, err := box(-5, -5, 5, 5, 5, 5)
	if err != nil {
		fmt.Println(err)
		return
	}
	stack, err := decad.Union(ctx, plate, boss)
	if err != nil {
		fmt.Println(err)
		return
	}
	shell, err := stack.Shell(ctx, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
		units.Millimeters(1))
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := shell.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("%.3f mm³\n", volume.Value.Base())
	// Output: 1116.201 mm³
}
