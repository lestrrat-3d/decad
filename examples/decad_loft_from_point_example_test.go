package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_loft_from_point() {
	ctx := context.Background()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 6)
	if err != nil {
		fmt.Println(err)
		return
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Println(err)
		return
	}
	rectangle := s.CreateRectangle(-2, -2, 2, 2)
	s.Fix(rectangle.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Println(err)
		return
	}
	body, err := decad.New().LoftFromPoint(ctx, r3.NewVec(0, 0, 0), s, s.Profiles()[0])
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	mesh, err := body.Tessellate(ctx, units.Millimeters(0.01))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("%.0f mm³; boundary %t; volume %t\n",
		volume.Value.Base(), mesh.BoundaryVerified(), mesh.VolumeVerified())
	// Output: 32 mm³; boundary true; volume true
}
