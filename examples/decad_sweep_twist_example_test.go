package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A straight Sweep can rotate a centred polygon along its path. The true
// volume uses the rotating sections, while the mesh carries a certified
// approximation of the curved walls.
func Example_decad_sweepTwist() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	if err != nil {
		fmt.Println(err)
		return
	}
	body, err := decad.New().Sweep(context.Background(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(30)))
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	mesh, err := body.Tessellate(context.Background(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("volume: %s; mesh triangles: %d\n", volume.Value, len(mesh.Triangles()))
	fmt.Printf("boundary verified: %t; volume verified: %t\n",
		mesh.BoundaryVerified(), mesh.VolumeVerified())
	// Output:
	// volume: 20 mm^3; mesh triangles: 12
	// boundary verified: true; volume verified: true
}
