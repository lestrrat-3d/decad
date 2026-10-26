package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ChamferAngle bevels a right-angled edge from a distance and an angle instead
// of two distances, the way a drawing dimensions a chamfer such as "3 mm at
// 30 degrees". The reference face takes the distance, and the angle is
// measured from that face.
func Example_chamfer_angle() {
	// Make a plate with a sharp vertical corner to bevel (see Example_extrude
	// for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The bevel leaves the wall facing +X 4 mm from the corner at (40, 20),
	// at 45 degrees to that wall, so it meets the other wall 4 mm from it.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(1)
	wall := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	beveled, err := decad.ChamferAngle(context.Background(), plate, edge, units.Millimeters(4), wall, units.Degrees(45))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate is 8000 mm^3. The bevel removes a 4 x 4 triangle, 10 mm tall.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 7920 mm^3
}
