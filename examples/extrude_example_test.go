package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Extrude turns a closed sketch profile into a solid by sweeping it along the
// sketch normal. It is the usual way to start a part: sketch the outline,
// extrude it, then measure or modify the resulting body.
func Example_extrude() {
	// Sketch a 100 x 60 mm rectangle on the XY plane. Fix pins one corner
	// so the sketch solver has a single answer.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Extrude the sketch's one profile 10 mm along its normal (+Z). The
	// document owns every body it creates.
	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Measurements come with an exactness label. A plain prism is exact.
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("volume: %s (%s)\n", volume.Value, volume.Exactness)
	// Output:
	// solid: true, faces: 6
	// volume: 60000 mm^3 (Exact)
}
