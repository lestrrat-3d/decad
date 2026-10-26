package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Arrange lays several parts out side by side on a flat bed, such as a print
// plate. Each part keeps its orientation and moves by one translation, so it
// rests on the bed's surface and keeps a gap from its neighbours and from the
// bed's edge. Use it to place a batch of parts before checking they fit.
func Example_arrange() {
	doc := decad.New()
	var parts []*decad.Body
	for _, size := range [][2]float64{{20, 10}, {30, 15}, {20, 10}} {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(0, 0, size[0], size[1])
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		part, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		parts = append(parts, part)
	}

	// A 60 x 40 mm bed at the origin with a 2 mm gap. The deepest part, the
	// 30 x 15 one, starts the first row.
	placed, err := decad.Arrange(context.Background(), parts, decad.ArrangeBed{
		Origin: r3.Vec{},
		Width:  units.Millimeters(60),
		Depth:  units.Millimeters(40),
		Gap:    units.Millimeters(2),
	})
	if err != nil {
		fmt.Printf("failed to arrange: %s\n", err)
		return
	}
	for _, part := range placed {
		box, err := part.Bounds()
		if err != nil {
			fmt.Printf("failed to read bounds: %s\n", err)
			return
		}
		fmt.Printf("x %.0f..%.0f, y %.0f..%.0f\n", box.Min.X, box.Max.X, box.Min.Y, box.Max.Y)
	}
	// Output:
	// x 34..54, y 2..12
	// x 2..32, y 2..17
	// x 2..22, y 19..29
}
