package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Web stands crossing ribs in a pocket. The block is 40 x 40 x 20 with a
// 30 x 30 pocket 15 deep. The rib lines are drawn on a plane 1 mm below the
// pocket floor, so each rib is buried in the floor, and run 2 mm into the
// pocket walls so the rib ends are buried in them too.
func Example_web() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()

	// block builds a prism from a rectangle on the plane at height z.
	block := func(x0, y0, x1, y1, z, height float64) (*decad.Body, error) {
		plane, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	solid, err := block(0, 0, 40, 40, 0, 20)
	if err != nil {
		fmt.Printf("failed to build block: %s\n", err)
		return
	}
	pocket, err := block(5, 5, 35, 35, 5, 15)
	if err != nil {
		fmt.Printf("failed to build pocket: %s\n", err)
		return
	}
	part, err := decad.Cut(ctx, solid, pocket)
	if err != nil {
		fmt.Printf("failed to cut pocket: %s\n", err)
		return
	}

	plane, err := w.CreateOffsetPlane(w.XY(), 4)
	if err != nil {
		fmt.Printf("failed to create rib plane: %s\n", err)
		return
	}
	ribs, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create rib sketch: %s\n", err)
		return
	}
	line := func(x0, y0, x1, y1 float64) *sketch.Line {
		a, b := ribs.CreatePoint(x0, y0), ribs.CreatePoint(x1, y1)
		ribs.Fix(a)
		ribs.Fix(b)
		return ribs.CreateLine(a, b)
	}
	lines := []*sketch.Line{line(3, 20, 37, 20), line(20, 3, 20, 37)}
	if _, err := ribs.Solve(ctx); err != nil {
		fmt.Printf("failed to solve rib sketch: %s\n", err)
		return
	}

	// Each rib is 2 mm thick and stands 11 mm up from the sketch plane, to
	// z = 15, which is 5 mm below the pocket's open top.
	out, err := decad.Web(ctx, part, ribs, lines, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to add web: %s\n", err)
		return
	}
	volume, err := out.Volume()
	if err != nil {
		fmt.Printf("failed to measure part: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// volume: 19660.0 mm^3
	// live bodies: 1
}
