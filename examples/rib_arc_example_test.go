package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// RibArc stands a curved fin on a body: a rib that follows a circular arc,
// such as a stiffener around a boss. The arc is drawn in a sketch, and the rib
// is that arc thickened by the width you give and raised to the height you give.
func Example_rib_arc() {
	// A 60 x 60 x 5 mm plate (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 60, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Draw the rib's centre line on a plane 1 mm below the plate's top, so the
	// rib is rooted in the plate. It is a half circle of radius 15 about the
	// plate's middle, drawn counter-clockwise from (45, 30) to (15, 30).
	ws := sketch.NewWorld()
	plane, err := ws.CreateOffsetPlane(ws.XY(), 4)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	rs, err := ws.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre, start, end := rs.CreatePoint(30, 30), rs.CreatePoint(45, 30), rs.CreatePoint(15, 30)
	rs.Fix(centre)
	rs.Fix(start)
	rs.Fix(end)
	arc := rs.CreateArc(centre, start, end)
	if _, err := rs.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// A 2 mm thick rib, 9 mm tall from the sketch plane, so it stands 8 mm
	// above the plate's top face.
	ribbed, err := decad.RibArc(context.Background(), plate, rs, arc, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(9), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to add the rib: %s\n", err)
		return
	}
	volume, err := ribbed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 18000 mm^3. The half circle's band is pi x 15 x 2 mm^2
	// and stands 8 mm above the plate.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 18754 mm^3
}
