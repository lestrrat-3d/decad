package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WebPath turns a run of sketch lines and arcs into one stiffening rib, such as
// a wall that follows a slot-shaped outline. Each leg is thickened as Rib and
// RibArc thicken one, and a small overrun lets the legs overlap where they meet.
func Example_web_path() {
	// A 40 x 40 x 5 mm plate (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The rib's centre line on a plane 1 mm below the plate's top: a line from
	// (10, 12) to (20, 12), a half circle of radius 5 about (20, 17), and a line
	// back to (10, 22).
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
	fixed := func(x, y float64) *sketch.Point {
		p := rs.CreatePoint(x, y)
		rs.Fix(p)
		return p
	}
	a, b, c, d, centre := fixed(10, 12), fixed(20, 12), fixed(20, 22), fixed(10, 22), fixed(20, 17)
	path := []sketch.Entity{rs.CreateLine(a, b), rs.CreateArc(centre, b, c), rs.CreateLine(c, d)}
	if _, err := rs.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// A 2 mm wall 9 mm tall from the sketch plane, so it stands 8 mm above the
	// plate, with a 0.5 mm overrun at each leg's ends.
	ribbed, err := decad.WebPath(context.Background(), plate, rs, path, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(9), Dir: decad.Along}, decad.WithRibOverrun(units.Millimeters(0.5)))
	if err != nil {
		fmt.Printf("failed to add the rib: %s\n", err)
		return
	}
	volume, err := ribbed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8587 mm^3
}
