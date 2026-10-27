package examples_test

import (
	"bytes"
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// IGES writes a body as the planar-faced solid B-reps of an IGES 5.3 file, one face for each triangle of its
// tessellation, and ReadIGES reads the file back to a mesh of the same vertices. Here a 20 x 10 x 6 mm block makes a
// round trip and keeps its volume.
func Example_iges_export() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to start a sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	var file bytes.Buffer
	if err := export.IGES(context.Background(), &file, block, units.Millimeters(0.01)); err != nil {
		fmt.Printf("failed to write IGES: %s\n", err)
		return
	}
	back, err := decad.ReadIGES(context.Background(), &file)
	if err != nil {
		fmt.Printf("failed to read IGES: %s\n", err)
		return
	}
	volume, err := back.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("closed: %t, volume: %.0f mm^3\n", back.Report().Closed, volume.Value.Base())
	// Output:
	// closed: true, volume: 1200 mm^3
}
