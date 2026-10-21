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

// export.ThreeMF writes a body in the 3MF format, a modern triangle-mesh format
// for 3D printing that carries units and is read by most slicers. Use it in
// place of STL when your printer software supports it.
func Example_export_3mf() {
	// Make a body to export: sketch a rectangle and extrude it into a box
	// (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 2, 3)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Write to any io.Writer. A 3MF file is a zip archive, so the output is
	// binary data.
	var out bytes.Buffer
	if err := export.ThreeMF(context.Background(), &out, box, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	// A zip archive starts with the two bytes "PK".
	fmt.Printf("is a zip archive: %v\n", bytes.HasPrefix(out.Bytes(), []byte("PK")))
	// Output:
	// is a zip archive: true
}
