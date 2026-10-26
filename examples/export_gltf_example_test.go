package examples_test

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// export.GLTF writes a body as a glTF 2.0 file, the format web viewers and game
// engines load. The file is one JSON document with its mesh embedded, and glTF
// measures in metres, so a millimetre model is scaled by 0.001.
func Example_export_gltf() {
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

	var out bytes.Buffer
	if err := export.GLTF(context.Background(), &out, box, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	// Read the box's size back from the position accessor's bounds.
	var file struct {
		Accessors []struct {
			Max []float64 `json:"max"`
		} `json:"accessors"`
	}
	if err := json.Unmarshal(out.Bytes(), &file); err != nil {
		fmt.Printf("failed to read the file: %s\n", err)
		return
	}
	size := file.Accessors[0].Max
	fmt.Printf("size in metres: %.3f x %.3f x %.3f\n", size[0], size[1], size[2])
	// Output:
	// size in metres: 0.002 x 0.003 x 0.004
}
