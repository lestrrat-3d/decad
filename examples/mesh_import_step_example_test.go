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

// ReadSTEP loads the solids of a STEP file made of planar faces into a MeshBody:
// a part from another CAD system with flat faces, or a file decad wrote. Each
// face is cut into triangles and the file's unit is converted to millimetres.
// Faces on curved surfaces are refused. Here a block is written to STEP and read
// back.
func Example_mesh_import_step() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	var file bytes.Buffer
	if err := export.STEP(context.Background(), &file, block, units.Millimeters(0.1),
		export.WithSTEPName("block"), export.WithSTEPAuthor("decad"), export.WithSTEPOrganization("example")); err != nil {
		fmt.Printf("failed to write STEP: %s\n", err)
		return
	}

	mesh, err := decad.ReadSTEP(context.Background(), &file)
	if err != nil {
		fmt.Printf("failed to read STEP: %s\n", err)
		return
	}
	volume, err := mesh.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("closed: %t, volume: %.0f mm^3, triangles: %d\n", mesh.Report().Closed, volume.Value.Base(), mesh.Report().Triangles)
	// Output:
	// closed: true, volume: 6000 mm^3, triangles: 12
}
