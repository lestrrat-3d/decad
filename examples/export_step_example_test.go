package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// export.STEP writes a body as a STEP file, the standard exchange format for
// CAD. Use it to hand a part to other CAD software, a CAM tool or a
// manufacturer. STEP keeps the exact surfaces (planes, cylinders and so on)
// instead of triangles, so the receiver can still edit and measure the part.
func Example_export_step() {
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

	// Write to any io.Writer. Here it is a buffer, but it could be a file.
	// The tolerance is how far curved surfaces may stray when a format needs
	// them approximated. The options fill in the file header; WithSTEPHeader
	// sets every field at once.
	var out bytes.Buffer
	err = export.STEP(context.Background(), &out, box, units.Millimeters(0.1),
		export.WithSTEPName("box.step"),
		export.WithSTEPAuthor("Example"),
		export.WithSTEPOrganization("Example"))
	if err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("is a STEP file: %v\n", strings.HasPrefix(out.String(), "ISO-10303-21;"))
	fmt.Printf("faces written: %d\n", strings.Count(out.String(), "=ADVANCED_FACE("))
	// Output:
	// is a STEP file: true
	// faces written: 6
}

// A solid with a cavity writes to STEP as a BREP_WITH_VOIDS: the outer shell and
// the cavity's shell, which a CAD system reads back as a hollow part. Here a
// 20 mm cube is shelled into a closed box with 3 mm walls.
func Example_export_step_cavity() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	cube, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	hollow, err := decad.Shell(context.Background(), cube, nil, units.Millimeters(3), decad.WithNoOpenings())
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	var out bytes.Buffer
	if err := export.STEP(context.Background(), &out, hollow, units.Millimeters(0.1),
		export.WithSTEPName("hollow"), export.WithSTEPAuthor("decad"), export.WithSTEPOrganization("example")); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("holds a cavity: %t\n", strings.Contains(out.String(), "BREP_WITH_VOIDS"))
	// Output:
	// holds a cavity: true
}
