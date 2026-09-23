package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithSurfaceResult on Sweep builds the swept wall set alone and omits both
// section caps, publishing a sheet body — Kind() == BodySheet — instead of a
// solid (docs/surface-design.md §4). This example uses a one-span straight
// path, which reduces to the same prism build a surface-extruded Extrude
// takes, so the sheet's own construction proves it sound (docs/sweep-design.md
// Table D row D1): Volume and Centroid answer ErrNotSolid for a sheet, proven
// sound or not; Area and Bounds still answer, over the walls alone.
// Tessellating a sweep sheet is staged for a later increment
// (docs/sweep-design.md Table D row D2), for every Sweep body, solid or
// sheet.
func Example_decad_surfaceSweep() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}

	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}

	doc := decad.New()
	sheet, err := doc.Sweep(context.Background(), s, s.Profiles()[0], path, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}

	free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	if err != nil {
		fmt.Printf("failed to select free edges: %s\n", err)
		return
	}
	area, err := sheet.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	_, volErr := sheet.Volume()

	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	_, tessErr := sheet.Tessellate(context.Background(), units.Millimeters(0.1))

	fmt.Printf("is sheet: %v\n", sheet.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(sheet.Faces()))
	fmt.Printf("free edges: %d\n", len(free))
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("volume error: %v\n", volErr)
	fmt.Printf("verify status: %s\n", report.Status)
	fmt.Printf("tessellate is unsupported: %v\n", errors.Is(tessErr, decad.ErrUnsupported))
	// Output:
	// is sheet: true
	// faces: 4
	// free edges: 8
	// area: 3200 mm^2
	// volume error: decad: body is not a solid
	// verify status: Sound
	// tessellate is unsupported: true
}
