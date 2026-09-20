package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Unstitch is Stitch's own inverse (docs/surface-design.md §6.5): stitch a
// box closed, then take it apart again into one single-face sheet per face.
// Every edge of every result is free, and the six faces' areas sum back to
// the same 15200 mm^2 the solid's own Area already reported.
func Example_decad_unstitch() {
	w := sketch.NewWorld()
	ctx := context.Background()

	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	prof := s.Profiles()[0]

	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create the top plane: %s\n", err)
		return
	}
	ts, err := w.CreateSketch(top)
	if err != nil {
		fmt.Printf("failed to create the top sketch: %s\n", err)
		return
	}
	topRect := ts.CreateRectangle(0, 0, 100, 60)
	ts.Fix(topRect.A)
	if _, err := ts.Solve(ctx); err != nil {
		fmt.Printf("failed to solve the top sketch: %s\n", err)
		return
	}

	doc := decad.New()
	walls, err := doc.Extrude(s, prof,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude the walls: %s\n", err)
		return
	}
	bottom, err := doc.Patch(s, prof)
	if err != nil {
		fmt.Printf("failed to patch the bottom: %s\n", err)
		return
	}
	topPatch, err := doc.Patch(ts, ts.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch the top: %s\n", err)
		return
	}

	box, err := decad.Stitch(walls, bottom, topPatch)
	if err != nil {
		fmt.Printf("failed to stitch: %s\n", err)
		return
	}

	sheets, err := box.Unstitch()
	if err != nil {
		fmt.Printf("failed to unstitch: %s\n", err)
		return
	}

	var areaSum float64
	freeEdges := 0
	for _, sheet := range sheets {
		a, err := sheet.Area()
		if err != nil {
			fmt.Printf("failed to measure a sheet's area: %s\n", err)
			return
		}
		areaSum += a.Value.Base()
		for _, e := range sheet.Edges() {
			if e.IsFree() {
				freeEdges++
			}
		}
	}

	fmt.Printf("sheets: %d\n", len(sheets))
	fmt.Printf("free edges: %d\n", freeEdges)
	fmt.Printf("area sum: %g mm^2\n", areaSum)
	fmt.Printf("document bodies: %d\n", len(doc.Bodies()))
	// Output:
	// sheets: 6
	// free edges: 24
	// area sum: 15200 mm^2
	// document bodies: 6
}
