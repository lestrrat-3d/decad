package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Stitch is the one operation that turns a boundary into a solid
// (docs/surface-design.md §6): weld a surface-extruded wall set to a patch
// at each open end, and the box closes into a BodySolid with an exact
// volume — every coordinate in it is stated rather than computed, so every
// welded rim edge and its matching patch edge hold the same float64 values
// at a zero bound, which is exactly what Table J admits.
func Example_decad_stitch() {
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
	bottom, err := doc.Patch(context.Background(), s, prof)
	if err != nil {
		fmt.Printf("failed to patch the bottom: %s\n", err)
		return
	}
	topPatch, err := doc.Patch(context.Background(), ts, ts.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch the top: %s\n", err)
		return
	}

	box, err := decad.Stitch(walls, bottom, topPatch)
	if err != nil {
		fmt.Printf("failed to stitch: %s\n", err)
		return
	}

	vol, err := box.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}

	fmt.Printf("kind is solid: %v\n", box.Kind() == decad.BodySolid)
	fmt.Printf("is solid: %v\n", box.IsSolid())
	fmt.Printf("faces: %d\n", len(box.Faces()))
	fmt.Printf("edges: %d\n", len(box.Edges()))
	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	// Output:
	// kind is solid: true
	// is solid: true
	// faces: 6
	// edges: 12
	// volume: 60000 mm^3 (Exact)
}
