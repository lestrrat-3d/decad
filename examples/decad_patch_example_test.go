package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Document.Patch builds a single planar face directly from a recorded
// profile, on the sketch plane's own frame (docs/surface-design.md §5.1). The
// result is a one-face sheet body: Volume and Centroid answer ErrNotSolid,
// while Area and Bounds answer over the one face.
func Example_decad_patch() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	patch, err := doc.Patch(context.Background(), s, s.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch: %s\n", err)
		return
	}

	free, err := decad.Edges(decad.Free()).SelectEdges(patch)
	if err != nil {
		fmt.Printf("failed to select free edges: %s\n", err)
		return
	}
	area, err := patch.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	_, err = patch.Volume()

	fmt.Printf("is sheet: %v\n", patch.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(patch.Faces()))
	fmt.Printf("free edges: %d\n", len(free))
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("volume error: %v\n", err)
	// Output:
	// is sheet: true
	// faces: 1
	// free edges: 4
	// area: 6000 mm^2
	// volume error: decad: body is not a solid
}
