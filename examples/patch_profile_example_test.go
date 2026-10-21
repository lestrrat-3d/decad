package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// PatchProfile turns a closed sketch profile into a single flat face, without
// giving it any thickness. Use it to make a cap, a lid or a reference
// surface that you will stitch to other sheets or thicken later.
func Example_patch_profile() {
	// Draw the outline of the face. A rectangle will do.
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

	// Fill the profile with a face. The result is a sheet with one face and
	// four free edges, its outline.
	doc := decad.New()
	face, err := decad.PatchProfile(context.Background(), doc, s, s.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch: %s\n", err)
		return
	}
	edges, err := decad.Edges(decad.Free()).SelectEdges(face)
	if err != nil {
		fmt.Printf("failed to select edges: %s\n", err)
		return
	}
	area, err := face.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	fmt.Printf("faces: %d, free edges: %d\n", len(face.Faces()), len(edges))
	fmt.Printf("area: %s\n", area.Value)
	// Output:
	// faces: 1, free edges: 4
	// area: 6000 mm^2
}
