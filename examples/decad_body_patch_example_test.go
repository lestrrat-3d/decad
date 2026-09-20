package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Body.Patch fills one or more closed chains of a body's own free edges
// (docs/surface-design.md §5.2). A surface-extruded tube has no caps and two
// congruent free-edge rims, and no edge predicate can tell them apart —
// Edges(Free()) always resolves to both — so the contract fills every
// closed chain the selection partitions into, proving each independently,
// which is what lets one call cap both ends of the tube at once.
func Example_decad_body_patch() {
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

	doc := decad.New()
	tube, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude the tube: %s\n", err)
		return
	}

	rims, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(tube)
	if err != nil {
		fmt.Printf("failed to select the tube's rims: %s\n", err)
		return
	}

	capped, err := tube.Patch(decad.Edges(decad.Free()).Exactly(len(rims)))
	if err != nil {
		fmt.Printf("failed to patch the tube: %s\n", err)
		return
	}

	area, err := capped.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}

	_, err = decad.Edges(decad.Free()).SelectEdges(capped)

	fmt.Printf("kind is sheet: %v\n", capped.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(capped.Faces()))
	fmt.Printf("area: %s (%s)\n", area.Value, area.Exactness)
	fmt.Printf("no free edge remains: %v\n", err != nil)
	// Output:
	// kind is sheet: true
	// faces: 6
	// area: 15200 mm^2 (Exact)
	// no free edge remains: true
}
