package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// RevolveChain spins an open sketch chain about an axis into a shell with no
// cap: one swept wall per recorded segment, and none of the two closing
// faces a revolved Profile would mint. A FULL revolution of a chain still
// leaves it open — its two free ends sweep two circles that nothing fills —
// unlike a full-turn Profile revolve, which closes.
func Example_decad_revolve_chain() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// An open two-segment walk clear of the axis: (10,0)-(10,5)-(15,8).
	a := s.CreatePoint(10, 0)
	b := s.CreatePoint(10, 5)
	c := s.CreatePoint(15, 8)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own v axis, stated in plane coordinates.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

	doc := decad.New()
	body, err := doc.RevolveChain(s, s.Chains()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve chain: %s\n", err)
		return
	}

	free, err := decad.Edges(decad.Free()).SelectEdges(body)
	if err != nil {
		fmt.Printf("failed to select free edges: %s\n", err)
		return
	}

	fmt.Printf("sheet: %v, solid: %v, faces: %d\n", body.Kind() == decad.BodySheet, body.IsSolid(), len(body.Faces()))
	fmt.Printf("free edges: %d\n", len(free))

	_, err = body.Volume()
	fmt.Printf("volume error: %v\n", err)
	// Output:
	// sheet: true, solid: false, faces: 2
	// free edges: 2
	// volume error: decad: body is not a solid
}
