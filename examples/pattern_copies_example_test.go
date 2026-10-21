package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// PatternCopies repeats a body in a pattern, as separate bodies, and leaves
// the original in place. Use it for bolt circles, rows of teeth and similar
// repeated features. A CircularPattern spaces copies evenly about an axis. A
// LinearPattern spaces them along a direction.
func Example_pattern_copies() {
	// A 4 x 2 x 5 mm block whose center is 20 mm out along +X.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(18, -1, 22, 1)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Four blocks about the Z axis, one every quarter turn. The count
	// includes the original, so the call returns three new bodies.
	ring, err := decad.PatternCopies(context.Background(), block,
		decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 4})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	fmt.Printf("circular: %d new bodies\n", len(ring))

	// Three blocks in a row along +Y, 10 mm apart, starting from the first
	// copy. Again the count includes the starting body.
	row, err := decad.PatternCopies(context.Background(), ring[0],
		decad.LinearPattern{Dir: r3.NewVec(0, 1, 0), Step: units.Millimeters(10), Count: 3})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	fmt.Printf("linear: %d new bodies\n", len(row))
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// circular: 3 new bodies
	// linear: 2 new bodies
	// live bodies: 6
}
