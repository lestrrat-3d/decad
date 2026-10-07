package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// PatternCopies repeats a body along a line or about an axis, leaving the
// receiver live. A prism moved in its own plane keeps its frame: the copy is
// the same record moved, so integer-millimetre steps and the quarter turns of
// a four-way pattern stay Exact.
func Example_decad_pattern() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	// A 4×2 block 20 mm out along +x.
	rect := s.CreateRectangle(18, -1, 22, 1)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	block, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Four instances about the z axis: the receiver plus three copies.
	copies, err := block.PatternCopies(ctx, decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 4})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	for i, c := range copies {
		centroid, err := c.Centroid()
		if err != nil {
			fmt.Printf("failed to measure copy %d: %s\n", i+1, err)
			return
		}
		fmt.Printf("copy %d centroid: (%g, %g, %g) %s\n", i+1, centroid.Value.X, centroid.Value.Y, centroid.Value.Z, centroid.Exactness)
	}

	// Two more along +y at 10 mm, patterned from the first copy.
	row, err := copies[0].PatternCopies(ctx, decad.LinearPattern{Dir: r3.NewVec(0, 1, 0), Step: units.Millimeters(10), Count: 3})
	if err != nil {
		fmt.Printf("failed to pattern the row: %s\n", err)
		return
	}
	last, err := row[len(row)-1].Centroid()
	if err != nil {
		fmt.Printf("failed to measure the row: %s\n", err)
		return
	}
	fmt.Printf("row end centroid: (%g, %g, %g)\n", last.Value.X, last.Value.Y, last.Value.Z)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// copy 1 centroid: (0, 20, 2.5) Exact
	// copy 2 centroid: (-20, 0, 2.5) Exact
	// copy 3 centroid: (0, -20, 2.5) Exact
	// row end centroid: (0, 40, 2.5)
	// live bodies: 6
}
