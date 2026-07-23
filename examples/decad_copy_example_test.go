package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A body can be copied without consuming it. Placed retires its receiver, so
// modelling a part once and instancing it several times — a bolt pattern, a
// reused cutting tool — would otherwise mean rebuilding the feature chain per
// instance. Duplicate and PlacedCopy leave the source live: the source is
// depended on, never consumed. PlacedCopy re-evaluates the payload under the
// motion, so each instance's centroid moves by its own placement while the
// master part stays put.
func Example_decad_copy() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	// A 10 x 10 peg footprint at the origin.
	peg := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(peg.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	// Step 0: the master peg, 4 mm tall — modelled once.
	master, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude peg: %s\n", err)
		return
	}

	// Place three instances at 40 mm spacing along x. PlacedCopyContext can
	// cancel any faceted rebuild and leaves the same master live, so one feature
	// chain feeds the whole pattern.
	for i := 1; i <= 3; i++ {
		shift, err := r3.Translation(r3.NewVec(float64(40*i), 0, 0))
		if err != nil {
			fmt.Printf("failed to build placement: %s\n", err)
			return
		}
		inst, err := master.PlacedCopyContext(context.Background(), shift)
		if err != nil {
			fmt.Printf("failed to place instance %d: %s\n", i, err)
			return
		}
		c, err := inst.Centroid()
		if err != nil {
			fmt.Printf("failed to measure instance %d: %s\n", i, err)
			return
		}
		fmt.Printf("instance %d centroid x: %v\n", i, c.Value.X)
	}

	// The master never moved and is still live alongside its three copies.
	mc, err := master.Centroid()
	if err != nil {
		fmt.Printf("failed to measure master: %s\n", err)
		return
	}
	fmt.Printf("master centroid x: %v\n", mc.Value.X)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// instance 1 centroid x: 45
	// instance 2 centroid x: 85
	// instance 3 centroid x: 125
	// master centroid x: 5
	// live bodies: 4
}
