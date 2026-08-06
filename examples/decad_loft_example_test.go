package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Loft rules a solid between two profiles recorded on distinct planes: a
// 40 mm square on the sketch plane and a 20 mm square on a plane offset
// 10 mm along its normal, both centred on the axis. The result is a
// pyramidal frustum, whose volume h/3*(A0+A1+sqrt(A0*A1)) = 28000/3 mm^3 is
// not exactly representable in binary — the evaluator reports it
// Approximate with a proven bound, the same honesty every other analytic
// measurement carries.
func Example_decad_loft() {
	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create the bottom sketch: %s\n", err)
		return
	}
	bottom := s0.CreateRectangle(-20, -20, 20, 20)
	s0.Fix(bottom.A)
	if _, err := s0.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the bottom sketch: %s\n", err)
		return
	}

	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create the top plane: %s\n", err)
		return
	}
	s1, err := w.CreateSketch(top)
	if err != nil {
		fmt.Printf("failed to create the top sketch: %s\n", err)
		return
	}
	topRect := s1.CreateRectangle(-10, -10, 10, 10)
	s1.Fix(topRect.A)
	if _, err := s1.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the top sketch: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	bounds, err := body.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}

	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("faces: %d\n", len(body.Faces()))
	// Output:
	// volume: 9333.333333333334 mm^3 (Approximate)
	// bounds: {-20 -20 0} .. {20 20 10}
	// faces: 10
}
