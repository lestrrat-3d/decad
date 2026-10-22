package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A document carries a gap tolerance: how far the evaluator may move a vertex
// of a faceted result, off the exact point it stands for, to keep that result
// from folding through itself. With no option the tolerance is 0.01 mm, far
// below what the eye can see. ExactOnly refuses any result that only such a
// move would keep, and WithGapTolerance sets the amount.
func Example_gap_tolerance() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The same peg in three documents: the default, one that tolerates up to
	// 0.05 mm, and one that keeps every result exact.
	slide, err := r3.Translation(r3.Vec{X: 5})
	if err != nil {
		fmt.Printf("failed to build a translation: %s\n", err)
		return
	}
	for _, tc := range []struct {
		name string
		opts []decad.DocumentOption
	}{
		{"default", nil},
		{"0.05 mm", []decad.DocumentOption{decad.WithGapTolerance(units.Millimeters(0.05))}},
		{"exact only", []decad.DocumentOption{decad.ExactOnly()}},
	} {
		doc := decad.New(tc.opts...)
		peg, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		moved, err := decad.Transform(context.Background(), peg, slide)
		if err != nil {
			fmt.Printf("failed to place: %s\n", err)
			return
		}
		centroid, err := moved.Centroid()
		if err != nil {
			fmt.Printf("failed to measure centroid: %s\n", err)
			return
		}
		fmt.Printf("%s: centroid x = %.1f\n", tc.name, centroid.Value.X)
	}

	// A tolerance that is not a non-negative length is refused when an
	// operation reads it.
	doc := decad.New(decad.WithGapTolerance(units.Millimeters(-1)))
	peg, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	_, err = decad.Transform(context.Background(), peg, slide)
	fmt.Printf("negative tolerance: %t\n", errors.Is(err, decad.ErrNegativeMagnitude))

	// Output:
	// default: centroid x = 10.0
	// 0.05 mm: centroid x = 10.0
	// exact only: centroid x = 10.0
	// negative tolerance: true
}
