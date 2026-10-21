package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A boolean that fails returns a *decad.BooleanError. Its Code tells your
// program what to do next: BooleanEmpty means the model is fine but the result
// is nothing, BooleanUnsupportedContact means the bodies touch in a way the
// evaluator cannot handle yet, and BooleanEvaluatorFailure is a bug to report.
// Here two cubes that do not touch are intersected, which is an empty result.
func Example_boolean_errors() {
	// We need two bodies that do not touch, so that intersecting them has nothing
	// to return. Draw two squares 40 mm apart in one sketch.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	left := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(left.A)
	right := s.CreateRectangle(40, 0, 50, 10)
	s.Fix(right.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	var cubes []*decad.Body
	for _, profile := range s.Profiles() {
		cube, err := decad.Extrude(context.Background(), doc, s, profile,
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		cubes = append(cubes, cube)
	}

	_, err = decad.Intersect(context.Background(), cubes[0], cubes[1])

	// errors.As finds the typed error, and Code says which case it is.
	var boolErr *decad.BooleanError
	if !errors.As(err, &boolErr) {
		fmt.Printf("unexpected error: %v\n", err)
		return
	}
	fmt.Printf("code is BooleanEmpty: %v\n", boolErr.Code == decad.BooleanEmpty)
	// The error also wraps a general sentinel, for code that only needs to
	// know that a boolean failed.
	fmt.Printf("is ErrBooleanFailed: %v\n", errors.Is(err, decad.ErrBooleanFailed))
	// Output:
	// code is BooleanEmpty: true
	// is ErrBooleanFailed: true
}
