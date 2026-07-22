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

// A boolean that fails on the geometry returns a typed *decad.BooleanError. It
// names the operation and the operands, and carries a branchable Code that
// separates the three caller moves: BooleanEmpty (the model is sound and the
// operation asked for nothing), BooleanUnsupportedContact (the model is valid
// but its contact is past this evaluator's reach), and BooleanEvaluatorFailure
// (a bug to file). The error still wraps the §12 sentinel errors.Is branches on,
// so errors.Is(err, decad.ErrUnsupported) holds for an unclassifiable contact
// and errors.Is(err, decad.ErrBooleanFailed) for an empty result.
func Example_decad_boolean_error() {
	// Two 10 mm cubes stacked face on face in one document: a valid model whose
	// coplanar contact the evaluator cannot classify from the tessellated chords.
	doc := decad.New()
	lower, err := boolErrCube(doc, r3.Vec{})
	if err != nil {
		fmt.Printf("failed to build lower cube: %s\n", err)
		return
	}
	upper, err := boolErrCube(doc, r3.Vec{Z: 10})
	if err != nil {
		fmt.Printf("failed to build upper cube: %s\n", err)
		return
	}
	if _, err := decad.Union(lower, upper); err != nil {
		reportBooleanError("union", err)
	}

	// Two disjoint cubes: their intersection encloses no volume — a normal
	// geometric outcome, not a malformed input.
	doc2 := decad.New()
	left, err := boolErrCube(doc2, r3.Vec{})
	if err != nil {
		fmt.Printf("failed to build left cube: %s\n", err)
		return
	}
	right, err := boolErrCube(doc2, r3.Vec{X: 40})
	if err != nil {
		fmt.Printf("failed to build right cube: %s\n", err)
		return
	}
	if _, err := decad.Intersect(left, right); err != nil {
		reportBooleanError("intersect", err)
	}

	// Output:
	// union: BooleanUnsupportedContact, operands 2, is-unsupported true, is-degenerate false
	// intersect: BooleanEmpty, operands 2, is-boolean-failed true
}

// reportBooleanError branches on the typed *decad.BooleanError, printing only
// its stable structured fields so the outcome is deterministic.
func reportBooleanError(what string, err error) {
	var be *decad.BooleanError
	if !errors.As(err, &be) {
		fmt.Printf("%s: unexpected error: %s\n", what, err)
		return
	}
	switch be.Code {
	case decad.BooleanUnsupportedContact:
		fmt.Printf("%s: BooleanUnsupportedContact, operands %d, is-unsupported %v, is-degenerate %v\n",
			what, len(be.Inputs),
			errors.Is(err, decad.ErrUnsupported), errors.Is(err, decad.ErrDegenerate))
	case decad.BooleanEmpty:
		fmt.Printf("%s: BooleanEmpty, operands %d, is-boolean-failed %v\n",
			what, len(be.Inputs), errors.Is(err, decad.ErrBooleanFailed))
	case decad.BooleanEvaluatorFailure:
		fmt.Printf("%s: BooleanEvaluatorFailure (a bug to file), operands %d\n", what, len(be.Inputs))
	}
}

// boolErrCube extrudes a 10 mm cube into doc, translated by offset.
func boolErrCube(doc *decad.Document, offset r3.Vec) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, err
	}
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		return nil, err
	}
	if offset == (r3.Vec{}) {
		return body, nil
	}
	move, err := r3.Translation(offset)
	if err != nil {
		return nil, err
	}
	return body.Placed(move)
}
