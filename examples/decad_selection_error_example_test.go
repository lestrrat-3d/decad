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

// A selector that resolves to the wrong count fails loudly, and the failure is
// a *decad.SelectionError: it wraps ErrNoMatch or ErrCardinality (so errors.Is
// still branches) and carries what an agent needs to repair the query without
// reconstructing it from source — the stable rendering, the expected/actual
// counts, and the running match count after each clause, which names the
// clause that emptied the set.
func Example_decad_selection_error() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The query asks for the vertical edges that are also circular. A box has
	// four vertical edges but none is circular, so the conjunction empties and
	// the assertion fails.
	_, err = decad.Edges(
		decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.Circular(),
	).Exactly(1).SelectEdges(box)
	if err == nil {
		fmt.Printf("expected a selection failure\n")
		return
	}

	// errors.Is still branches on the wrapped sentinel.
	fmt.Printf("is cardinality error: %t\n", errors.Is(err, decad.ErrCardinality))

	var se *decad.SelectionError
	if !errors.As(err, &se) {
		fmt.Printf("expected a *decad.SelectionError, got %T\n", err)
		return
	}
	fmt.Printf("kind: %s\n", se.Kind)
	fmt.Printf("query: %s\n", se.Query)
	fmt.Printf("expected %s, matched %d\n", se.Expected, se.Actual)
	for _, r := range se.Residuals {
		fmt.Printf("after %s: %d remaining\n", r.Predicate, r.Remaining)
	}
	// Output:
	// is cardinality error: true
	// kind: edge
	// query: edges(parallel_to(0,0,1), circular).exactly(1)
	// expected exactly 1, matched 0
	// after parallel_to(0,0,1): 4 remaining
	// after circular: 0 remaining
}
