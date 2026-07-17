package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A model-input or evaluator refusal wraps a sentinel from decad's error
// vocabulary, so a caller — an agent, especially — branches with errors.Is and
// repairs its next call instead of parsing a message. (Two failures keep their
// own cause instead: Verify can return the context's cancellation directly, and
// STL/OBJ preserve the writer's error.) Here two real refusals drive a repair: a
// staged tapered extrude (ErrUnsupported) is retried straight, and an
// over-asserted selector (ErrCardinality) is relaxed to a count the body can
// meet.
func Example_decad_error_recovery() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// This evaluator refuses a nonzero taper before it records any step, so
	// the failed call leaves the document untouched: the profile is still
	// current and the repair is to drop the taper.
	doc := decad.New()
	prof := s.Profiles()[0]
	body, err := doc.Extrude(s, prof,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithTaper(units.Degrees(5)))
	if errors.Is(err, decad.ErrUnsupported) {
		fmt.Println("taper unsupported: retrying without it")
		body, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	}
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Over-assert a selector: the plate has six planar faces, not 99. The
	// assertion failing is ErrCardinality (not ErrNoMatch — the query did
	// match), so the repair relaxes the count.
	_, err = decad.Faces(decad.Planar()).Exactly(99).SelectFaces(body)
	if !errors.Is(err, decad.ErrCardinality) {
		fmt.Printf("unexpected selector outcome: %v\n", err)
		return
	}
	fmt.Println("wrong count asserted: relaxing to AtLeast(1)")
	faces, err := decad.Faces(decad.Planar()).AtLeast(1).SelectFaces(body)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("planar faces: %d\n", len(faces))
	// Output:
	// taper unsupported: retrying without it
	// wrong count asserted: relaxing to AtLeast(1)
	// planar faces: 6
}
