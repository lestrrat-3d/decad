package decad_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This is the canonical decad loop: build and solve a 2D profile in sketch,
// turn it into a body with a feature verb, then gate the live document on
// Verify's one trustworthy bit. It lives in the root package so godoc and
// pkg.go.dev render it right beside the package doc; the `go doc` CLI skips
// test files, so there it is the doc.go Quickstart section that a caller reads
// instead. The examples directory holds the fuller, feature-specific cases.
func Example_decad_quickstart() {
	// 1. A solved 2D profile: a 100 x 60 plate on the XY plane.
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

	// 2. Model: extrude the profile 10 mm along the sketch normal. Every
	// failure wraps a sentinel from the package error vocabulary, so a caller
	// branches on it with errors.Is instead of parsing the message.
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		if errors.Is(err, decad.ErrUnsupported) {
			fmt.Println("this evaluator cannot build that combination yet")
		}
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// 3. Verify and gate: Trustworthy() is true only for a Sound report —
	// every body a proven solid and nothing left undecided.
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	fmt.Printf("status: %s, trustworthy: %v\n", report.Status, report.Trustworthy())
	// Output:
	// volume: 60000 mm^3 (Exact)
	// status: Sound, trustworthy: true
}
