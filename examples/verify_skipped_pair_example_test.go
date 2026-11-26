package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Some bodies share space on purpose, such as a bolt threaded through its nut,
// or you prove a pair another way, such as with Document.ContactPair.
// WithSkippedPair tells Verify not to work on that pair at all, which saves
// the cost of a proof you would ignore. Verify then says nothing about the
// pair, and Report.SkippedPairs lists it so the report shows what it did not
// check. Every other pair is still checked.
func Example_verify_skipped_pair() {
	// Three 10 x 10 x 10 mm blocks. The first two overlap on purpose; the
	// third overlaps the second by mistake. Each has its own sketch (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	doc := decad.New()
	var blocks []*decad.Body
	for _, x0 := range []float64{0, 5, 12} {
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(x0, 0, x0+10, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		block, err := decad.Extrude(doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along}).Do(context.Background())
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		blocks = append(blocks, block)
	}

	report, err := doc.Verify(context.Background(), decad.WithSkippedPair(blocks[0], blocks[1]))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("skipped pairs: %d\n", len(report.SkippedPairs))
	for _, i := range report.Interferences {
		fmt.Printf("overlap: %.0f mm^3, involves the third block: %v\n", i.Volume.Value.Base(), i.B == blocks[2])
	}
	fmt.Println("status:", report.Status)
	// Output:
	// skipped pairs: 1
	// overlap: 300 mm^3, involves the third block: true
	// status: Interfering
}
