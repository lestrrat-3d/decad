package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Example_decad_loft_placed builds one tapered tooth as a Loft, then
// PlacedCopy's a mirror of it into a herringbone pair — the target case
// docs/loft-design.md names for PR 2a. The tooth occupies x in [0, 40]; the
// mirror, reflected across the x=0 plane, occupies x in [-40, 0], so the
// pair meets at the shared root.
//
// A reflection is still a rigid motion, so the mirrored tooth's volume
// matches the source's, but every held vertex now carries the payload's own
// proven displacement delta (docs/loft-design.md §5/§12 PR 2a): a general
// rigid motion rounds inside its own products and sums. The placement
// therefore demotes the source's Exact Centroid and Bounds to Approximate.
// The source's Volume is already Approximate before any placement — the
// tooth's true volume is 16000/3 mm^3, which float64 cannot represent — and
// the placement only widens that bound, from 3.03e-13 to 8.74e-10 mm^3.
func Example_decad_loft_placed() {
	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create the bottom sketch: %s\n", err)
		return
	}
	bottom := s0.CreateRectangle(0, -10, 40, 10)
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
	topRect := s1.CreateRectangle(5, -5, 35, 5)
	s1.Fix(topRect.A)
	if _, err := s1.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the top sketch: %s\n", err)
		return
	}

	doc := decad.New()
	tooth, err := doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft the tooth: %s\n", err)
		return
	}

	// Mirror across the plane x=0: origin at the world origin, spanned by Y
	// and Z, so its normal U x V is the X axis.
	mirrorFrame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		fmt.Printf("failed to build the mirror frame: %s\n", err)
		return
	}
	mirror, err := r3.Reflection(mirrorFrame)
	if err != nil {
		fmt.Printf("failed to build the reflection: %s\n", err)
		return
	}
	other, err := tooth.PlacedCopy(mirror)
	if err != nil {
		fmt.Printf("failed to mirror the tooth: %s\n", err)
		return
	}

	toothVol, err := tooth.Volume()
	if err != nil {
		fmt.Printf("failed to measure the tooth's volume: %s\n", err)
		return
	}
	otherVol, err := other.Volume()
	if err != nil {
		fmt.Printf("failed to measure the mirrored tooth's volume: %s\n", err)
		return
	}
	otherBounds, err := other.Bounds()
	if err != nil {
		fmt.Printf("failed to measure the mirrored tooth's bounds: %s\n", err)
		return
	}

	fmt.Printf("tooth volume: %s (%s)\n", toothVol.Value, toothVol.Exactness)
	fmt.Printf("mirrored volume: %s (%s)\n", otherVol.Value, otherVol.Exactness)
	fmt.Printf("mirrored bounds: %v .. %v\n", otherBounds.Min, otherBounds.Max)
	fmt.Printf("bodies in the document: %d\n", len(doc.Bodies()))
	// Output:
	// tooth volume: 5333.333333333333 mm^3 (Approximate)
	// mirrored volume: 5333.333333333333 mm^3 (Approximate)
	// mirrored bounds: {-40 -10 0} .. {0 10 10}
	// bodies in the document: 2
}
