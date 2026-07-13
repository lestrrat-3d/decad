package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// RecordProfile is the sketch seam: it converts a solved, closed profile into
// the structural records a Recipe Step carries — the region in decad's own
// plane-local types, and the sketch plane as three vectors. sketch has already
// proven the region closes; decad records the entities' own defining data and
// rejects anything it cannot record exactly.
func Example_decad_recordProfile() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate with a Ø20 hole.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(50, 30), 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The plate region: four lines outside, the circle bounding its hole.
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
			break
		}
	}

	rec, plane, err := decad.RecordProfile(s, prof)
	if err != nil {
		fmt.Printf("failed to record profile: %s\n", err)
		return
	}

	fmt.Printf("outer segments: %d\n", len(rec.Outer.Segments))
	fmt.Printf("holes: %d\n", len(rec.Holes))
	hole := rec.Holes[0].Segments[0].(decad.CircleSeg)
	fmt.Printf("hole: circle r=%s ccw=%v\n", hole.Radius, hole.CCW)
	fmt.Printf("plane normal: %v\n", plane.U.Cross(plane.V))
	// Output:
	// outer segments: 4
	// holes: 1
	// hole: circle r=10 mm ccw=false
	// plane normal: {0 0 1}
}
