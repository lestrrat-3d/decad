package examples_test

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// A FitSplineSeg profile's Area, Centroid and SecondMoments now answer
// (docs/spline-design.md Table F): the recorded fit points are sketch's own
// defining data for the natural-cubic interpolant, taken exactly, so the
// boundary integral is an exact rational — reported Exact when that rational
// is representable in the returned unit, Approximate with a one-rounding
// bound otherwise (§3). decad never re-runs sketch's interpolation solve; it
// consumes the solved interpolant sketch already computed.
//
// A fit-spline section still cannot EXTRUDE — that build-time capability
// (increment P4) remains staged and unaffected by this.
func Example_decad_fitSplineMoments() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// An open fit spline through three points — a shallow hump — closed by
	// a straight chord back to its start.
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(5, 4)
	end := s.CreatePoint(10, 0)
	if _, err := s.CreateFitSpline(start, mid, end); err != nil {
		fmt.Printf("failed to create fit spline: %s\n", err)
		return
	}
	s.CreateLine(end, start)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			prof = p
			break
		}
	}
	rec, _, err := decad.RecordProfile(s, prof)
	if err != nil {
		fmt.Printf("failed to record profile: %s\n", err)
		return
	}

	area, err := rec.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	centroid, err := rec.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}

	fmt.Printf("area: %s (%s)\n", area.Value, area.Exactness)
	fmt.Printf("centroid: %v\n", centroid.Value)
	// Output:
	// area: 25 mm^2 (Approximate)
	// centroid: {5 1.5542857142857143 0}
}
