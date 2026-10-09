package examples_test

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// A fit-spline profile's Area, Centroid and SecondMoments answer
// (docs/spline-design.md Table F): the recorded fit points are sketch's own
// defining data for the natural-cubic interpolant, taken exactly, so the
// boundary integral is an exact rational — reported Exact when that rational
// is representable in the returned unit, Approximate with a one-rounding
// bound otherwise (§3). decad never re-runs sketch's interpolation solve; it
// consumes the solved interpolant sketch already computed.
//
// A fit-spline section can also EXTRUDE now (docs/spline-design.md §10 P4b);
// this example stays scoped to the moments-only capability above.
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
	measured, err := decad.MeasureProfile(s, prof)
	if err != nil {
		fmt.Printf("failed to measure profile: %s\n", err)
		return
	}

	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	centroid, err := measured.Centroid()
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
