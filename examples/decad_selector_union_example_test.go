package examples_test

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Or adds a branch to a query: the query then selects every face that matches
// any one branch, and Exactly counts the faces of the union. A quarter-turn
// revolve has two angular caps in different planes, and its meridian's end
// faces are planar too, so no single list of predicates names just the caps.
// One branch per cap, by provenance, names both, and Shell removes them.
func Example_decad_selector_union() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A ring's meridian: 20 mm along the x axis it spins about, from radius 5
	// to radius 10.
	corners := [][2]float64{{0, 5}, {20, 5}, {20, 10}, {0, 10}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	ring, err := doc.Revolve(s, s.Profiles()[0], axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	planar, err := decad.Faces(decad.Planar()).SelectFaces(ring)
	if err != nil {
		fmt.Printf("failed to select planar faces: %s\n", err)
		return
	}
	fmt.Printf("planar faces: %d\n", len(planar))

	caps := decad.Faces(decad.FaceCreatedBy(decad.CapStart(ring))).
		Or(decad.FaceCreatedBy(decad.CapEnd(ring))).
		Exactly(2)
	// A wrong count fails loudly, and the error names the whole union.
	if _, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(ring))).
		Or(decad.FaceCreatedBy(decad.CapEnd(ring))).
		Exactly(1).SelectFaces(ring); err != nil {
		var se *decad.SelectionError
		if errors.As(err, &se) {
			fmt.Printf("exactly 1: matched %d\n", se.Actual)
		}
	}

	shelled, err := ring.Shell(context.Background(), caps, units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	vol, err := shelled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	// Pappus over a quarter turn: the wall is the meridian less its 1 mm
	// erosion z ∈ [1, 19], ρ ∈ [6, 9], so ∫ρ dA = 75/2·20 − 45/2·18 = 345.
	pappus := math.Pi / 2 * 345
	fmt.Printf("volume: %.3f mm^3, Pappus: %.3f mm^3\n", mm3, pappus)
	fmt.Printf("within bound: %v\n", math.Abs(mm3-pappus) <= vol.Bound.Mag()+1e-12*pappus)
	// Output:
	// planar faces: 4
	// exactly 1: matched 2
	// volume: 541.925 mm^3, Pappus: 541.925 mm^3
	// within bound: true
}
