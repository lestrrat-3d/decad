package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Fillet also rounds a revolve's swept meridian junctions: the latitude circle
// a corner of the revolved profile sweeps. The rewrite is the same tangent arc
// a prism fillet inserts, applied to the meridian, so the blend is a torus
// whose tube radius is the fillet radius, and the volume follows Pappus over
// the rewritten meridian.
func Example_decad_revolve_fillet() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A turned shaft's meridian: a radius-10 journal over x ∈ [0, 10] stepping
	// down to a radius-5 journal over x ∈ [10, 20], drawn above the x axis it
	// spins about.
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {10, 5}, {10, 10}, {0, 10}}
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
	shaft, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// The shoulder's inner corner is the shaft's one concave junction. A 2 mm
	// fillet there relieves the stress riser where the journals meet.
	const r = 2.0
	filleted, err := shaft.Fillet(context.Background(), decad.Edges(decad.Circular(), decad.Concave()), units.Millimeters(r))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}

	for _, f := range filleted.Faces() {
		if torus, ok := f.Surface().(decad.Torus); ok {
			fmt.Printf("blend torus: major %s, minor %s, centre x = %g\n", torus.Major, torus.Minor, torus.Center.X)
		}
	}

	vol, err := filleted.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	// Pappus over the rewritten meridian: the two journals' ∫ρ dA = 625, plus
	// the filled spandrel's area (1 − π/4)r² at radius 5 + k·r, with
	// k = (10 − 3π)/(12 − 3π) its centroid's offset from the corner.
	spandrel := (1 - math.Pi/4) * r * r
	k := (10 - 3*math.Pi) / (12 - 3*math.Pi)
	pappus := 2 * math.Pi * (625 + spandrel*(5+k*r))
	fmt.Printf("volume: %.3f mm^3 (%s), Pappus: %.3f mm^3\n", mm3, vol.Exactness, pappus)
	fmt.Printf("within bound: %v\n", math.Abs(mm3-pappus) <= vol.Bound.Mag()+1e-12*pappus)
	// Output:
	// blend torus: major 7 mm, minor 2 mm, centre x = 12
	// volume: 3956.368 mm^3 (Approximate), Pappus: 3956.368 mm^3
	// within bound: true
}
