package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shell with WithNoOpenings and a nil selector hollows a full revolve into a
// sealed body: every face stays, and the cavity is a second, void shell
// inside the first. A meridian edge lying on the axis grows no wall, so a
// solid cylinder becomes a closed tank with flat ends, not a tube.
func Example_decad_revolve_closed_shell() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A solid cylinder's half-section: 40 mm along the x axis it spins about,
	// 10 mm in radius, its bottom edge on the axis.
	corners := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {0, 10}}
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
	rod, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	tank, err := rod.Shell(context.Background(), nil, units.Millimeters(2), decad.WithNoOpenings())
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	for _, lump := range tank.Lumps() {
		for _, sh := range lump.Shells() {
			for _, f := range sh.Faces() {
				if c, ok := f.Surface().(decad.Cylinder); ok {
					fmt.Printf("void %v: cylinder radius %s\n", sh.IsVoid(), c.Radius)
				}
			}
		}
	}
	vol, err := tank.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	// Pappus over a full turn: the wall region is the half-section less its
	// cavity x ∈ [2, 38], ρ ∈ [0, 8], so ∫ρ dA = 100/2·40 − 64/2·36 = 848.
	pappus := 2 * math.Pi * 848
	fmt.Printf("volume: %.3f mm^3, Pappus: %.3f mm^3\n", mm3, pappus)
	fmt.Printf("within bound: %v\n", math.Abs(mm3-pappus) <= vol.Bound.Mag()+1e-12*pappus)
	// Output:
	// void false: cylinder radius 10 mm
	// void true: cylinder radius 8 mm
	// volume: 5328.141 mm^3, Pappus: 5328.141 mm^3
	// within bound: true
}
