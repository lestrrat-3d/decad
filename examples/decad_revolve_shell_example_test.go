package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shell also hollows a partial revolve once both of its angular caps are
// removed. The wall is the meridian's own offset swept over the same angle,
// and a meridian edge lying on the axis grows no wall: a half cylinder shelled
// this way is a trough with closed ends, not a tube around its own axis.
func Example_decad_revolve_shell() {
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
	half, err := doc.Revolve(s, s.Profiles()[0], axis, decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// A half turn leaves both angular caps in the sketch plane, so one query
	// names the pair. Removing them opens the trough along its length.
	caps := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).Exactly(2)
	trough, err := half.Shell(context.Background(), caps, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	for _, f := range trough.Faces() {
		if c, ok := f.Surface().(decad.Cylinder); ok {
			fmt.Printf("cylinder radius %s\n", c.Radius)
		}
	}
	vol, err := trough.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	// Pappus over a half turn: the wall region is the half-section less its
	// cavity x ∈ [2, 38], ρ ∈ [0, 8], so ∫ρ dA = 100/2·40 − 64/2·36 = 848.
	pappus := math.Pi * 848
	fmt.Printf("volume: %.3f mm^3, Pappus: %.3f mm^3\n", mm3, pappus)
	fmt.Printf("within bound: %v\n", math.Abs(mm3-pappus) <= vol.Bound.Mag()+1e-12*pappus)
	// Output:
	// cylinder radius 10 mm
	// cylinder radius 8 mm
	// volume: 2664.071 mm^3, Pappus: 2664.071 mm^3
	// within bound: true
}
