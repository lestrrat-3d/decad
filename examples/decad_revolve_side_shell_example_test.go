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

// Shell on a revolve can remove one of its generated side faces: removing the
// far end disk of a solid cylinder hollows it into a cup. The wall follows the
// kept skin and the kept end, and the meridian edge on the axis grows no wall,
// so the cup's floor meets the axis instead of leaving a tube along it.
func Example_decad_revolve_side_shell() {
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

	// Remove the disk at x = 40, the one planar face facing +x.
	cup, err := rod.Shell(context.Background(), decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0))), units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	vol, err := cup.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	// The rod less its cavity x ∈ [2, 40], ρ ≤ 8: π·(10²·40 − 8²·38).
	want := math.Pi * (100*40 - 64*38)
	fmt.Printf("shells: %d\n", len(cup.Shells()))
	fmt.Printf("volume: %.3f mm^3, closed form: %.3f mm^3\n", mm3, want)
	fmt.Printf("within bound: %v\n", math.Abs(mm3-want) <= vol.Bound.Mag()+1e-12*want)
	// Output:
	// shells: 1
	// volume: 4926.017 mm^3, closed form: 4926.017 mm^3
	// within bound: true
}
