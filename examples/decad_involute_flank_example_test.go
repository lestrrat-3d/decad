package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A real involute gear-tooth flank — not a synthetic hump — is a fit spline
// through points sampled along the involute curve, and it now extrudes
// (docs/spline-design.md §10 P4b): a Tier A free-form section builds as long
// as its chain proves ONE curvature sign along its whole length (§6.5). An
// involute flank is convex its entire run from base circle to tooth tip, so
// its chain clears that certificate and the section builds into a fully
// measured solid — the same certificate that refuses a section whose
// curvature changes sign partway along (see Example_decad_freeformExtrude's
// neighbours in extrude_freeform_test.go for a refused case).
//
// module 1, 17 teeth, 20 degree pressure angle are a gear dialog's own
// defaults.
func Example_decad_involuteFlank() {
	const (
		module        = 1.0
		toothNumber   = 17.0
		pressureAngle = 20 * math.Pi / 180
		steps         = 15
	)
	pitchR := module * toothNumber / 2
	baseR := pitchR * math.Cos(pressureAngle)
	tipR := (module*toothNumber + 2*module) / 2

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	o := s.CreatePoint(0, 0)
	s.Fix(o)

	// Sample the involute from the base circle to the tooth tip, mirrored
	// across +X, and fit a spline through the samples.
	flank := make([]*sketch.Point, 0, steps)
	for i := range steps {
		r := baseR + (tipR-baseR)*float64(i)/float64(steps-1)
		alpha := math.Acos(baseR / r)
		t := math.Tan(alpha)
		x := baseR * (math.Cos(t) + t*math.Sin(t))
		y := baseR * (math.Sin(t) - t*math.Cos(t))
		p := s.CreatePoint(x, -y)
		s.Fix(p)
		flank = append(flank, p)
	}
	if _, err := s.CreateFitSpline(flank...); err != nil {
		fmt.Printf("failed to create fit spline: %s\n", err)
		return
	}
	// Close the flank back to the origin so it bounds a single region.
	s.CreateLine(flank[len(flank)-1], o)
	s.CreateLine(o, flank[0])

	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}

	// Round every printed measurement: the involute samples above are built
	// from expressions of the shape a*b + c, which Go permits the compiler
	// to fuse into a single rounding step on some architectures and not
	// others. The unrounded value can therefore differ in its last digits
	// between hosts; a rounded value does not.
	fmt.Printf("volume: %.4f mm^3 (%s)\n", vol.Value.Mag(), vol.Exactness)
	fmt.Printf("faces: %d\n", len(body.Faces()))

	// Find the free-form wall among the built faces: the one whose surface
	// is a NURBSSurface. Both closing lines' walls stay Plane.
	var freeformFaces int
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindNURBS {
			freeformFaces++
		}
	}
	fmt.Printf("free-form (NURBS) walls: %d\n", freeformFaces)
	// Output:
	// volume: 28.3866 mm^3 (Approximate)
	// faces: 5
	// free-form (NURBS) walls: 1
}
