package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Three exact copies of one curved section make an analytic straight solid.
func Example_decad_loftCurvedSections() {
	ctx := context.Background()
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i, z := range []float64{0, 5, 10} {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), z)
			if err != nil {
				fmt.Printf("plane failed: %s\n", err)
				return
			}
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			fmt.Printf("sketch failed: %s\n", err)
			return
		}
		point := func(radius, angle float64) *sketch.Point {
			p := s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
			s.Fix(p)
			return p
		}
		origin := s.CreatePoint(0, 0)
		s.Fix(origin)
		r0, r1, r2 := point(4, -0.3), point(6, -0.24), point(8, -0.2)
		l0, l1, l2 := point(4, 0.3), point(6, 0.24), point(8, 0.2)
		if _, err := s.CreateFitSpline(r0, r1, r2); err != nil {
			fmt.Printf("first flank failed: %s\n", err)
			return
		}
		s.CreateArc(origin, r2, l2)
		if _, err := s.CreateFitSpline(l2, l1, l0); err != nil {
			fmt.Printf("second flank failed: %s\n", err)
			return
		}
		s.CreateArc(origin, l0, r0)
		if _, err := s.Solve(ctx); err != nil {
			fmt.Printf("solve failed: %s\n", err)
			return
		}
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Outer) == 4 {
				sections[i] = decad.LoftSection{Sketch: s, Profile: profile}
				break
			}
		}
		if sections[i].Profile == nil {
			fmt.Println("curved section missing")
			return
		}
	}
	body, err := decad.New().LoftSections(ctx, sections[:]...)
	if err != nil {
		fmt.Printf("loft failed: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("volume failed: %s\n", err)
		return
	}
	mesh, err := body.Tessellate(ctx, units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Printf("mesh failed: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f mm³, faces: %d, verified: %t\n",
		volume.Value.Base(), len(body.Faces()), mesh.VolumeVerified())
	// Output:
	// volume: 616.7 mm³, faces: 6, verified: true
}
