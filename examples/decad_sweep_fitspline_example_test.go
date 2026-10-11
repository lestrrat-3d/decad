package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Sweep carries the sketch's fitted spline through tangent arc, line, and arc
// path spans. The resulting walls keep their free-form surface identity.
func Example_decad_sweepFitSpline() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	start := s.CreatePoint(-1, -1)
	middle := s.CreatePoint(0, -1.25)
	end := s.CreatePoint(1, -1)
	if _, err := s.CreateFitSpline(start, middle, end); err != nil {
		fmt.Printf("failed to create fitted spline: %s\n", err)
		return
	}
	rightTop := s.CreatePoint(1, 1)
	leftTop := s.CreatePoint(-1, 1)
	s.CreateLine(end, rightTop)
	s.CreateLine(rightTop, leftTop)
	s.CreateLine(leftTop, start)
	profiles := s.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		fmt.Println("failed to close fitted spline profile")
		return
	}
	path, err := decad.NewPath(r3.NewVec(0, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		decad.LineTo{End: r3.NewVec(15, 0, 5)},
		decad.ArcThrough{Through: r3.NewVec(18, 1, 5), End: r3.NewVec(20, 5, 5)})
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}
	body, err := decad.New().Sweep(context.Background(), s, profiles[0], path)
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	mesh, err := body.Tessellate(context.Background(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	freeformWalls := 0
	for _, face := range body.Faces() {
		if face.Surface().Kind() == decad.KindNURBS {
			freeformWalls++
		}
	}
	fmt.Printf("solid: %t, free-form walls: %d, verified mesh: %t\n",
		body.IsSolid(), freeformWalls, mesh.BoundaryVerified() && mesh.VolumeVerified())
	// Output:
	// solid: true, free-form walls: 3, verified mesh: true
}
