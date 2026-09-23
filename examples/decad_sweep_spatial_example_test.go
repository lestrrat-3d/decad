package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Example_decad_sweep_spatial builds a square duct whose path turns first in
// the XZ plane and then in the XY plane. The integer ArcThrough points form
// quarter circles, and the line between them meets both arcs tangentially.
func Example_decad_sweep_spatial() {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}

	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{
			Through: r3.NewVec(2, 0, 4),
			End:     r3.NewVec(5, 0, 5),
		},
		decad.LineTo{End: r3.NewVec(15, 0, 5)},
		decad.ArcThrough{
			Through: r3.NewVec(18, 1, 5),
			End:     r3.NewVec(20, 5, 5),
		},
	)
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Sweep(context.Background(), s, s.Profiles()[0], path)
	if err != nil {
		fmt.Printf("failed to sweep profile: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	area, err := body.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	centroid, err := body.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	bounds, err := body.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify sweep: %s\n", err)
		return
	}

	fmt.Printf("solid: %v, spans: %d, faces: %d\n", body.IsSolid(), len(path.Segments()), len(body.Faces()))
	fmt.Printf("volume: %.3f mm^3\n", volume.Value.Base())
	fmt.Printf("area: %.3f mm^2\n", area.Value.Base())
	fmt.Printf("centroid: {%.3f %.3f %.3f}\n", centroid.Value.X, centroid.Value.Y, centroid.Value.Z)
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("verified: %v\n", report.Passed())
	// Output:
	// solid: true, spans: 3, faces: 14
	// volume: 102.832 mm^3
	// area: 213.664 mm^2
	// centroid: {10.000 0.542 4.458}
	// bounds: {-1 -1 0} .. {21 5 6}
	// verified: true
}
