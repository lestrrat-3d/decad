package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithMitredJoins sweeps a straight-sided profile along a LineTo polyline
// whose joins are corners, and WithSectionScale tapers the section span by
// span (docs/sweep-design.md §16). This builds a tree-support branch as one
// solid: a hexagon of radius 1.5 mm that leans over and narrows to 1 mm.
// Every wall is an exact plane, so the volume is the nearest float to an
// exact rational, and the branch tessellates without chording.
func Example_decad_sweepMitred() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	hexagon, err := s.CreatePolygon(0, 0, 6, 1.5)
	if err != nil {
		fmt.Printf("failed to create polygon: %s\n", err)
		return
	}
	s.Fix(hexagon.Center)
	for _, v := range hexagon.Vertices {
		s.Fix(v)
	}
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}

	// The path starts in the sketch plane, rises straight, then leans 45°.
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 6)},
		decad.LineTo{End: r3.NewVec(4, 0, 10)},
	)
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}

	// One factor per span: the section at the end of span k is factors[k]
	// times the authored one, here radius 1.25 mm and then 1 mm.
	doc := decad.New()
	branch, err := doc.Sweep(context.Background(), s, s.Profiles()[0], path,
		decad.WithMitredJoins(),
		decad.WithSectionScale(units.Scalar(1.25/1.5), units.Scalar(1/1.5)),
	)
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}

	volume, err := branch.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mesh, err := branch.Tessellate(context.Background(), units.Millimeters(0.01),
		decad.WithVerification(decad.VerifyBoundary))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	fmt.Printf("faces: %d\n", len(branch.Faces()))
	fmt.Printf("volume: %.6f mm³\n", volume.Value.Base())
	fmt.Printf("triangles: %d\n", len(mesh.Triangles()))
	fmt.Printf("boundary verified: %v\n", mesh.BoundaryVerified())
	fmt.Printf("verify status: %s\n", report.Status)
	// Output:
	// faces: 14
	// volume: 48.198588 mm³
	// triangles: 32
	// boundary verified: true
	// verify status: Sound
}
