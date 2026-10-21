package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithPullDirection checks that a part can be pulled out of a mold in one
// direction. A face that points against the pull is an undercut, which would
// trap the part. Use it on molded and cast parts, with the direction the mold
// opens.
func Example_verify_pull_direction() {
	// A 100 mm cube is a simple part with no undercuts when pulled straight
	// up (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	if _, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(100), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Pull straight up (+Z). Every side wall is parallel to the pull, so the
	// part slides out cleanly.
	report, err := doc.Verify(context.Background(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("pull up: %s, undercut faces: %d\n", report.Status, len(report.Bodies[0].Undercut.Faces))

	// Pull at an angle instead. Now two faces point against the pull and
	// would catch on the mold.
	report, err = doc.Verify(context.Background(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("pull at an angle: %s, undercut faces: %d\n", report.Status, len(report.Bodies[0].Undercut.Faces))
	// Output:
	// pull up: Sound, undercut faces: 0
	// pull at an angle: Violating, undercut faces: 2
}
