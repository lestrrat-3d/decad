package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// VerifyMotion checks that a body can move along a path without hitting
// anything else in the document. Use it to prove that a hinge, a door or a
// swinging arm has room to move, before you build it. Nothing is moved: the
// check sweeps the path and reports the first collision it finds. Revolute
// describes a swing about an axis.
func Example_verify_motion_revolute() {
	doc := decad.New()

	// The moving part is an arm 48 mm long and 28 mm wide. It lies along +X
	// from the origin, which is where we will put the hinge.
	arm, err := extrudeBlock(doc, 0, -14, 48, 14, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the arm: %s\n", err)
		return
	}

	// The obstacle is a wall in the arm's way, with its near face at y = 40.
	// It is tall enough to reach past the arm's top and bottom, so the
	// two overlap properly rather than only touching along a face.
	if _, err := extrudeBlock(doc, -100, 40, 100, 60, -10, 40); err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	// Swing the arm from 0 to 90 degrees about the Z axis through the origin.
	// WithResolution is how finely to search the path: a quarter degree.
	swing := decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}
	report, err := doc.VerifyMotion(context.Background(), []*decad.Body{arm}, swing,
		decad.WithResolution(units.Degrees(0.25)))
	if err != nil {
		fmt.Printf("failed to check the motion: %s\n", err)
		return
	}

	// The arm's far corner is 50 mm from the hinge, so it reaches the wall at
	// 36.87 degrees. The report gives the first angle the search found
	// colliding, which is within one step of that.
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision at %.2f degrees\n", report.Collisions[0].At.Mag())
	// Output:
	// status: Interfering
	// first collision at 36.91 degrees
}
