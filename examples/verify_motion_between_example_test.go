package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Between describes a motion by its start and end poses, instead of by a joint.
// The body travels from one pose to the other along a screw motion: it turns
// and slides at the same time. Use it for a move you know only by its two ends,
// such as a part being lifted and swung into place. VerifyMotion then checks
// the whole path for collisions.
func Example_verify_motion_between() {
	doc := decad.New()

	// The moving part is an arm 48 mm long and 28 mm wide, lying along +X.
	arm, err := extrudeBlock(doc, 0, -14, 48, 14, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the arm: %s\n", err)
		return
	}

	// The obstacle is a wall with its near face at y = 40, tall enough to
	// cover every height the arm reaches while it rises.
	if _, err := extrudeBlock(doc, -100, 40, 100, 60, -10, 50); err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	// The end pose is a quarter turn about Z followed by a 20 mm rise. The
	// start pose is the identity: the arm where it already is.
	turn, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Degrees(90))
	if err != nil {
		fmt.Printf("failed to build the turn: %s\n", err)
		return
	}
	rise, err := r3.Translation(r3.NewVec(0, 0, 20))
	if err != nil {
		fmt.Printf("failed to build the rise: %s\n", err)
		return
	}
	end, err := turn.Then(rise)
	if err != nil {
		fmt.Printf("failed to compose the end pose: %s\n", err)
		return
	}

	// The motion parameter is the fraction of the way from start (0) to end
	// (1). The search stops at a resolution of 1/256 of the path.
	screw := decad.Between{From: r3.Identity(), To: end}
	report, err := doc.VerifyMotion(context.Background(), []*decad.Body{arm}, screw,
		decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the motion: %s\n", err)
		return
	}
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision at %.0f%% of the way\n", 100*report.Collisions[0].At.Mag())
	// Output:
	// status: Interfering
	// first collision at 41% of the way
}
