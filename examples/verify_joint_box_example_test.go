package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// VerifyJointBox checks a whole range of joint values at once, instead of one
// path through them. Use it to prove that a mechanism is safe at every pose its
// operators can reach, or to find a pose that is not. Here a crane's mast turns
// anywhere from 0 to 80 degrees and its boom slides anywhere from 0 to 30 mm,
// toward a wall.
func Example_verify_joint_box() {
	doc := decad.New()
	mast, err := extrudeBlock(doc, -5, -5, 5, 5, 0, 38)
	if err != nil {
		fmt.Printf("failed to build the mast: %s\n", err)
		return
	}
	// The boom sits 2 mm above the mast, so the two links never touch each
	// other and only the wall can cause a collision.
	boom, err := extrudeBlock(doc, 10, -5, 60, 5, 40, 10)
	if err != nil {
		fmt.Printf("failed to build the boom: %s\n", err)
		return
	}
	// The obstacle is a wall at y = 62.
	wall, err := extrudeBlock(doc, -100, 62, 150, 82, 30, 70)
	if err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	// Build the linkage: a mast that turns about Z, carrying a boom that
	// slides out along X.
	crane := decad.NewLinkage()
	turn, err := crane.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*decad.Body{mast})
	if err != nil {
		fmt.Printf("failed to attach the mast: %s\n", err)
		return
	}
	extend, err := turn.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boom})
	if err != nil {
		fmt.Printf("failed to attach the boom: %s\n", err)
		return
	}
	// The box is the allowed range of each joint. Every pose inside it is
	// checked, not just the ones along a path.
	box := decad.JointBox{
		{Link: turn, Min: units.Degrees(0), Max: units.Degrees(80)},
		{Link: extend, Min: units.Millimeters(0), Max: units.Millimeters(30)},
	}

	// WithResolution sets how small the search cells get.
	report, err := doc.VerifyJointBox(context.Background(), crane, box, decad.WithResolution(units.Scalar(1.0/16)))
	if err != nil {
		fmt.Printf("failed to check the joint box: %s\n", err)
		return
	}
	// The report gives the joint values of the first collision it found.
	names := map[*decad.Body]string{mast: "mast", boom: "boom", wall: "wall"}
	first := report.Collisions[0]
	theta, err := first.Configuration.Values[0].In(units.Degree)
	if err != nil {
		fmt.Printf("failed to read the mast angle: %s\n", err)
		return
	}
	d, err := first.Configuration.Values[1].In(units.Millimeter)
	if err != nil {
		fmt.Printf("failed to read the boom slide: %s\n", err)
		return
	}
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at θ = %.2f°, d = %.2f mm\n", names[first.A], names[first.B], theta, d)
	// Output:
	// status: Interfering
	// first collision: boom against wall at θ = 60.00°, d = 15.00 mm
}
