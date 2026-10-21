package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// VerifyLinkage checks a linkage, a set of bodies joined by hinges and slides,
// against the rest of the document. A Drive says how far each joint moves.
// Use it to prove that a mechanism, such as a robot arm, can run through its
// whole range without hitting anything, before you build it. Here a two-joint
// arm swings toward a wall.
func Example_verify_linkage_chain() {
	doc := decad.New()
	upper, err := extrudeBlock(doc, 0, -14, 48, 14, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the upper arm: %s\n", err)
		return
	}
	// The forearm sits 2 mm above the upper arm, so the two links never touch
	// each other and only the wall can cause a collision.
	forearm, err := extrudeBlock(doc, 48, -14, 96, 14, 12, 10)
	if err != nil {
		fmt.Printf("failed to build the forearm: %s\n", err)
		return
	}
	// The obstacle is a wall at y = 38. It is taller than the arms, so a
	// collision is a real overlap and not two faces merely touching.
	wall, err := extrudeBlock(doc, -100, 38, 150, 58, -10, 50)
	if err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	// Build the linkage. The shoulder is a hinge about Z at the origin that
	// carries the upper arm. The elbow is a hinge 48 mm out that carries the
	// forearm and moves with the shoulder.
	arm := decad.NewLinkage()
	shoulder, err := arm.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*decad.Body{upper})
	if err != nil {
		fmt.Printf("failed to attach the upper arm: %s\n", err)
		return
	}
	elbow, err := shoulder.Revolute(r3.NewVec(48, 0, 0), r3.NewVec(0, 0, 1), []*decad.Body{forearm})
	if err != nil {
		fmt.Printf("failed to attach the forearm: %s\n", err)
		return
	}
	// Drive both joints at once: the shoulder swings 0 to 90 degrees while the
	// elbow swings the opposite way, so the forearm keeps pointing the same
	// way as the arm rides around.
	drive := decad.Drive{
		{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	}

	// Check the whole drive. WithResolution sets how finely to search it.
	report, err := doc.VerifyLinkage(context.Background(), arm, drive, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the linkage: %s\n", err)
		return
	}
	// The collision is reported as a fraction s of the drive: 0 is the start
	// pose and 1 is the end pose. Names make the report readable.
	names := map[*decad.Body]string{upper: "upper arm", forearm: "forearm", wall: "wall"}
	first := report.Collisions[0]
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at s = %.3f\n", names[first.A], names[first.B], first.At.Mag())
	// Output:
	// status: Interfering
	// first collision: forearm against wall at s = 0.336
}
