package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A Drive can pass through waypoints. Give each joint a From value, any Via
// values, and a To value, and the motion visits them in order. Use it for a
// mechanism that does not move in one smooth sweep, such as a crane that
// lifts, swings, and then lowers. Here an arm lifts 20 mm, swings 90 degrees
// over a post, and lowers onto a landing block.
func Example_verify_linkage_waypoints() {
	doc := decad.New()
	hub, err := extrudeBlock(doc, -5, -5, 5, 5, -20, 8)
	if err != nil {
		fmt.Printf("failed to build the hub: %s\n", err)
		return
	}
	arm, err := extrudeBlock(doc, 0, -5, 50, 5, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the arm: %s\n", err)
		return
	}
	// The post stands in the swing's path, 5 mm below the lifted arm, so the
	// lifted arm clears it. Without the lift it would hit the post.
	post, err := extrudeBlock(doc, 17, 17, 25, 25, -10, 25)
	if err != nil {
		fmt.Printf("failed to build the post: %s\n", err)
		return
	}
	landing, err := extrudeBlock(doc, -20, 30, 20, 40, -10, 15)
	if err != nil {
		fmt.Printf("failed to build the landing block: %s\n", err)
		return
	}

	crane := decad.NewLinkage()
	// Build the linkage: a hub that turns about Z, carrying an arm that slides
	// up and down along Z.
	z := r3.NewVec(0, 0, 1)
	swing, err := crane.Ground().Revolute(r3.Vec{}, z, []*decad.Body{hub})
	if err != nil {
		fmt.Printf("failed to attach the hub: %s\n", err)
		return
	}
	lift, err := swing.Prismatic(z, []*decad.Body{arm})
	if err != nil {
		fmt.Printf("failed to attach the arm: %s\n", err)
		return
	}
	// Each joint lists its waypoints. The swing stays at 0 degrees while the
	// arm lifts, turns to 90 degrees while the arm is up, then stays there
	// while the arm lowers. The three segments take equal shares of the drive.
	drive := decad.Drive{
		{Link: swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(0), units.Degrees(90)}, To: units.Degrees(90)},
		{Link: lift, From: units.Millimeters(0), Via: []units.Value{units.Millimeters(20), units.Millimeters(20)}, To: units.Millimeters(0)},
	}

	// Check the whole motion in one report.
	report, err := doc.VerifyLinkage(context.Background(), crane, drive, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the linkage: %s\n", err)
		return
	}
	// The fraction s tells which segment the collision is in: segment j of 3
	// covers s from j/3 to (j+1)/3. Here the lowering arm hits the landing block.
	names := map[*decad.Body]string{hub: "hub", arm: "arm", post: "post", landing: "landing block"}
	segments := []string{"lift", "swing", "lower"}
	first := report.Collisions[0]
	s := first.At.Mag()
	j := min(int(math.Floor(s*3)), 2)
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at s = %.3f\n", names[first.A], names[first.B], s)
	fmt.Printf("segment: %s, %.0f%% of the way through it\n", segments[j], 100*(3*s-float64(j)))
	// Output:
	// status: Interfering
	// first collision: arm against landing block at s = 0.918
	// segment: lower, 75% of the way through it
}
