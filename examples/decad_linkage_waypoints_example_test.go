package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A drive can pass through waypoints. An arm lifts 20 mm, swings 90° over a
// post, then lowers onto its landing place: three segments, written as each
// joint's From, Via values and To. The check covers the whole motion in one
// report over one fraction s, each segment taking an equal share of it, so
// segment j of n covers s ∈ [j/n, (j+1)/n]. The landing block's top sits
// 5 mm above the arm's resting underside, so the lowering arm meets it at
// s = 11/12, and the first collision is the first grid point past that.
func Example_decad_linkageWaypoints() {
	block := func(doc *decad.Document, x0, y0, x1, y1, z0, h float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), z0)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	}

	doc := decad.New()
	hub, err := block(doc, -5, -5, 5, 5, -20, 8)
	if err != nil {
		fmt.Printf("failed to build the hub: %s\n", err)
		return
	}
	arm, err := block(doc, 0, -5, 50, 5, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the arm: %s\n", err)
		return
	}
	// The post stands in the swing's path, 5 mm below the lifted arm.
	post, err := block(doc, 17, 17, 25, 25, -10, 25)
	if err != nil {
		fmt.Printf("failed to build the post: %s\n", err)
		return
	}
	landing, err := block(doc, -20, 30, 20, 40, -10, 15)
	if err != nil {
		fmt.Printf("failed to build the landing block: %s\n", err)
		return
	}

	crane := decad.NewLinkage()
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
	drive := decad.Drive{
		{Link: swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(0), units.Degrees(90)}, To: units.Degrees(90)},
		{Link: lift, From: units.Millimeters(0), Via: []units.Value{units.Millimeters(20), units.Millimeters(20)}, To: units.Millimeters(0)},
	}

	report, err := doc.VerifyLinkage(context.Background(), crane, drive, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the linkage: %s\n", err)
		return
	}
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
