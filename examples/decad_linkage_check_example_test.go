package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// VerifyLinkage checks a chain of joints against the rest of the document
// without moving anything. An upper arm swings 90° about the Z axis while
// its forearm, pinned at the elbow 48 mm out, swings −90° about its own
// joint, so the forearm keeps its orientation and rides the elbow's circle.
// Its top face reaches a wall at y = 38 when the shoulder stands at 30°, a
// third of the way through the drive. The check bisects the drive on a dyadic
// grid of its fraction down to the stated resolution, so the first collision
// it reports is the first grid point past the true contact, the same on every
// platform.
func Example_decad_linkageCheck() {
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
	upper, err := block(doc, 0, -14, 48, 14, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the upper arm: %s\n", err)
		return
	}
	// The forearm rides 2 mm above the upper arm, so the two never touch.
	forearm, err := block(doc, 48, -14, 96, 14, 12, 10)
	if err != nil {
		fmt.Printf("failed to build the forearm: %s\n", err)
		return
	}
	// The wall reaches past every cap, so no pair shares a face plane and an
	// overlap can be measured.
	wall, err := block(doc, -100, 38, 150, 58, -10, 50)
	if err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

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
	drive := decad.Drive{
		{Link: shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	}

	report, err := doc.VerifyLinkage(context.Background(), arm, drive, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the linkage: %s\n", err)
		return
	}
	names := map[*decad.Body]string{upper: "upper arm", forearm: "forearm", wall: "wall"}
	first := report.Collisions[0]
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at s = %.3f\n", names[first.A], names[first.B], first.At.Mag())
	// Output:
	// status: Interfering
	// first collision: forearm against wall at s = 0.336
}
