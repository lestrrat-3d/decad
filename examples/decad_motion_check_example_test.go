package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// VerifyMotion checks a rigid swing against the rest of the document without
// moving anything. An arm 48 mm long and 28 mm wide swings 90° about the Z
// axis toward a wall whose near face is at y = 40; its far corner, 50 mm from
// the axis, reaches the wall at atan(3/4) ≈ 36.87°. The check bisects the path
// on a dyadic grid of 90° down to the stated resolution, so the first
// collision it reports is bracketed to one grid step above the true contact,
// and it prints the same on every platform.
func Example_decad_motionCheck() {
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
	arm, err := block(doc, 0, -14, 48, 14, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the arm: %s\n", err)
		return
	}
	// The wall reaches past the arm's caps, so the two share no face plane
	// and their overlap can be measured.
	if _, err := block(doc, -100, 40, 100, 60, -10, 40); err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	swing := decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}
	report, err := doc.VerifyMotion(context.Background(), []*decad.Body{arm}, swing, decad.WithResolution(units.Degrees(0.25)))
	if err != nil {
		fmt.Printf("failed to check the motion: %s\n", err)
		return
	}
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision at %.2f deg\n", report.Collisions[0].At.Mag())
	// Output:
	// status: Interfering
	// first collision at 36.91 deg
}
