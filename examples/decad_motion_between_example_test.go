package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A Between joins two arbitrary rigid poses along the screw motion between
// them: it rotates through the shorter arc about one axis while sliding
// uniformly along it. Here an arm 48 mm long and 28 mm wide makes a quarter
// turn about the Z axis while rising 20 mm, toward a wall whose near face is
// at y = 40. The rise leaves every y unchanged, so its far corner reaches the
// wall at the fraction (2/π)·atan(3/4) ≈ 0.40967 of the path. The parameter
// is that dimensionless fraction, bisected on a dyadic grid down to the
// stated resolution, so the first collision prints the same on every
// platform: 105/256, the first grid point past the contact.
func Example_decad_motionBetween() {
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
	// The wall reaches past the arm's caps at every height the rise visits,
	// so the two share no face plane and their overlap can be measured.
	if _, err := block(doc, -100, 40, 100, 60, -10, 50); err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

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
	to, err := turn.Then(rise)
	if err != nil {
		fmt.Printf("failed to compose the end pose: %s\n", err)
		return
	}

	screw := decad.Between{From: r3.Identity(), To: to}
	report, err := doc.VerifyMotion(context.Background(), []*decad.Body{arm}, screw, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the motion: %s\n", err)
		return
	}
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision at s = %.3f\n", report.Collisions[0].At.Mag())
	// Output:
	// status: Interfering
	// first collision at s = 0.410
}
