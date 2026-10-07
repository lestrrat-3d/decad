package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// VerifyJointBox proves a whole box of joint values clear, or finds where it
// is not, without moving anything. A crane's mast turns anywhere in
// [0°, 80°] and its boom slides anywhere in [0, 30] mm; a wall stands at
// y = 62. The check cuts the box into cells, evaluates each cell's centre,
// and certifies a cell clear when the gap there exceeds how far any point
// can travel within the cell; a cell it cannot certify is halved. The cells
// are dyadic fractions of each range, so the first collision it reports
// sits at the same joint values on every platform.
func Example_decad_jointBox() {
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
	mast, err := block(doc, -5, -5, 5, 5, 0, 38)
	if err != nil {
		fmt.Printf("failed to build the mast: %s\n", err)
		return
	}
	// The boom rides 2 mm above the mast, so the two never touch.
	boom, err := block(doc, 10, -5, 60, 5, 40, 10)
	if err != nil {
		fmt.Printf("failed to build the boom: %s\n", err)
		return
	}
	wall, err := block(doc, -100, 62, 150, 82, 30, 70)
	if err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

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
	box := decad.JointBox{
		{Link: turn, Min: units.Degrees(0), Max: units.Degrees(80)},
		{Link: extend, Min: units.Millimeters(0), Max: units.Millimeters(30)},
	}

	report, err := doc.VerifyJointBox(context.Background(), crane, box, decad.WithResolution(units.Scalar(1.0/16)))
	if err != nil {
		fmt.Printf("failed to check the joint box: %s\n", err)
		return
	}
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
