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

// VerifyJointBox varies a closed loop at its driver alone. The crank-rocker
// four-bar of Example_decad_linkageLoop turns its crank anywhere in
// [0°, 90°], and a gate above the follower slides down anywhere in [0, 10]
// mm. The box lists the crank and the gate; the coupler's and the follower's
// angles over every cell are read from sketch's certified enclosure of the
// loop, and the far end of the follower's range over a cell is charged into
// the travel the cell's centre must clear. The follower rocks up toward the
// gate and back while the gate comes down, so the colliding region's edge
// turns back along the crank.
func Example_decad_jointBoxLoop() {
	prism := func(doc *decad.Document, pts [][2]float64, z0, h float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), z0)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		corners := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			corners[i] = s.CreatePoint(p[0], p[1])
			s.Fix(corners[i])
		}
		for i := range corners {
			s.CreateLine(corners[i], corners[(i+1)%len(corners)])
		}
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	}
	// bar is a link 8 mm wide from p to q, 8 mm tall from z0.
	bar := func(doc *decad.Document, p, q [2]float64, z0 float64) (*decad.Body, error) {
		dx, dy := q[0]-p[0], q[1]-p[1]
		n := math.Hypot(dx, dy)
		nx, ny := -dy/n*4, dx/n*4
		return prism(doc, [][2]float64{
			{p[0] - nx, p[1] - ny}, {q[0] - nx, q[1] - ny}, {q[0] + nx, q[1] + ny}, {p[0] + nx, p[1] + ny},
		}, z0, 8)
	}

	// The coupler pin B where the loop closes at the zero pose, with the
	// crank along +X: 80 mm from the crank pin (30, 0) and 70 mm from the
	// follower's pivot (100, 0), above the ground line.
	beta := math.Acos((70*70 + 70*70 - 80*80) / (2.0 * 70 * 70))
	b := [2]float64{100 + 70*math.Cos(math.Pi-beta), 70 * math.Sin(math.Pi-beta)}

	doc := decad.New()
	// Each link sits in its own layer, so no two links ever meet.
	crankBody, err := bar(doc, [2]float64{0, 0}, [2]float64{30, 0}, 0)
	if err != nil {
		fmt.Printf("failed to build the crank: %s\n", err)
		return
	}
	couplerBody, err := bar(doc, [2]float64{30, 0}, b, 10)
	if err != nil {
		fmt.Printf("failed to build the coupler: %s\n", err)
		return
	}
	followerBody, err := bar(doc, [2]float64{100, 0}, b, 20)
	if err != nil {
		fmt.Printf("failed to build the follower: %s\n", err)
		return
	}
	// The gate's underside stands at y = 72 and comes down by its slide.
	gateBody, err := prism(doc, [][2]float64{{-50, 72}, {150, 72}, {150, 82}, {-50, 82}}, 19, 10)
	if err != nil {
		fmt.Printf("failed to build the gate: %s\n", err)
		return
	}

	z := r3.NewVec(0, 0, 1)
	fourBar := decad.NewLinkage()
	crank, err := fourBar.Ground().Revolute(r3.Vec{}, z, []*decad.Body{crankBody})
	if err != nil {
		fmt.Printf("failed to attach the crank: %s\n", err)
		return
	}
	coupler, err := crank.Revolute(r3.NewVec(30, 0, 0), z, []*decad.Body{couplerBody})
	if err != nil {
		fmt.Printf("failed to attach the coupler: %s\n", err)
		return
	}
	follower, err := fourBar.Ground().Revolute(r3.NewVec(100, 0, 0), z, []*decad.Body{followerBody})
	if err != nil {
		fmt.Printf("failed to attach the follower: %s\n", err)
		return
	}
	if _, err := fourBar.Close(coupler, follower, r3.NewVec(b[0], b[1], 0), z); err != nil {
		fmt.Printf("failed to close the loop: %s\n", err)
		return
	}

	gate, err := fourBar.Ground().Prismatic(r3.NewVec(0, -1, 0), []*decad.Body{gateBody})
	if err != nil {
		fmt.Printf("failed to attach the gate: %s\n", err)
		return
	}

	box := decad.JointBox{
		{Link: crank, Min: units.Degrees(0), Max: units.Degrees(90)},
		{Link: gate, Min: units.Millimeters(0), Max: units.Millimeters(10)},
	}
	report, err := doc.VerifyJointBox(context.Background(), fourBar, box, decad.WithResolution(units.Scalar(1.0/16)))
	if err != nil {
		fmt.Printf("failed to check the joint box: %s\n", err)
		return
	}
	names := map[*decad.Body]string{crankBody: "crank", couplerBody: "coupler", followerBody: "follower", gateBody: "gate"}
	first := report.Collisions[0]
	theta, err := first.Configuration.Values[0].In(units.Degree)
	if err != nil {
		fmt.Printf("failed to read the crank angle: %s\n", err)
		return
	}
	d, err := first.Configuration.Values[3].In(units.Millimeter)
	if err != nil {
		fmt.Printf("failed to read the gate slide: %s\n", err)
		return
	}
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at crank %.2f°, gate %.2f mm\n", names[first.A], names[first.B], theta, d)
	// Output:
	// status: Interfering
	// first collision: follower against gate at crank 45.00°, gate 5.00 mm
}
