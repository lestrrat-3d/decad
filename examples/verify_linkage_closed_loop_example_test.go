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

// VerifyLinkage also checks closed loops, not just open chains. A closed loop
// is a mechanism whose links connect back to the ground, such as a four-bar
// linkage. Build it as a tree of links, then close it with Close, which adds the
// last pin. The Drive moves only the driving joint (the crank); the other
// joints follow from the loop's geometry. Here the crank turns and the
// follower rocks up into a wall.
func Example_verify_linkage_closed_loop() {
	// prism extrudes a polygon of fixed corner points into a body h mm tall,
	// starting at height z0. It is used for the wall and, through bar, for the
	// links.
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
		return decad.Extrude(context.Background(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	}
	// bar is a link between two pins: a rectangle 8 mm wide that runs from
	// point p to point q, 8 mm tall, starting at height z0.
	bar := func(doc *decad.Document, p, q [2]float64, z0 float64) (*decad.Body, error) {
		dx, dy := q[0]-p[0], q[1]-p[1]
		n := math.Hypot(dx, dy)
		nx, ny := -dy/n*4, dx/n*4
		return prism(doc, [][2]float64{
			{p[0] - nx, p[1] - ny}, {q[0] - nx, q[1] - ny}, {q[0] + nx, q[1] + ny}, {p[0] + nx, p[1] + ny},
		}, z0, 8)
	}

	// Find the pin B where the coupler meets the follower when the crank lies
	// along +X. It is 80 mm from the crank pin (30, 0) and 70 mm from the
	// follower's pivot (100, 0), above the ground line. The law of cosines
	// gives its position.
	beta := math.Acos((70*70 + 70*70 - 80*80) / (2.0 * 70 * 70))
	b := [2]float64{100 + 70*math.Cos(math.Pi-beta), 70 * math.Sin(math.Pi-beta)}

	doc := decad.New()
	// The four-bar has a crank (30 mm), a coupler (80 mm) and a follower
	// (70 mm), with the fixed ground 100 mm between the crank and follower
	// pivots. Each link sits in its own layer, so no two links ever touch each
	// other and only the wall can cause a collision.
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
	wall, err := prism(doc, [][2]float64{{-50, 68.5}, {150, 68.5}, {150, 78.5}, {-50, 78.5}}, 19, 10)
	if err != nil {
		fmt.Printf("failed to build the wall: %s\n", err)
		return
	}

	// Build the loop as a tree: the crank and follower each hinge to the
	// ground, and the coupler hinges to the crank.
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
	// Close the loop with a pin joining the coupler to the follower at B.
	if _, err := fourBar.Close(coupler, follower, r3.NewVec(b[0], b[1], 0), z); err != nil {
		fmt.Printf("failed to close the loop: %s\n", err)
		return
	}

	// Only the crank is driven, from 0 to 90 degrees.
	drive := decad.Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}}
	report, err := doc.VerifyLinkage(context.Background(), fourBar, drive, decad.WithResolution(units.Scalar(1.0/256)))
	if err != nil {
		fmt.Printf("failed to check the linkage: %s\n", err)
		return
	}
	// Collisions are reported as a fraction s of the drive (0 start, 1 end).
	names := map[*decad.Body]string{crankBody: "crank", couplerBody: "coupler", followerBody: "follower", wall: "wall"}
	first := report.Collisions[0]
	fmt.Printf("status: %s\n", report.Status)
	fmt.Printf("first collision: %s against %s at s = %.3f\n", names[first.A], names[first.B], first.At.Mag())
	// Output:
	// status: Interfering
	// first collision: follower against wall at s = 0.141
}
