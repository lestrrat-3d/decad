package main

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
)

// The crank-rocker's geometry, in millimetres (docs/linkage-check-design.md
// §15.10, scene 7): the ground pivot of the follower, the crank, coupler and
// follower lengths, the bars' half-width and the wall face the follower's top
// corner meets.
const (
	rockerGround   = 100.0
	rockerCrank    = 30.0
	rockerCoupler  = 80.0
	rockerFollower = 70.0
	rockerHalf     = 4.0
	rockerWallFace = 68.5
)

// rockerTheta4 is the follower's angle from the ground line at crank angle
// th2 (radians), on the branch with the coupler pin above the ground line:
// the four-bar's two-circle construction.
func rockerTheta4(th2 float64) float64 {
	g, r, l, f := rockerGround, rockerCrank, rockerCoupler, rockerFollower
	d := math.Sqrt(g*g + r*r - 2*g*r*math.Cos(th2))
	phi := math.Atan2(r*math.Sin(th2), r*math.Cos(th2)-g)
	beta := math.Acos((f*f + d*d - l*l) / (2 * f * d))
	return math.Mod(phi-beta+2*math.Pi, 2*math.Pi)
}

// rockerBar is a bar of half-width rockerHalf from p to q in the XY plane,
// z ∈ [z0, z0 + 8].
func rockerBar(ctx context.Context, doc *decad.Document, p, q [2]float64, z0 float64) (*decad.Body, error) {
	dx, dy := q[0]-p[0], q[1]-p[1]
	n := math.Hypot(dx, dy)
	nx, ny := -dy/n*rockerHalf, dx/n*rockerHalf
	body, err := extrudedPolygon(ctx, doc, [][2]float64{
		{p[0] - nx, p[1] - ny}, {q[0] - nx, q[1] - ny}, {q[0] + nx, q[1] + ny}, {p[0] + nx, p[1] + ny},
	}, 8)
	if err != nil {
		return nil, err
	}
	shift, err := r3.Translation(r3.Vec{Z: z0})
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, shift)
}

// crankRockerScene is docs/linkage-check-design.md §15.10's scene 7, a closed
// loop. The crank, x ∈ [0, 30], y ∈ [−4, 4], z ∈ [0, 8], turns about Z through
// the origin from 0° to 90°; the coupler, a bar from A = (30, 0) to the pin B
// the four-bar construction puts above the ground line, z ∈ [10, 18], hangs
// from the crank at A; the follower, a bar from O4 = (100, 0) to B,
// z ∈ [20, 28], turns about O4; and the coupler is closed onto the follower at
// B, so the coupler's and the follower's joints follow the crank. A static
// wall, x ∈ [−50, 150], y ∈ [68.5, 78.5], z ∈ [19, 29], stands where the
// follower's top corner rises past y = 68.5.
//
// Its drive moves a loop, so its frames are posed through a decad.Schedule
// rather than Linkage.PoseAt: the schedule holds the loop's certified
// enclosures, and VerifyLinkage reads every pose through the same chain.
func crankRockerScene(ctx context.Context) (*linkageScene, error) {
	scene := &linkageScene{
		doc: decad.New(),
		camera: kinetograph.Camera{
			Position: r3.Vec{X: 50, Y: -150, Z: 230},
			Target:   r3.Vec{X: 50, Y: 35, Z: 10},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(45)),
		},
	}
	t4 := rockerTheta4(0)
	b := [2]float64{rockerGround + rockerFollower*math.Cos(t4), rockerFollower * math.Sin(t4)}
	crankBody, err := extrudedBox(ctx, scene.doc, 0, -rockerHalf, rockerCrank, rockerHalf, 0, 8)
	if err != nil {
		return nil, fmt.Errorf("crank: %w", err)
	}
	coupler, err := rockerBar(ctx, scene.doc, [2]float64{rockerCrank, 0}, b, 10)
	if err != nil {
		return nil, fmt.Errorf("coupler: %w", err)
	}
	follower, err := rockerBar(ctx, scene.doc, [2]float64{rockerGround, 0}, b, 20)
	if err != nil {
		return nil, fmt.Errorf("follower: %w", err)
	}
	wall, err := extrudedBox(ctx, scene.doc, -50, rockerWallFace, 150, rockerWallFace+10, 19, 10)
	if err != nil {
		return nil, fmt.Errorf("wall: %w", err)
	}

	z := r3.NewVec(0, 0, 1)
	scene.linkage = decad.NewLinkage()
	crank, err := scene.linkage.Ground().Revolute(r3.Vec{}, z, []*decad.Body{crankBody})
	if err != nil {
		return nil, fmt.Errorf("crank: %w", err)
	}
	couplerLink, err := crank.Revolute(r3.NewVec(rockerCrank, 0, 0), z, []*decad.Body{coupler})
	if err != nil {
		return nil, fmt.Errorf("coupler: %w", err)
	}
	followerLink, err := scene.linkage.Ground().Revolute(r3.NewVec(rockerGround, 0, 0), z, []*decad.Body{follower})
	if err != nil {
		return nil, fmt.Errorf("follower: %w", err)
	}
	if _, err := scene.linkage.Close(couplerLink, followerLink, r3.NewVec(b[0], b[1], 0), z); err != nil {
		return nil, fmt.Errorf("closure: %w", err)
	}
	scene.drive = decad.Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}}
	scene.schedule, err = scene.linkage.Schedule(ctx, scene.drive)
	if err != nil {
		return nil, fmt.Errorf("schedule: %w", err)
	}
	scene.parts = []linkagePart{
		{name: "crank", body: crankBody, link: crank, color: violet},
		{name: "coupler", body: coupler, link: couplerLink, color: gold},
		{name: "follower", body: follower, link: followerLink, color: violet},
		{name: "wall", body: wall, color: navy},
	}
	return scene, nil
}
