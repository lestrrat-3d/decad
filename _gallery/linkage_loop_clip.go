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

// The crank-rocker's geometry, in millimetres and degrees: the ground pivot
// of the follower, the crank, coupler and follower lengths between their
// pins, the follower angle θ4 at which its leading flank lies along the stop's
// face, the stop face's extent along the follower from its pivot, and the
// stop's depth behind its face.
const (
	rockerGround    = 100.0
	rockerCrank     = 30.0
	rockerCoupler   = 80.0
	rockerFollower  = 70.0
	rockerStopDeg   = 144.5
	rockerStopFrom  = 30.0
	rockerStopTo    = 60.0
	rockerStopDepth = 8.0
)

// The crank-rocker's bars and hardware, in millimetres: the crank's and the
// follower's end radius, the coupler's, the bars' thickness, the pin shafts'
// radius and the bores' clearance around them, the pin caps' radius, each
// bar's underside along Z and the top of the base plate. The crank is at the
// bottom, the follower in the middle and the coupler on top, so the coupler
// can sweep over the crank's post.
const (
	rockerEnd        = 8.0
	rockerCouplerEnd = 7.0
	rockerThick      = 6.0
	rockerPin        = 4.0
	rockerClearance  = 0.5
	rockerHead       = 6.0
	rockerCrankZ     = 0.0
	rockerFollowerZ  = 10.0
	rockerCouplerZ   = 20.0
	rockerBaseTop    = -4.0
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

// rockerTheta2 is the first crank angle (radians) at which the follower
// reaches the angle th4 on its way up: the coupler pin B = O4 + 70·(cos θ4,
// sin θ4) is fixed by θ4, and the crank pin A lies on both the crank's circle
// about the origin and the coupler's circle about B, at the angle
// atan2(B) + acos((30² + |B|² − 80²)/(2·30·|B|)).
func rockerTheta2(th4 float64) float64 {
	bx, by := rockerGround+rockerFollower*math.Cos(th4), rockerFollower*math.Sin(th4)
	nb := math.Hypot(bx, by)
	return math.Atan2(by, bx) + math.Acos((rockerCrank*rockerCrank+nb*nb-rockerCoupler*rockerCoupler)/(2*rockerCrank*nb))
}

// rockerPoint is the point at distance u along the follower from its pivot
// O4 and n off it to the follower's left, with the follower at angle th4
// (radians).
func rockerPoint(th4, u, n float64) [2]float64 {
	c, s := math.Cos(th4), math.Sin(th4)
	return [2]float64{rockerGround + u*c - n*s, u*s + n*c}
}

// crankRockerScene is a crank-rocker four-bar on a base plate, a closed loop
// whose three bars lie in three layers along Z. The crank, a slot bar from
// the origin to A = (30, 0) with z ∈ [0, 6], turns a full turn about Z on a
// steel post that rises from the base plate: a collar under the crank, the
// shaft through its bore and a cap over it. The follower, a slot bar from
// O4 = (100, 0) to the pin B the four-bar construction puts above the ground
// line, z ∈ [10, 16], turns about O4 on a pivot shaft it carries down into a
// bearing tube on the base plate, with a cap on top. The coupler, a slot bar
// from A to B, z ∈ [20, 26], hangs from the crank at A and is closed onto
// the follower at B. The crank and the follower each carry the pin the
// coupler turns on, set pinOffset off its bore's centre: a shaft from the
// carrying bar up through the coupler's bore and a cap 1 mm over it. Every
// shaft clears its bore by at least 0.25 mm, and every shaft-in-bore pair is
// a declared joint contact.
//
// The follower rocks between about 101.8° and 152.3° from the ground line. A
// stop block on the base plate, z ∈ [−4, 16.5], has its face along the
// follower's leading flank at θ4 = 144.5°: the line 8 mm to the follower's
// left, from 30 mm to 60 mm along it, clear of the follower's rounded ends.
//
// Its drive moves a loop, so its frames are posed through a decad.Schedule
// rather than Linkage.PoseAt: the schedule holds the loop's certified
// enclosures, and VerifyLinkage reads every pose through the same chain.
func crankRockerScene(ctx context.Context) (*linkageScene, error) {
	scene := &linkageScene{
		doc: decad.New(),
		camera: kinetograph.Camera{
			Position: r3.Vec{X: 35, Y: -130, Z: 120},
			Target:   r3.Vec{X: 35, Y: 26, Z: 8},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(39)),
		},
	}
	t4 := rockerTheta4(0)
	b := [2]float64{rockerGround + rockerFollower*math.Cos(t4), rockerFollower * math.Sin(t4)}
	a := [2]float64{rockerCrank, 0}
	o4 := [2]float64{rockerGround, 0}
	bore := rockerPin + rockerClearance
	crankBody, err := linkBar{p: [2]float64{}, q: a, radius: rockerEnd, bore: bore, bores: []int{0},
		z0: rockerCrankZ, thickness: rockerThick}.body(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("crank: %w", err)
	}
	coupler, err := linkBar{p: a, q: b, radius: rockerCouplerEnd, bore: bore, bores: []int{0, 1},
		z0: rockerCouplerZ, thickness: rockerThick}.body(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("coupler: %w", err)
	}
	follower, err := linkBar{p: o4, q: b, radius: rockerEnd, z0: rockerFollowerZ, thickness: rockerThick}.body(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("follower: %w", err)
	}
	couplerPinSteps := []pinStep{
		{radius: rockerPin, top: rockerCouplerZ + rockerThick + 1},
		{radius: rockerHead, top: rockerCouplerZ + rockerThick + 3},
	}
	crankPin, err := pinStack(ctx, scene.doc, a[0], a[1]-pinOffset, rockerCrankZ, couplerPinSteps)
	if err != nil {
		return nil, fmt.Errorf("crank pin: %w", err)
	}
	couplerPin, err := pinStack(ctx, scene.doc, b[0], b[1]-pinOffset, rockerFollowerZ, couplerPinSteps)
	if err != nil {
		return nil, fmt.Errorf("coupler pin: %w", err)
	}
	pivot, err := pinStack(ctx, scene.doc, o4[0], o4[1], rockerBaseTop+2, []pinStep{
		{radius: rockerPin, top: rockerFollowerZ},
	})
	if err != nil {
		return nil, fmt.Errorf("follower pivot: %w", err)
	}
	pivotCap, err := pinStack(ctx, scene.doc, o4[0], o4[1], rockerFollowerZ+rockerThick, []pinStep{
		{radius: 7, top: rockerFollowerZ + rockerThick + 1.5},
	})
	if err != nil {
		return nil, fmt.Errorf("follower pivot cap: %w", err)
	}
	bearing, err := pinStack(ctx, scene.doc, o4[0], o4[1], rockerBaseTop, []pinStep{
		{radius: 9, bore: bore, top: rockerFollowerZ - 1},
	})
	if err != nil {
		return nil, fmt.Errorf("follower bearing: %w", err)
	}
	crankPost, err := pinStack(ctx, scene.doc, 0, 0, rockerBaseTop, []pinStep{
		{radius: 9, top: rockerCrankZ - 1},
		{radius: rockerPin, top: rockerCrankZ + rockerThick + 1},
		{radius: 7, top: rockerCrankZ + rockerThick + 2.5},
	})
	if err != nil {
		return nil, fmt.Errorf("crank post: %w", err)
	}
	stopAngle := rockerStopDeg * math.Pi / 180
	stop, err := extrudedPolygon(ctx, scene.doc, [][2]float64{
		rockerPoint(stopAngle, rockerStopFrom, rockerEnd),
		rockerPoint(stopAngle, rockerStopFrom, rockerEnd+rockerStopDepth),
		rockerPoint(stopAngle, rockerStopTo, rockerEnd+rockerStopDepth),
		rockerPoint(stopAngle, rockerStopTo, rockerEnd),
	}, rockerFollowerZ+rockerThick+0.5-rockerBaseTop)
	if err != nil {
		return nil, fmt.Errorf("stop: %w", err)
	}
	shift, err := r3.Translation(r3.Vec{Z: rockerBaseTop})
	if err != nil {
		return nil, err
	}
	if stop, err = stop.Placed(ctx, shift); err != nil {
		return nil, fmt.Errorf("stop: %w", err)
	}
	base, err := extrudedBox(ctx, scene.doc, -45, -45, 115, 85, rockerBaseTop-6, 6)
	if err != nil {
		return nil, fmt.Errorf("base: %w", err)
	}

	z := r3.NewVec(0, 0, 1)
	scene.linkage = decad.NewLinkage()
	crank, err := scene.linkage.Ground().Revolute(r3.Vec{}, z, append([]*decad.Body{crankBody}, crankPin...))
	if err != nil {
		return nil, fmt.Errorf("crank: %w", err)
	}
	couplerLink, err := crank.Revolute(r3.NewVec(a[0], a[1], 0), z, []*decad.Body{coupler})
	if err != nil {
		return nil, fmt.Errorf("coupler: %w", err)
	}
	followerBodies := append([]*decad.Body{follower}, pivot[0], pivotCap[0])
	followerLink, err := scene.linkage.Ground().Revolute(r3.NewVec(o4[0], o4[1], 0), z,
		append(followerBodies, couplerPin...))
	if err != nil {
		return nil, fmt.Errorf("follower: %w", err)
	}
	if _, err := scene.linkage.Close(couplerLink, followerLink, r3.NewVec(b[0], b[1], 0), z); err != nil {
		return nil, fmt.Errorf("closure: %w", err)
	}
	for _, pair := range [][2]*decad.Body{
		{crankBody, crankPost[1]}, {crankPin[0], coupler}, {pivot[0], bearing[0]}, {coupler, couplerPin[0]},
	} {
		if err := scene.linkage.DeclareJointContact(pair[0], pair[1]); err != nil {
			return nil, fmt.Errorf("joint contact: %w", err)
		}
	}
	scene.drive = decad.Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(360)}}
	scene.schedule, err = scene.linkage.Schedule(ctx, scene.drive)
	if err != nil {
		return nil, fmt.Errorf("schedule: %w", err)
	}
	scene.parts = []linkagePart{
		{name: "crank", body: crankBody, link: crank, color: violet},
		{name: "coupler", body: coupler, link: couplerLink, color: gold},
		{name: "follower", body: follower, link: followerLink, color: violet},
		{name: "stop", body: stop, color: navy},
		{name: "base", body: base, color: slate},
	}
	scene.addHardware("crank-pin", crankPin, crank)
	scene.addHardware("coupler-pin", couplerPin, followerLink)
	scene.addHardware("follower-pivot", append(pivot, pivotCap...), followerLink)
	scene.addHardware("crank-post", crankPost, nil)
	scene.addHardware("follower-bearing", bearing, nil)
	return scene, nil
}
