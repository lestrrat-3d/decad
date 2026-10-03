package main

import (
	"context"
	"fmt"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// shelfPitch is the distance between two neighbouring parts on the shelf
// along X. The largest footprint half-diagonals, the tray's 56 mm and the
// dish's at most 59 mm, leave at least 25 mm between neighbours at every spin
// angle.
const shelfPitch = 140.0

// dollyMargin is how far before the first part the shelf camera starts, and
// how far past the last part it ends.
const dollyMargin = 87.5

// shelfSlot is one place on the shelf: the part's name, which names its spin
// track shape.<name>.spin, its look, and the builder of its bodies. Every
// body of one slot is drawn alike, and they turn together.
type shelfSlot struct {
	name       string
	appearance render.Appearance
	bodies     func(context.Context) ([]*decad.Body, error)
}

// one adapts a builder of one body to shelfSlot.bodies.
func one(build func(context.Context) (*decad.Body, error)) func(context.Context) ([]*decad.Body, error) {
	return func(ctx context.Context) ([]*decad.Body, error) {
		body, err := build(ctx)
		if err != nil {
			return nil, err
		}
		return []*decad.Body{body}, nil
	}
}

// shelf is act B's parts in shelf order: the bodies and colours of the
// revolve, sweep, loft, free-form, shell and surface thumbnails. The dish is
// violet outside and gold on its inner side, as surfaceShot draws it.
func shelf() []shelfSlot {
	inner := solidlens.Matte(gold)
	dish := matte(violet)
	dish.Back = &inner
	return []shelfSlot{
		{name: "ring", appearance: matte(blue), bodies: one(ringBody)},
		{name: "duct", appearance: matte(cyan), bodies: ductSpans},
		{name: "loft", appearance: matte(violet), bodies: one(loftDuct)},
		{name: "blade", appearance: matte(coral), bodies: one(bladeBody)},
		{name: "tray", appearance: matte(blue), bodies: one(trayBody)},
		{name: "dish", appearance: dish, bodies: one(dishBody)},
	}
}

// shapesTake is act B: the six parts in shelf order, each turning about the
// centre of its own footprint, passed by a dollying camera.
func shapesTake(ctx context.Context, ch *Channels) (*Take, error) {
	rig := kinetograph.NewRig()
	scene := kinetograph.NewScene(rig)
	parts := map[string]render.Appearance{}
	for k, slot := range shelf() {
		name := slot.name
		bodies, err := slot.bodies(ctx)
		if err != nil {
			return nil, fmt.Errorf("shelf part %s: %w", name, err)
		}
		node, err := shelfNode(rig.Root(), ch, k, name, bodies)
		if err != nil {
			return nil, fmt.Errorf("shelf part %s: %w", name, err)
		}
		for i, body := range bodies {
			partName := name
			if len(bodies) > 1 {
				partName = name + "." + strconv.Itoa(i)
			}
			if err := scene.AddPart(partName, node, body); err != nil {
				return nil, err
			}
			parts[partName] = slot.appearance
		}
	}

	dolly, err := ch.Get("shapes.dolly")
	if err != nil {
		return nil, err
	}
	cameraNode, err := rig.Root().Prismatic(r3.NewVec(1, 0, 0), dolly)
	if err != nil {
		return nil, fmt.Errorf("camera: %w", err)
	}
	err = scene.SetCamera(cameraNode, kinetograph.Camera{
		Position: featureCameraPosition,
		Target:   featureCameraTarget,
		Up:       r3.NewVec(0, 0, 1),
		FOV:      kinetograph.Constant(units.Degrees(30)),
	})
	if err != nil {
		return nil, fmt.Errorf("camera: %w", err)
	}
	return &Take{Scene: scene, Style: shapesStyle(parts)}, nil
}

// shelfNode is root -> Fixed translation to slot k -> Revolute about Z by
// track shape.<name>.spin -> Fixed translation by -c, where c is the centre
// of the bodies' combined XY extent.
func shelfNode(root *kinetograph.Node, ch *Channels, k int, name string, bodies []*decad.Body) (*kinetograph.Node, error) {
	centre, err := footprintCentre(bodies)
	if err != nil {
		return nil, err
	}
	slot, err := r3.Translation(r3.NewVec(shelfPitch*float64(k), 0, 0))
	if err != nil {
		return nil, err
	}
	slotNode, err := root.Fixed(slot)
	if err != nil {
		return nil, err
	}
	spin, err := ch.Get("shape." + name + ".spin")
	if err != nil {
		return nil, err
	}
	spinNode, err := slotNode.Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), spin)
	if err != nil {
		return nil, err
	}
	recentre, err := r3.Translation(r3.NewVec(-centre.X, -centre.Y, 0))
	if err != nil {
		return nil, err
	}
	return spinNode.Fixed(recentre)
}

// footprintCentre is the centre of the bodies' combined XY extent, from
// Body.Bounds.
func footprintCentre(bodies []*decad.Body) (r3.Vec, error) {
	var lo, hi r3.Vec
	for i, body := range bodies {
		box, err := body.Bounds()
		if err != nil {
			return r3.Vec{}, err
		}
		if i == 0 {
			lo, hi = box.Min, box.Max
			continue
		}
		lo = r3.NewVec(min(lo.X, box.Min.X), min(lo.Y, box.Min.Y), 0)
		hi = r3.NewVec(max(hi.X, box.Max.X), max(hi.Y, box.Max.Y), 0)
	}
	return r3.NewVec((lo.X+hi.X)/2, (lo.Y+hi.Y)/2, 0), nil
}
