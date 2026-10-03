package main

import (
	"context"
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// Each drill tool is a cylinder toolMargin larger in radius than its hole, so
// that a landed tool covers the hole's rim. Landed, it spans z = -16 to
// 32 mm; it starts toolTravel above that, out of view.
const (
	toolMargin       = 0.3
	toolLength       = 48.0
	toolLandedBottom = -16.0
	toolTravel       = 160.0
)

// toolFade is a drill tool's opacity while it plunges and holds in its hole:
// the gold tool still reads as a tool, and the hole shows growing inside it.
const toolFade = 0.4

// The fillet lamp is a white point light at filletLampPosition on a revolute
// joint about Z through the origin, which turns it from filletLampFrom to
// filletLampTo degrees past the round nearest the camera. It sits at the
// plate's mid-height, below the top face's plane, so it lights the side faces
// and the rounds and leaves the top face, whose few large triangles solidlens
// shades flat, unlit. At filletLampIntensity it adds about 1.5 to the +X side
// face, 32 mm away.
var filletLampPosition = r3.NewVec(80, 0, 8)

const (
	filletLampFrom      = -60.0
	filletLampTo        = 30.0
	filletLampIntensity = 1500.0
)

// The flange's final fillet radius and chamfer setback.
const (
	filletRadius   = 12.0
	chamferSetback = 2.0
)

// chamferStep is the grid the flange's chamfer setback is rounded down to.
// On the filleted, drilled plate decad refuses most setbacks between 0.5 and
// 2 mm ("the offset changes the section's topology"): of 301 setbacks 0.005 mm
// apart it accepted 70, among them every multiple of 0.125 mm. One step is
// about one pixel at act A's framing.
const chamferStep = 0.125

// pinDrop is how far verifyPin's pin drops into the 18 mm bore, landing
// standing on the plate's bottom plane.
const pinDrop = 200.0

// buildTiltAxis is horizontal and perpendicular to the feature camera's view
// in the orbit joint's frame; a positive angle about it raises the camera.
var buildTiltAxis = r3.NewVec(-165, -105, 0)

// flangeBuilder is the flange as a parametric part. Its parameters are
// height, fillet, chamfer and one plunge distance per drill name. A plunge
// becomes a hole depth the way the tool's tip reaches into the plate: the tip
// starts toolTravel above its landed pose at z = -16 mm, so at plunge p it
// lies p - 128 mm below a 16 mm plate's top face. The chamfer setback is
// rounded down to a multiple of chamferStep.
type flangeBuilder struct{}

// Build builds the flange from params. Its errors name every parameter value.
func (flangeBuilder) Build(ctx context.Context, params kinetograph.Params) (*decad.Body, error) {
	shape, err := shapeFromParams(params)
	if err == nil {
		var body *decad.Body
		body, err = flangeBody(ctx, shape)
		if err == nil {
			return body, nil
		}
	}
	return nil, fmt.Errorf("flange at %s: %w", formatParams(params), err)
}

// shapeFromParams reads flangeBuilder's parameters in millimetres.
func shapeFromParams(params kinetograph.Params) (flangeShape, error) {
	read := func(name string) (float64, error) {
		v, ok := params[name]
		if !ok {
			return 0, fmt.Errorf("no parameter %q", name)
		}
		return v.In(units.Millimeter)
	}
	var shape flangeShape
	var err error
	if shape.height, err = read("height"); err != nil {
		return shape, err
	}
	if shape.fillet, err = read("fillet"); err != nil {
		return shape, err
	}
	chamfer, err := read("chamfer")
	if err != nil {
		return shape, err
	}
	shape.chamfer = math.Floor(chamfer/chamferStep) * chamferStep
	shape.depths = make(map[string]float64, len(drills))
	for _, d := range drills {
		plunge, err := read(d.name)
		if err != nil {
			return shape, err
		}
		tip := toolLandedBottom + toolTravel - plunge
		shape.depths[d.name] = shape.height - tip
	}
	return shape, nil
}

// formatParams writes params as name=value pairs in name order.
func formatParams(params kinetograph.Params) string {
	names := make([]string, 0, len(params))
	for name := range params {
		names = append(names, name)
	}
	slices.Sort(names)
	pairs := make([]string, len(names))
	for i, name := range names {
		pairs[i] = name + "=" + params[name].String()
	}
	return strings.Join(pairs, " ")
}

// buildTake is act A with the flange as a parametric part.
func buildTake(ctx context.Context, ch *Channels) (*Take, error) {
	return assembleBuild(ctx, ch, nil)
}

// assembleBuild is act A's scene and style. With plate nil the flange is the
// parametric part flangeBuilder builds; otherwise plate stands in for it,
// still, which is how build_test compares two stages of the plate.
func assembleBuild(ctx context.Context, ch *Channels, plate *decad.Body) (*Take, error) {
	rig := kinetograph.NewRig()
	scene := kinetograph.NewScene(rig)
	if err := addFlange(scene, rig.Root(), ch, plate); err != nil {
		return nil, fmt.Errorf("flange: %w", err)
	}

	parts := map[string]render.Appearance{}
	for _, d := range drills {
		name := "tool." + d.name
		if err := addTool(ctx, scene, rig.Root(), ch, d); err != nil {
			return nil, fmt.Errorf("%s: %w", name, err)
		}
		fade, err := ch.Get(name + ".fade")
		if err != nil {
			return nil, fmt.Errorf("%s: %w", name, err)
		}
		look := matte(gold)
		look.Fade = fade
		parts[name] = look
	}
	if err := addPin(ctx, scene, rig.Root(), ch); err != nil {
		return nil, fmt.Errorf("pin: %w", err)
	}
	parts["pin"] = matte(gold)

	lamp, err := addFilletLamp(scene, rig.Root(), ch)
	if err != nil {
		return nil, fmt.Errorf("fillet lamp: %w", err)
	}
	if err := setBuildCamera(scene, rig.Root(), ch); err != nil {
		return nil, fmt.Errorf("camera: %w", err)
	}
	return &Take{Scene: scene, Style: buildStyle(parts, map[string]render.LightAppearance{filletLamp: lamp})}, nil
}

// filletLamp is the name of act A's moving light.
const filletLamp = "lamp.fillet"

// addFilletLamp hangs the fillet lamp off root -> Revolute about Z through the
// origin (lamp.fillet.turn) and returns its look: white, at the intensity of
// track lamp.fillet.intensity.
func addFilletLamp(scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels) (render.LightAppearance, error) {
	turn, err := ch.Get("lamp.fillet.turn")
	if err != nil {
		return render.LightAppearance{}, err
	}
	intensity, err := ch.Get("lamp.fillet.intensity")
	if err != nil {
		return render.LightAppearance{}, err
	}
	node, err := root.Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), turn)
	if err != nil {
		return render.LightAppearance{}, err
	}
	light := kinetograph.Light{Kind: kinetograph.PointLight, Position: filletLampPosition}
	if err := scene.AddLight(filletLamp, node, light); err != nil {
		return render.LightAppearance{}, err
	}
	return whiteLamp(intensity), nil
}

// addFlange attaches the flange to root: plate when it is non-nil, the
// parametric flange otherwise.
func addFlange(scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels, plate *decad.Body) error {
	if plate != nil {
		return scene.AddPart("flange", root, plate)
	}
	tracks := map[string]string{
		"height":  "flange.height",
		"fillet":  "flange.fillet",
		"chamfer": "flange.chamfer",
	}
	for _, d := range drills {
		tracks[d.name] = "hole." + d.name
	}
	params := make(map[string]*kinetograph.Channel, len(tracks))
	for param, track := range tracks {
		c, err := ch.Get(track)
		if err != nil {
			return err
		}
		params[param] = c
	}
	return scene.AddParametric("flange", root, flangeBuilder{}, params)
}

// addTool hangs d's drill tool toolTravel above its landed pose over its hole
// and plunges it along -Z by its plunge track.
func addTool(ctx context.Context, scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels, d drill) error {
	w := sketch.NewWorld()
	body, err := cylinder(ctx, decad.New(), w, w.XY(), point{}, d.radius+toolMargin, decad.Distance{
		D:   units.Millimeters(toolLength),
		Dir: decad.Along,
	})
	if err != nil {
		return err
	}
	node, err := dropNode(root, r3.NewVec(d.x, 0, toolLandedBottom+toolTravel), ch, "tool."+d.name+".plunge")
	if err != nil {
		return err
	}
	return scene.AddPart("tool."+d.name, node, body)
}

// addPin hangs verifyPin's pin pinDrop above the bore and drops it along -Z by
// track pin.drop, so it lands standing on the plate's bottom plane.
func addPin(ctx context.Context, scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels) error {
	w := sketch.NewWorld()
	body, err := verifyPin(ctx, decad.New(), w)
	if err != nil {
		return err
	}
	node, err := dropNode(root, r3.NewVec(0, 0, pinDrop), ch, "pin.drop")
	if err != nil {
		return err
	}
	return scene.AddPart("pin", node, body)
}

// dropNode is root -> Fixed translation by start -> Prismatic -Z by track.
func dropNode(root *kinetograph.Node, start r3.Vec, ch *Channels, track string) (*kinetograph.Node, error) {
	hang, err := r3.Translation(start)
	if err != nil {
		return nil, err
	}
	over, err := root.Fixed(hang)
	if err != nil {
		return nil, err
	}
	drop, err := ch.Get(track)
	if err != nil {
		return nil, err
	}
	return over.Prismatic(r3.NewVec(0, 0, -1), drop)
}

// setBuildCamera puts the feature camera on root -> Revolute about Z through
// the origin (cam.orbit) -> Revolute about buildTiltAxis through the target
// (cam.tilt), with field of view cam.fov.
func setBuildCamera(scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels) error {
	orbit, err := ch.Get("cam.orbit")
	if err != nil {
		return err
	}
	tilt, err := ch.Get("cam.tilt")
	if err != nil {
		return err
	}
	fov, err := ch.Get("cam.fov")
	if err != nil {
		return err
	}
	orbitNode, err := root.Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), orbit)
	if err != nil {
		return err
	}
	tiltNode, err := orbitNode.Revolute(featureCameraTarget, buildTiltAxis, tilt)
	if err != nil {
		return err
	}
	return scene.SetCamera(tiltNode, kinetograph.Camera{
		Position: featureCameraPosition,
		Target:   featureCameraTarget,
		Up:       r3.NewVec(0, 0, 1),
		FOV:      fov,
	})
}
