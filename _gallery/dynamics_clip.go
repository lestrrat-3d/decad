package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"math/big"
	"runtime"
	"sort"
	"strings"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// dynamicsScene is one exit scene of docs/multibody-dynamics-design.md §2:
// the world and its start state, the per-step input and step length, the
// clip length, and how each body is drawn.
type dynamicsScene struct {
	name   string
	doc    *decad.Document
	config dynamics.WorldConfig
	world  *dynamics.World
	start  dynamics.State
	input  dynamics.StepInput
	dt     units.Value
	length time.Duration
	parts  []dynamicsPart
	camera kinetograph.Camera
}

// dynamicsPart is one body of a scene, as kinetograph draws it.
type dynamicsPart struct {
	name  string
	body  *decad.Body
	color solidlens.Color
}

// dynamicsScenes are the scenes `go run . dynamics -scene <name>` renders.
var dynamicsScenes = map[string]func(context.Context) (*dynamicsScene, error){
	"parts-bin":      partsBinScene,
	"stack-and-drop": stackAndDropScene,
	"tumble":         tumbleScene,
}

// stackAndDropSpheres are the three spheres' release centers: over x = 120,
// at z = 60, 90 and 120, offset in y so the second lands on the first and the
// third on them.
var stackAndDropSpheres = [3]r3.Vec{{X: 120, Y: 10, Z: 60}, {X: 120, Y: 14, Z: 90}, {X: 120, Y: 6, Z: 120}}

// stackAndDropCylinder is the cylinder's release translation; its modeled
// lower disk is centered on the origin.
var stackAndDropCylinder = r3.Vec{X: 160, Y: 10, Z: 80}

// stackAndDropBoxes are the 3-2-1 pyramid's 20 mm boxes, by lower corner:
// three on the floor, two bridging the 5 mm gaps on top, one on the top row.
var stackAndDropBoxes = [6]r3.Vec{
	{X: 0}, {X: 25}, {X: 50},
	{X: 12.5, Z: 20}, {X: 37.5, Z: 20},
	{X: 25, Z: 40},
}

// stackAndDropScene is §2's Phase 1 scene. A fixed 360×2000×10 mm source-box
// floor has its top at z = 0, spanning x = −60…300 and y = −990…1010: the
// spheres glance off one another and roll along y, with no rolling
// resistance, up to 766 mm from their drop line, and the sphere-box contact
// needs each sphere inside the floor's top face. The camera looks along the
// floor from the −x side so the whole roll stays in frame. The pyramid of stackAndDropBoxes starts at rest
// in touch. Three radius-8 source spheres and a Ø20×30 source cylinder, axis
// along z, drop from rest. Boxes and the floor take restitution 0.3 and
// spheres 0.6 (a pair takes the smaller), friction is 0.4 everywhere and
// density 0.001 kg/mm³. Gravity is −9810 mm/s², the step 1/256 s so every
// kick is exact, and ImpactSpeed 64 mm/s lies above the kick so a resting
// body stays at rest. The clip is 2 s.
func stackAndDropScene(ctx context.Context) (*dynamicsScene, error) {
	scene := &dynamicsScene{
		name:   "stack-and-drop",
		doc:    decad.New(),
		input:  dynamics.StepInput{Gravity: millimetersPerSecondSquared(0, 0, -9810)},
		dt:     units.Seconds(1.0 / 256),
		length: 2 * time.Second,
		camera: kinetograph.Camera{
			Position: r3.Vec{X: -652, Y: -318, Z: 600},
			Target:   r3.Vec{X: 120, Y: -110, Z: 0},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(45)),
		},
	}
	density := units.KilogramsPerCubicMillimeter(0.001)
	box := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	ball := dynamics.Material{Restitution: units.Scalar(0.6), Friction: units.Scalar(0.4)}

	var entries []dynamics.BodyState
	add := func(name string, body *decad.Body, role dynamics.BodyRole, material dynamics.Material,
		at r3.Vec, color solidlens.Color) error {
		definition := dynamics.RigidBody{Body: body, Role: role, Material: material}
		if role == dynamics.Dynamic {
			definition.Density = &density
		}
		pose, err := r3.Translation(at)
		if err != nil {
			return err
		}
		scene.config.Bodies = append(scene.config.Bodies, definition)
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose,
			LinearVelocity: millimetersPerSecond(0, 0, 0), AngularVelocity: radiansPerSecond(0, 0, 0)})
		scene.parts = append(scene.parts, dynamicsPart{name: name, body: body, color: color})
		return nil
	}

	floor, err := extrudedBox(ctx, scene.doc, -60, -990, 300, 1010, -10, 10)
	if err != nil {
		return nil, fmt.Errorf("floor: %w", err)
	}
	if err := add("floor", floor, dynamics.Fixed, box, r3.Vec{}, navy); err != nil {
		return nil, err
	}
	for i, corner := range stackAndDropBoxes {
		body, err := extrudedBox(ctx, scene.doc, corner.X, corner.Y, corner.X+20, corner.Y+20, corner.Z, 20)
		if err != nil {
			return nil, fmt.Errorf("box %d: %w", i, err)
		}
		if err := add(fmt.Sprintf("box%d", i), body, dynamics.Dynamic, box, r3.Vec{}, violet); err != nil {
			return nil, err
		}
	}
	for i, center := range stackAndDropSpheres {
		body, err := revolvedSphere(ctx, scene.doc, 8)
		if err != nil {
			return nil, fmt.Errorf("sphere %d: %w", i, err)
		}
		if err := add(fmt.Sprintf("sphere%d", i), body, dynamics.Dynamic, ball, center, coral); err != nil {
			return nil, err
		}
	}
	cylinder, err := extrudedCylinder(ctx, scene.doc, 10, 30)
	if err != nil {
		return nil, fmt.Errorf("cylinder: %w", err)
	}
	if err := add("cylinder", cylinder, dynamics.Dynamic, box, stackAndDropCylinder, gold); err != nil {
		return nil, err
	}

	scene.config.Step = sceneStepConfig()
	scene.world, err = dynamics.NewWorld(ctx, scene.doc, scene.config)
	if err != nil {
		return nil, fmt.Errorf("world: %w", err)
	}
	scene.start, err = scene.world.NewState(entries)
	if err != nil {
		return nil, fmt.Errorf("start state: %w", err)
	}
	return scene, nil
}

// sceneStepConfig is the step configuration both exit scenes share:
// 1e-6 resolutions and residuals, ImpactSpeed 64 mm/s above the 9810/256 mm/s
// kick so a resting body targets zero speed, and budgets the scenes' busiest
// steps stay within.
func sceneStepConfig() dynamics.StepConfig {
	return dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution:          units.Seconds(1e-9),
		ContactSlop:             units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(64),
		MaxPoseEvaluations:      128,
		MaxIterations:           4096,
		MaxEvents:               64,
		MaxPairSweeps:           1 << 16,
	}
}

// steps is how many steps of dt make the clip length. The length must be a
// whole number of steps, exactly.
func (s *dynamicsScene) steps() (int, error) {
	length := new(big.Rat).SetFrac64(int64(s.length), int64(time.Second))
	dt := new(big.Rat).SetFloat64(s.dt.Base())
	if dt == nil || dt.Sign() <= 0 {
		return 0, fmt.Errorf("scene %s: step %s is not a positive time", s.name, s.dt)
	}
	n := new(big.Rat).Quo(length, dt)
	if !n.IsInt() {
		return 0, fmt.Errorf("scene %s: clip length %s is not a whole number of %s steps", s.name, s.length, s.dt)
	}
	return int(n.Num().Int64()), nil
}

// advance runs the scene's timeline to the clip length, or until a step is
// Undecided: the timeline then stops at its certified end, and Stopped holds
// the step that stopped it. Only a step error is an error here.
func (s *dynamicsScene) advance(ctx context.Context) (*dynamics.Timeline, error) {
	n, err := s.steps()
	if err != nil {
		return nil, err
	}
	timeline, err := dynamics.NewTimeline(s.world, s.start)
	if err != nil {
		return nil, err
	}
	for range n {
		report, err := timeline.Advance(ctx, s.input, s.dt)
		if err != nil {
			return nil, fmt.Errorf("scene %s: step at %s: %w", s.name, timeline.End(), err)
		}
		if report.Status != dynamics.Advanced {
			break
		}
	}
	return timeline, nil
}

// stopError describes the step that stopped timeline, naming each
// diagnostic's code, bodies, interval, limit and reason.
func (s *dynamicsScene) stopError(timeline *dynamics.Timeline) error {
	report := timeline.Stopped()
	if report == nil {
		return nil
	}
	names := make(map[*decad.Body]string, len(s.parts))
	for _, part := range s.parts {
		names[part.body] = part.name
	}
	var b strings.Builder
	fmt.Fprintf(&b, "scene %s: the timeline stopped at %s, before the clip length %s", s.name, timeline.End(), s.length)
	for _, d := range report.Diagnostics {
		fmt.Fprintf(&b, "\n  code %d", d.Code)
		if d.Pair.A != nil {
			fmt.Fprintf(&b, ", pair %s/%s", names[d.Pair.A], names[d.Pair.B])
		}
		if len(d.Bodies) != 0 {
			bodies := make([]string, len(d.Bodies))
			for i, body := range d.Bodies {
				bodies[i] = names[body]
			}
			fmt.Fprintf(&b, ", bodies %s", strings.Join(bodies, " "))
		}
		fmt.Fprintf(&b, ", from %s to %s after the step start, limit %s: %s", d.From, d.To, d.Limit, d.Reason)
	}
	return errors.New(b.String())
}

// clipScene builds the kinetograph scene that films timeline: one driven node
// per body directly under the rig's root, each part named after its body,
// and the scene's still camera.
func (s *dynamicsScene) clipScene(timeline *dynamics.Timeline) (*kinetograph.Scene, error) {
	rig := kinetograph.NewRig()
	scene := kinetograph.NewScene(rig)
	for _, part := range s.parts {
		node, err := rig.Root().Driven(timelineTrack{timeline: timeline, body: part.body})
		if err != nil {
			return nil, err
		}
		if err := scene.AddPart(part.name, node, part.body); err != nil {
			return nil, err
		}
	}
	if err := scene.SetCamera(rig.Root(), s.camera); err != nil {
		return nil, err
	}
	return scene, nil
}

// style is the scene's look at width×height: the feature thumbnails'
// background and directional lights, each part in its own color.
func (s *dynamicsScene) style(width, height int) render.Style {
	parts := make(map[string]render.Appearance, len(s.parts))
	for _, part := range s.parts {
		parts[part.name] = matte(part.color)
	}
	return render.Style{
		Width:             width,
		Height:            height,
		Chord:             units.Millimeters(clipPartChord),
		Background:        backgroundColor,
		Default:           matte(violet),
		Parts:             parts,
		DirectionalLights: featureDirectionalLights(),
	}
}

// dynamicsOptions are the dynamics subcommand's parsed flags.
type dynamicsOptions struct {
	scene         string
	out           string
	width, height int
	fps           int
	workers       int
	smoke         bool
}

// runDynamics renders one exit scene of docs/multibody-dynamics-design.md §2
// as a PNG sequence (§11.2): it advances the scene's dynamics.Timeline to the
// clip length, then films it with one kinetograph driven node per body, so
// every frame shows the certified pose at its time and none is interpolated.
//
//	go run . dynamics -scene stack-and-drop
//
// writes out/stack-and-drop_000000.png and on. When the timeline stops at an
// Undecided step, the frames before its certified end still render, the
// first frame past it fails, and the command fails naming the stopped step's
// diagnostics.
//
// Flags:
//
//   - -scene <name> picks the scene (required; stack-and-drop, tumble or
//     parts-bin).
//   - -out <dir> is where the frames go (default "out").
//   - -fps sets the frame rate (default 60).
//   - -width and -height set the frame size (default 1280x720).
//   - -workers sets how many frames render at once (default: the CPU count).
//   - -smoke renders the first frame alone at 160x90, after advancing the
//     whole timeline.
func runDynamics(ctx context.Context, args []string, stderr io.Writer) error {
	var opts dynamicsOptions
	fs := flag.NewFlagSet("dynamics", flag.ContinueOnError)
	fs.StringVar(&opts.scene, "scene", "", "scene to render: "+strings.Join(dynamicsSceneNames(), ", "))
	fs.StringVar(&opts.out, "out", "out", "directory to write the frames to")
	fs.IntVar(&opts.fps, "fps", 60, "frames per second")
	fs.IntVar(&opts.width, "width", 1280, "frame width in pixels")
	fs.IntVar(&opts.height, "height", 720, "frame height in pixels")
	fs.IntVar(&opts.workers, "workers", runtime.NumCPU(), "frames rendered at once")
	fs.BoolVar(&opts.smoke, "smoke", false, "render the first frame alone at 160x90")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() > 0 {
		return fmt.Errorf("dynamics: unexpected argument %q", fs.Arg(0))
	}
	build, ok := dynamicsScenes[opts.scene]
	if !ok {
		return fmt.Errorf("dynamics: unknown scene %q; valid names are %s", opts.scene,
			strings.Join(dynamicsSceneNames(), ", "))
	}
	if opts.smoke {
		opts.width, opts.height = smokeWidth, smokeHeight
	}
	if opts.fps < 1 || opts.width < 1 || opts.height < 1 || opts.workers < 1 {
		return fmt.Errorf("dynamics: -fps, -width, -height and -workers must be positive")
	}

	scene, err := build(ctx)
	if err != nil {
		return err
	}
	fmt.Fprintf(stderr, "%s: advancing the timeline to %s\n", scene.name, scene.length)
	timeline, err := scene.advance(ctx)
	if err != nil {
		return err
	}
	clipScene, err := scene.clipScene(timeline)
	if err != nil {
		return err
	}
	length := scene.length
	if opts.smoke {
		length = time.Second / time.Duration(opts.fps)
	}
	clip, err := kinetograph.NewClip(clipScene, opts.fps, length)
	if err != nil {
		return err
	}
	renderer, err := render.New(ctx, clip, scene.style(opts.width, opts.height))
	if err != nil {
		return err
	}
	fmt.Fprintf(stderr, "%s: rendering %d frames\n", scene.name, clip.FrameCount())
	seq, renderErr := renderer.Sequence(ctx, opts.out, render.WithWorkers(opts.workers),
		render.WithPrefix(scene.name+"_"))
	if stop := scene.stopError(timeline); stop != nil {
		return errors.Join(stop, renderErr)
	}
	if renderErr != nil {
		return renderErr
	}
	fmt.Fprintf(stderr, "%s: wrote %d frames to %s\n", scene.name, seq.Frames, seq.Dir)
	return nil
}

// dynamicsSceneNames lists the scenes in sorted order.
func dynamicsSceneNames() []string {
	names := make([]string, 0, len(dynamicsScenes))
	for name := range dynamicsScenes {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

func millimetersPerSecondSquared(x, y, z float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(x), Y: units.MillimetersPerSecondSquared(y),
		Z: units.MillimetersPerSecondSquared(z)}
}

func millimetersPerSecond(x, y, z float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecond(x), Y: units.MillimetersPerSecond(y),
		Z: units.MillimetersPerSecond(z)}
}

func radiansPerSecond(x, y, z float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.RadiansPerSecond(x), Y: units.RadiansPerSecond(y),
		Z: units.RadiansPerSecond(z)}
}

// extrudedBox is the source box [x0, x1]×[y0, y1]×[z0, z0+height], a
// rectangle sketched on the plane z = z0 and extruded up.
func extrudedBox(ctx context.Context, doc *decad.Document, x0, y0, x1, y1, z0, height float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
}

// revolvedSphere is the source sphere of the given radius centered on the
// origin: a half disk revolved a full turn about its diameter.
func revolvedSphere(ctx context.Context, doc *decad.Document, radius float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	left := s.CreatePoint(-radius, 0)
	s.Fix(left)
	right := s.CreatePoint(radius, 0)
	center := s.CreatePoint(0, 0)
	s.CreateLine(left, right)
	s.CreateArc(center, right, left)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
}

// extrudedCylinder is the source cylinder of the given radius and height,
// its lower disk centered on the origin and its axis along +z.
func extrudedCylinder(ctx context.Context, doc *decad.Document, radius, height float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, radius)
	s.Fix(center)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
}

// tumbleRelease is one tumble body's release: a proper rotation by degrees
// about axis, applied in the body's own frame, then a translation to at.
type tumbleRelease struct {
	axis    r3.Vec
	degrees float64
	at      r3.Vec
}

// tumbleBoxReleases are the four 20 mm boxes, centered on their own origin,
// turned about (1, 1, 0) by 30°, 45°, 60° and 75° and released with their
// centers 40 mm above the tray floor, 45 mm from the tray's center along
// each diagonal, so every box stays at least 20 mm inside the walls.
var tumbleBoxReleases = [4]tumbleRelease{
	{axis: r3.Vec{X: 1, Y: 1}, degrees: 30, at: r3.Vec{X: -45, Y: -45, Z: 40}},
	{axis: r3.Vec{X: 1, Y: 1}, degrees: 45, at: r3.Vec{X: 45, Y: -45, Z: 40}},
	{axis: r3.Vec{X: 1, Y: 1}, degrees: 60, at: r3.Vec{X: -45, Y: 45, Z: 40}},
	{axis: r3.Vec{X: 1, Y: 1}, degrees: 75, at: r3.Vec{X: 45, Y: 45, Z: 40}},
}

// tumbleHexRelease tips the hexagonal prism 37° about −Y about its corner at
// the origin, which every other corner then stands above, and puts that
// corner 20 mm above the tray floor: the prism lands on one vertex.
var tumbleHexRelease = tumbleRelease{axis: r3.Vec{Y: -1}, degrees: 37, at: r3.Vec{X: -55, Z: 20}}

// tumbleWedgeRelease and tumbleTetrahedronRelease turn the wedge and the
// tetrahedron about generic axes, so each lands on one vertex.
var (
	tumbleWedgeRelease       = tumbleRelease{axis: r3.Vec{X: 1, Y: 2, Z: 3}, degrees: 50, at: r3.Vec{X: 40, Z: 25}}
	tumbleTetrahedronRelease = tumbleRelease{axis: r3.Vec{X: 3, Y: -1, Z: 2}, degrees: 40, at: r3.Vec{Z: 25}}
)

// tumbleHexagon is the hexagonal prism's section, 20 mm across its flats with
// dyadic corners, one corner at the origin.
var tumbleHexagon = [][2]float64{{0, 0}, {5.75, -10}, {17.25, -10}, {23, 0}, {17.25, 10}, {5.75, 10}}

// tumbleWedge is the triangular wedge's section; the wedge is 8 mm thick.
var tumbleWedge = [][2]float64{{0, 0}, {16, 0}, {4, 12}}

// tumbleScene is §2's Phase 2 scene. The fixed support is a tray built as a
// zero-bound Boolean Cut: the source box [−90, 90]²×[−10, 40] minus the
// source box [−80, 80]²×[0, 50], which opens its top and leaves a 10 mm floor
// with its top face at z = 0 and four 10 mm walls around the 160×160 mm
// inside. The tray is the scene's only fixed body, so no other fixed body
// touches it. Dynamic: the four boxes of tumbleBoxReleases spinning at
// (2, 1, 0) rad/s, and the hexagonal prism, the wedge and the stitched
// tetrahedron, each released on a vertex at rest. Every body takes
// restitution 0.3 and friction 0.4 at density 0.001 kg/mm³, under the step
// of stack-and-drop with PenetrationResidual 10 µm and SupportBand 5 µm
// (§2, §10.8). The clip is 3 s.
func tumbleScene(ctx context.Context) (*dynamicsScene, error) {
	scene := &dynamicsScene{
		name:   "tumble",
		doc:    decad.New(),
		input:  dynamics.StepInput{Gravity: millimetersPerSecondSquared(0, 0, -9810)},
		dt:     units.Seconds(1.0 / 256),
		length: 3 * time.Second,
		camera: kinetograph.Camera{
			Position: r3.Vec{X: -150, Y: -260, Z: 230},
			Target:   r3.Vec{Z: 5},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(45)),
		},
	}
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	var entries []dynamics.BodyState
	add := func(name string, body *decad.Body, role dynamics.BodyRole, pose r3.Transform,
		spin dynamics.QuantityVec, color solidlens.Color) {
		definition := dynamics.RigidBody{Body: body, Role: role, Material: material}
		if role == dynamics.Dynamic {
			definition.Density = &density
		}
		scene.config.Bodies = append(scene.config.Bodies, definition)
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose,
			LinearVelocity: millimetersPerSecond(0, 0, 0), AngularVelocity: spin})
		scene.parts = append(scene.parts, dynamicsPart{name: name, body: body, color: color})
	}
	still := radiansPerSecond(0, 0, 0)

	tray, err := tumbleTray(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("tray: %w", err)
	}
	add("tray", tray, dynamics.Fixed, r3.Identity(), still, navy)
	for i, release := range tumbleBoxReleases {
		body, err := extrudedBox(ctx, scene.doc, -10, -10, 10, 10, -10, 20)
		if err != nil {
			return nil, fmt.Errorf("box %d: %w", i, err)
		}
		pose, err := release.pose()
		if err != nil {
			return nil, fmt.Errorf("box %d: %w", i, err)
		}
		add(fmt.Sprintf("box%d", i), body, dynamics.Dynamic, pose, radiansPerSecond(2, 1, 0), violet)
	}
	for _, shape := range []struct {
		name    string
		build   func() (*decad.Body, error)
		release tumbleRelease
		color   solidlens.Color
	}{
		{"hexagon", func() (*decad.Body, error) { return extrudedPolygon(ctx, scene.doc, tumbleHexagon, 12) },
			tumbleHexRelease, coral},
		{"wedge", func() (*decad.Body, error) { return extrudedPolygon(ctx, scene.doc, tumbleWedge, 8) },
			tumbleWedgeRelease, gold},
		{"tetrahedron", func() (*decad.Body, error) { return stitchedTetrahedron(ctx, scene.doc) },
			tumbleTetrahedronRelease, sky},
	} {
		body, err := shape.build()
		if err != nil {
			return nil, fmt.Errorf("%s: %w", shape.name, err)
		}
		pose, err := shape.release.pose()
		if err != nil {
			return nil, fmt.Errorf("%s: %w", shape.name, err)
		}
		add(shape.name, body, dynamics.Dynamic, pose, still, shape.color)
	}

	scene.config.Step = sceneStepConfig()
	scene.config.Step.PenetrationResidual = units.Millimeters(0.01)
	scene.config.Step.Contact.SupportBand = units.Millimeters(0.005)
	scene.config.Step.MaxPoseEvaluations = 512
	scene.world, err = dynamics.NewWorld(ctx, scene.doc, scene.config)
	if err != nil {
		return nil, fmt.Errorf("world: %w", err)
	}
	scene.start, err = scene.world.NewState(entries)
	if err != nil {
		return nil, fmt.Errorf("start state: %w", err)
	}
	return scene, nil
}

// pose is the release as a transform.
func (r tumbleRelease) pose() (r3.Transform, error) {
	turn, err := r3.Rotation(r.axis, units.Degrees(r.degrees))
	if err != nil {
		return r3.Transform{}, err
	}
	return r3.FromBasis(turn.Basis(), r.at)
}

// tumbleTray is §2's tray: the zero-bound Cut of two source boxes, each
// extruded from the XY plane and then translated, so every crossing of their
// facets lands on a dyadic point.
func tumbleTray(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	outer, err := translatedBox(ctx, doc, -90, -90, 90, 90, -10, 50)
	if err != nil {
		return nil, err
	}
	inner, err := translatedBox(ctx, doc, -80, -80, 80, 80, 0, 50)
	if err != nil {
		return nil, err
	}
	return decad.Cut(ctx, outer, inner)
}

// translatedBox is the source box [x0, x1]×[y0, y1]×[z0, z0+height],
// extruded from the XY plane and placed by a translation along z.
func translatedBox(ctx context.Context, doc *decad.Document, x0, y0, x1, y1, z0, height float64) (*decad.Body, error) {
	body, err := extrudedBox(ctx, doc, x0, y0, x1, y1, 0, height)
	if err != nil || z0 == 0 {
		return body, err
	}
	shift, err := r3.Translation(r3.Vec{Z: z0})
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, shift)
}

// extrudedPolygon is the prism over the closed polygon corners, sketched on
// the XY plane and extruded height along +z.
func extrudedPolygon(ctx context.Context, doc *decad.Document, corners [][2]float64, height float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
	}
	s.Fix(points[0])
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
}

// stitchedTetrahedron stitches the tetrahedron with corners a·(1, 0, 0),
// the origin, a·(0, 1, 0) and a·(1, 0, 1), a = 16·s with s = 1/√2 as r3 holds
// it. Two faces lie on the XY and XZ datum planes and two on the planes
// x + y = a and z = x, whose frames each hold one cardinal axis and the
// diagonal (±s, ±s); the diagonal corners sit at plane-local 16 along it, so
// every corner lands exactly and the stitched solid is exact.
func stitchedTetrahedron(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	probe, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	if err != nil {
		return nil, err
	}
	a := 16 * probe.V().Y
	slanted, err := r3.NewFrame(r3.NewVec(a, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	if err != nil {
		return nil, err
	}
	diagonal, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 1))
	if err != nil {
		return nil, err
	}
	slantedPlane, err := w.CreatePlaneFromFrame(slanted)
	if err != nil {
		return nil, err
	}
	diagonalPlane, err := w.CreatePlaneFromFrame(diagonal)
	if err != nil {
		return nil, err
	}
	faces := make([]*decad.Body, 0, 4)
	for _, face := range []struct {
		plane  *sketch.Plane
		corner [3][2]float64
	}{
		{w.XY(), [3][2]float64{{a, 0}, {0, 0}, {0, a}}},
		{w.XZ(), [3][2]float64{{a, 0}, {0, 0}, {a, a}}},
		{slantedPlane, [3][2]float64{{0, 0}, {0, 16}, {a, 0}}},
		{diagonalPlane, [3][2]float64{{0, 0}, {a, 0}, {0, 16}}},
	} {
		patch, err := trianglePatch(ctx, doc, w, face.plane, face.corner)
		if err != nil {
			return nil, err
		}
		faces = append(faces, patch)
	}
	return decad.Stitch(ctx, faces...)
}

// trianglePatch is the triangle with the given plane-local corners, drawn on
// plane and patched.
func trianglePatch(ctx context.Context, doc *decad.Document, w *sketch.World, plane *sketch.Plane,
	local [3][2]float64) (*decad.Body, error) {
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	var corners [3]*sketch.Point
	for i, p := range local {
		corners[i] = s.CreatePoint(p[0], p[1])
		s.Fix(corners[i])
	}
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%3])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Patch(ctx, s, s.Profiles()[0])
}

// partsBinOmega is the parts-bin cylinder's spin about −x, in rad/s; it rolls
// without slip along +y at partsBinOmega·10 mm/s.
const partsBinOmega = 1.5

// partsBinCylinderStart is the rolling cylinder's axis at release, on its
// side on the tray floor: its 30 mm length spans x = 30…60 and it rolls 60 mm
// in 4 s, its axis ending at y = 20, at least 20 mm inside the walls.
var partsBinCylinderStart = r3.Vec{X: 45, Y: -40, Z: 10}

// partsBinLoftBase and partsBinLoftTop are the loft's sections, on z = 0 and
// z = 16: a square with each side pushed out to a point 8.5 mm from the axis,
// and an octagon, every corner dyadic.
var (
	partsBinLoftBase = [][2]float64{{8.5, 0}, {8, 8}, {0, 8.5}, {-8, 8}, {-8.5, 0}, {-8, -8}, {0, -8.5}, {8, -8}}
	partsBinLoftTop  = [][2]float64{{6, -2.5}, {6, 2.5}, {2.5, 6}, {-2.5, 6}, {-6, 2.5}, {-6, -2.5}, {-2.5, -6}, {2.5, -6}}
)

// partsBinHexagon is the swept hexagon, 20 mm across its flats, centered on
// the origin with dyadic corners.
var partsBinHexagon = [][2]float64{{-11.5, 0}, {-5.75, -10}, {5.75, -10}, {11.5, 0}, {5.75, 10}, {-5.75, 10}}

// partsBinScene is §2's Phase 3 scene: tumble's tray, the scene's one fixed
// body, and every remaining solid payload. Dropped 8 mm onto the floor at
// rest, each at a translation pose at least 20 mm inside the walls: a
// 24×16×16 mm box shelled 2 mm through its top (a non-convex cup), a 12 mm
// cube with its top loop chamfered 2.3 mm (its held mesh displaced by its
// contour's rounding), a loft from a pushed-out square to an octagon, a
// straight 14 mm sweep of a hexagon and a revolved bottle (a Ø16 mm base, a
// quarter-circle shoulder to a Ø8 mm neck, read as a held mesh at HeldChord).
// A Ø20×30 mm source cylinder starts on its side on the floor, rolling along
// +y without slip. Every body takes restitution 0.3 and friction 0.4 at
// density 0.001 kg/mm³, under the tumble step with §2's Phase 3 residuals:
// PenetrationResidual 0.125 mm, SupportBand 0.05 mm, HeldChord 0.03 mm and
// PointResolution 0.1 mm, since a displaced body's witness balls carry its δ.
// The clip is 4 s.
func partsBinScene(ctx context.Context) (*dynamicsScene, error) {
	scene := &dynamicsScene{
		name:   "parts-bin",
		doc:    decad.New(),
		input:  dynamics.StepInput{Gravity: millimetersPerSecondSquared(0, 0, -9810)},
		dt:     units.Seconds(1.0 / 256),
		length: 4 * time.Second,
		camera: kinetograph.Camera{
			Position: r3.Vec{X: -150, Y: -260, Z: 230},
			Target:   r3.Vec{Z: 5},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(45)),
		},
	}
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	var entries []dynamics.BodyState
	add := func(name string, body *decad.Body, role dynamics.BodyRole, at r3.Vec,
		linear, spin dynamics.QuantityVec, color solidlens.Color) error {
		definition := dynamics.RigidBody{Body: body, Role: role, Material: material}
		if role == dynamics.Dynamic {
			definition.Density = &density
		}
		pose, err := r3.Translation(at)
		if err != nil {
			return fmt.Errorf("%s: %w", name, err)
		}
		scene.config.Bodies = append(scene.config.Bodies, definition)
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose, LinearVelocity: linear,
			AngularVelocity: spin})
		scene.parts = append(scene.parts, dynamicsPart{name: name, body: body, color: color})
		return nil
	}
	still, rest := radiansPerSecond(0, 0, 0), millimetersPerSecond(0, 0, 0)

	tray, err := tumbleTray(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("tray: %w", err)
	}
	if err := add("tray", tray, dynamics.Fixed, r3.Vec{}, rest, still, navy); err != nil {
		return nil, err
	}
	for _, shape := range []struct {
		name  string
		build func() (*decad.Body, error)
		at    r3.Vec
		color solidlens.Color
	}{
		{"cup", func() (*decad.Body, error) { return shelledCup(ctx, scene.doc) }, r3.Vec{X: -45, Y: -45, Z: 8}, violet},
		{"block", func() (*decad.Body, error) { return chamferedBlock(ctx, scene.doc) }, r3.Vec{Y: -45, Z: 8}, coral},
		{"loft", func() (*decad.Body, error) { return polygonLoft(ctx, scene.doc) }, r3.Vec{X: -45, Z: 8}, gold},
		{"sweep", func() (*decad.Body, error) { return hexagonSweep(ctx, scene.doc) }, r3.Vec{X: -45, Y: 45, Z: 8}, sky},
		{"bottle", func() (*decad.Body, error) { return revolvedBottle(ctx, scene.doc) }, r3.Vec{Y: 45, Z: 8}, cyan},
	} {
		body, err := shape.build()
		if err != nil {
			return nil, fmt.Errorf("%s: %w", shape.name, err)
		}
		if err := add(shape.name, body, dynamics.Dynamic, shape.at, rest, still, shape.color); err != nil {
			return nil, err
		}
	}
	cylinder, err := sideCylinder(ctx, scene.doc, 10, 30)
	if err != nil {
		return nil, fmt.Errorf("cylinder: %w", err)
	}
	if err := add("cylinder", cylinder, dynamics.Dynamic, partsBinCylinderStart,
		millimetersPerSecond(0, partsBinOmega*10, 0), radiansPerSecond(-partsBinOmega, 0, 0), orange); err != nil {
		return nil, err
	}

	scene.config.Step = sceneStepConfig()
	scene.config.Step.PenetrationResidual = units.Millimeters(0.125)
	scene.config.Step.Contact.SupportBand = units.Millimeters(0.05)
	scene.config.Step.Contact.HeldChord = units.Millimeters(0.03)
	scene.config.Step.Contact.PointResolution = units.Millimeters(0.1)
	scene.config.Step.MaxPoseEvaluations = 512
	scene.world, err = dynamics.NewWorld(ctx, scene.doc, scene.config)
	if err != nil {
		return nil, fmt.Errorf("world: %w", err)
	}
	scene.start, err = scene.world.NewState(entries)
	if err != nil {
		return nil, fmt.Errorf("start state: %w", err)
	}
	return scene, nil
}

// shelledCup is a 24×16×16 mm box on z = 0 shelled 2 mm inward through its
// top cap.
func shelledCup(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	box, err := extrudedBox(ctx, doc, -12, -8, 12, 8, 0, 16)
	if err != nil {
		return nil, err
	}
	return box.Shell(ctx, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(2))
}

// chamferedBlock is a 12 mm cube on z = 0 with its top loop chamfered
// 2.3 mm.
func chamferedBlock(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	box, err := extrudedBox(ctx, doc, -6, -6, 6, 6, 0, 12)
	if err != nil {
		return nil, err
	}
	return box.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(2.3))
}

// fixedPolygon sketches the closed polygon corners on plane, every corner
// fixed.
func fixedPolygon(ctx context.Context, w *sketch.World, plane *sketch.Plane, corners [][2]float64) (*sketch.Sketch,
	*sketch.Profile, error) {
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, nil, err
	}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, nil, err
	}
	if len(s.Profiles()) != 1 {
		return nil, nil, fmt.Errorf("polygon: %d profiles, want 1", len(s.Profiles()))
	}
	return s, s.Profiles()[0], nil
}

// polygonLoft is the 16 mm loft from partsBinLoftBase to partsBinLoftTop.
func polygonLoft(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 16)
	if err != nil {
		return nil, err
	}
	s0, p0, err := fixedPolygon(ctx, w, w.XY(), partsBinLoftBase)
	if err != nil {
		return nil, err
	}
	s1, p1, err := fixedPolygon(ctx, w, top, partsBinLoftTop)
	if err != nil {
		return nil, err
	}
	return doc.Loft(ctx, s0, p0, s1, p1)
}

// hexagonSweep sweeps partsBinHexagon 14 mm up a straight path.
func hexagonSweep(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, p, err := fixedPolygon(ctx, w, w.XY(), partsBinHexagon)
	if err != nil {
		return nil, err
	}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 14)})
	if err != nil {
		return nil, err
	}
	return doc.Sweep(ctx, s, p, path)
}

// revolvedBottle is the full revolve about z of a line-and-arc half-profile
// on the XZ plane: an 8 mm base radius up 14 mm, a quarter-circle shoulder of
// radius 4 to a 4 mm neck radius at z = 18, and the neck to z = 24.
func revolvedBottle(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		return nil, err
	}
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(8, 0)
	c := s.CreatePoint(8, 14)
	center := s.CreatePoint(4, 14)
	d := s.CreatePoint(4, 18)
	e := s.CreatePoint(4, 24)
	f := s.CreatePoint(0, 24)
	for _, p := range []*sketch.Point{a, b, c, center, d, e, f} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateArc(center, c, d)
	s.CreateLine(d, e)
	s.CreateLine(e, f)
	s.CreateLine(f, a)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	if len(s.Profiles()) != 1 {
		return nil, fmt.Errorf("bottle: %d profiles, want 1", len(s.Profiles()))
	}
	return doc.Revolve(s, s.Profiles()[0], decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}},
		decad.FullRevolution{})
}

// sideCylinder is the source cylinder of the given radius and length lying
// on its side, its axis along x and centered on the origin: a rectangle
// revolved a full turn about the x axis.
func sideCylinder(ctx context.Context, doc *decad.Document, radius, length float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	r := s.CreateRectangle(-length/2, 0, length/2, radius)
	s.Fix(r.A)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return doc.Revolve(s, s.Profiles()[0], decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}},
		decad.FullRevolution{})
}
