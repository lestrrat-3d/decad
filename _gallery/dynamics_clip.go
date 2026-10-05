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
	"stack-and-drop": stackAndDropScene,
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

	scene.config.Step = dynamics.StepConfig{
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
//   - -scene <name> picks the scene (required; stack-and-drop).
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
