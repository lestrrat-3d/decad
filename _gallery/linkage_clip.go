package main

import (
	"context"
	"flag"
	"fmt"
	"io"
	"runtime"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// The folding arm's frame rate and lengths. At 64 frames per second the
// 4 s drive spans 256 frames, so frame i shows the drive fraction i/256
// exactly, the grid VerifyLinkage bisects on at linkageResolution. The clip
// holds the drive's end for one more second.
const (
	linkageFPS           = 64
	linkageDriveDuration = 4 * time.Second
	linkageClipLength    = 5 * time.Second
)

// linkageResolution is the floor the folding arm's check refines to: one
// frame of the drive.
const linkageResolution = 1.0 / 256

// The folding arm's geometry, in millimetres. Both links are slot bars with
// semicircular ends of radius linkageEnd: the upper arm's ends sit on the
// shoulder and the elbow, linkageElbow apart, and the forearm's on the elbow
// and its tip, as far again. A pin shaft of radius linkagePin turns in a bore
// linkageClearance wider, and the heads, caps and collars have radius
// linkageHead. The stop's face is the plane y = linkageStopFace from
// x = linkageStopX on, in the forearm's way and beyond the upper arm's reach.
const (
	linkageElbow     = 48.0
	linkageEnd       = 12.0
	linkagePin       = 5.0
	linkageClearance = 0.5
	linkageHead      = 8.0
	linkageStopFace  = 37.5
	linkageStopX     = 64.0
)

// The folding arm's layers along Z, in millimetres: the upper arm's and the
// forearm's undersides, both bars' thickness, and the top of the base plate.
const (
	linkageUpperZ   = 0.0
	linkageForearmZ = 12.0
	linkageBarThick = 8.0
	linkageBaseTop  = -4.0
)

// pinOffset is how far, in millimetres, a pin that rides in a moving bore
// sits off the bore's centre, toward −Y at the zero pose. As the joint turns
// the shaft circles the bore's centre at that radius, so the clearance runs
// between 0.25 and 0.75 mm and never closes. A pin set exactly on its bore's
// centre left VerifyLinkage unable to measure the pair at most poses, where
// the two bodies' float poses put the circles a rounding error off
// concentric: the declared pair then published nothing and the check spent
// its time on the mesh path.
const pinOffset = 0.25

// linkageScene is a mechanism as verified and as filmed: the folding arm, or
// the crank-rocker, whose drive moves a loop and whose frames are posed
// through schedule.
type linkageScene struct {
	doc      *decad.Document
	linkage  *decad.Linkage
	drive    decad.Drive
	shoulder *decad.Link
	elbow    *decad.Link
	schedule *decad.Schedule // nil for a tree linkage
	parts    []linkagePart
	camera   kinetograph.Camera
}

// linkagePart is one body of the scene as kinetograph draws it: link is the
// link it belongs to, nil for a static body.
type linkagePart struct {
	name  string
	body  *decad.Body
	link  *decad.Link
	color solidlens.Color
}

// linkageHitColor is the colour a colliding body turns from the frame of
// VerifyLinkage's first collision on.
var linkageHitColor = coral

// foldingArmScene is a two-link arm on a base plate. The upper arm, a slot
// bar from the shoulder at the origin to the elbow at (48, 0) with ends of
// radius 12, a bore at each end and z ∈ [0, 8], turns about Z through the
// origin from 0° to 90° on a steel post that rises from the base plate: a
// collar under the arm, the shaft through its shoulder bore and a cap over
// it. The forearm, a slot bar from the elbow to (96, 0), z ∈ [12, 20], turns
// about Z through the elbow from 0° to −90°. It carries the elbow pin, so it
// needs no bore: a shaft from 1 mm under the upper arm up through its elbow
// bore, a spacer between the two bars and a cap on the forearm. Every shaft
// clears its bore by at least 0.25 mm (pinOffset), every head, cap and collar
// stands at least 1 mm off the bar it does not belong to, and both
// shaft-in-bore pairs are declared joint contacts.
//
// The two turns cancel, so at the shoulder angle θ = 90°·s the forearm keeps
// its orientation and its upper flank is the line y = 48·sin θ + 12. A stop
// block, x ∈ [64, 124], y ∈ [37.5, 53.5], z ∈ [−4, 26], stands on the base plate
// in the forearm's way and out of the upper arm's reach.
func foldingArmScene(ctx context.Context) (*linkageScene, error) {
	scene := &linkageScene{
		doc: decad.New(),
		camera: kinetograph.Camera{
			Position: r3.Vec{X: 20, Y: -120, Z: 100},
			Target:   r3.Vec{X: 52, Y: 16, Z: 4},
			Up:       r3.Vec{Z: 1},
			FOV:      kinetograph.Constant(units.Degrees(36)),
		},
	}
	upper, err := linkBar{
		p: [2]float64{0, 0}, q: [2]float64{linkageElbow, 0}, radius: linkageEnd,
		bore: linkagePin + linkageClearance, bores: []int{0, 1}, z0: linkageUpperZ, thickness: linkageBarThick,
	}.body(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("upper arm: %w", err)
	}
	forearm, err := linkBar{
		p: [2]float64{linkageElbow, 0}, q: [2]float64{2 * linkageElbow, 0}, radius: linkageEnd,
		z0: linkageForearmZ, thickness: linkageBarThick,
	}.body(ctx, scene.doc)
	if err != nil {
		return nil, fmt.Errorf("forearm: %w", err)
	}
	elbowPin, err := pinStack(ctx, scene.doc, linkageElbow, -pinOffset, linkageUpperZ-1, []pinStep{
		{radius: linkagePin, top: linkageUpperZ + linkageBarThick + 1},
		{radius: linkageHead, top: linkageForearmZ},
	})
	if err != nil {
		return nil, fmt.Errorf("elbow pin: %w", err)
	}
	elbowCap, err := pinStack(ctx, scene.doc, linkageElbow, -pinOffset, linkageForearmZ+linkageBarThick, []pinStep{
		{radius: linkageHead, top: linkageForearmZ + linkageBarThick + 2},
	})
	if err != nil {
		return nil, fmt.Errorf("elbow cap: %w", err)
	}
	post, err := pinStack(ctx, scene.doc, 0, 0, linkageBaseTop, []pinStep{
		{radius: linkageHead + 2, top: linkageUpperZ - 1},
		{radius: linkagePin, top: linkageUpperZ + linkageBarThick + 1},
		{radius: linkageHead, top: linkageForearmZ - 1.5},
	})
	if err != nil {
		return nil, fmt.Errorf("shoulder post: %w", err)
	}
	base, err := extrudedBox(ctx, scene.doc, -20, -20, 128, 56, linkageBaseTop-6, 6)
	if err != nil {
		return nil, fmt.Errorf("base: %w", err)
	}
	stop, err := extrudedBox(ctx, scene.doc, linkageStopX, linkageStopFace, linkageStopX+60, linkageStopFace+16,
		linkageBaseTop, linkageForearmZ+linkageBarThick+6-linkageBaseTop)
	if err != nil {
		return nil, fmt.Errorf("stop: %w", err)
	}

	scene.linkage = decad.NewLinkage()
	scene.shoulder, err = scene.linkage.Ground().Revolute(r3.Vec{}, r3.NewVec(0, 0, 1), []*decad.Body{upper})
	if err != nil {
		return nil, fmt.Errorf("shoulder: %w", err)
	}
	elbowBodies := append([]*decad.Body{forearm}, elbowPin...)
	scene.elbow, err = scene.shoulder.Revolute(r3.NewVec(linkageElbow, 0, 0), r3.NewVec(0, 0, 1),
		append(elbowBodies, elbowCap...))
	if err != nil {
		return nil, fmt.Errorf("elbow: %w", err)
	}
	if err := scene.linkage.DeclareJointContact(upper, post[1]); err != nil {
		return nil, fmt.Errorf("shoulder contact: %w", err)
	}
	if err := scene.linkage.DeclareJointContact(upper, elbowPin[0]); err != nil {
		return nil, fmt.Errorf("elbow contact: %w", err)
	}
	scene.drive = decad.Drive{
		{Link: scene.shoulder, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: scene.elbow, From: units.Degrees(0), To: units.Degrees(-90)},
	}
	scene.parts = []linkagePart{
		{name: "upper-arm", body: upper, link: scene.shoulder, color: violet},
		{name: "forearm", body: forearm, link: scene.elbow, color: gold},
		{name: "base", body: base, color: slate},
		{name: "stop", body: stop, color: navy},
	}
	scene.addHardware("elbow-pin", append(elbowPin, elbowCap...), scene.elbow)
	scene.addHardware("shoulder-post", post, nil)
	return scene, nil
}

// addHardware adds the bodies of one pin, post or bearing as steel parts
// named name-0, name-1 and on, on link, or static when link is nil.
func (s *linkageScene) addHardware(name string, bodies []*decad.Body, link *decad.Link) {
	for i, body := range bodies {
		s.parts = append(s.parts, linkagePart{name: fmt.Sprintf("%s-%d", name, i), body: body, link: link, color: steel})
	}
}

// verify runs VerifyLinkage over the drive at linkageResolution.
func (s *linkageScene) verify(ctx context.Context) (*decad.LinkageReport, error) {
	return s.doc.VerifyLinkage(ctx, s.linkage, s.drive, decad.WithResolution(units.Scalar(linkageResolution)))
}

// linkageFraction is the drive fraction s as a function of the clip's time:
// a linear channel from 0 at t = 0 to 1 at linkageDriveDuration, held at both
// ends, so the clip holds the drive's end. Both times are whole nanoseconds
// and frame i of 64 fps falls at exactly i·15625000 ns, so over the 256-frame
// drive frame i reads the dyadic fraction i/256 exactly.
func linkageFraction() (*kinetograph.Channel, error) {
	return kinetograph.NewChannel(
		kinetograph.Keyframe{At: 0, Value: units.Scalar(0)},
		kinetograph.Keyframe{At: linkageDriveDuration, Value: units.Scalar(1)})
}

// firstFrameAt is the index of the clip's first frame whose drive fraction
// is at or past s, or −1 when no frame reaches s.
func firstFrameAt(clip *kinetograph.Clip, s float64) (int, error) {
	fraction, err := linkageFraction()
	if err != nil {
		return 0, err
	}
	for i := range clip.FrameCount() {
		at, err := fraction.At(clip.FrameTime(i))
		if err != nil {
			return 0, err
		}
		if at.Mag() >= s {
			return i, nil
		}
	}
	return -1, nil
}

// clip builds the kinetograph clip of the drive at linkageFPS over length:
// one driven node per link directly under the rig's root, each link body a
// part on its link's node, each static body a part on the root. When
// report has a collision, its first collision's link body is drawn twice on
// one node, in its own colour up to the frame before the collision's
// fraction and in linkageHitColor from that frame on; the returned style
// switches the two with Fade.
func (s *linkageScene) clip(report *decad.LinkageReport, length time.Duration,
	width, height int) (*kinetograph.Clip, render.Style, error) {
	rig := kinetograph.NewRig()
	scene := kinetograph.NewScene(rig)
	fraction, err := linkageFraction()
	if err != nil {
		return nil, render.Style{}, err
	}
	nodes, err := s.linkNodes(scene, fraction)
	if err != nil {
		return nil, render.Style{}, err
	}
	var hit *decad.LinkCollision
	if report != nil && len(report.Collisions) > 0 {
		hit = &report.Collisions[0]
	}
	style := render.Style{
		Width:             width,
		Height:            height,
		Chord:             units.Millimeters(clipPartChord),
		Background:        backgroundColor,
		Default:           matte(violet),
		Parts:             make(map[string]render.Appearance, len(s.parts)+1),
		DirectionalLights: featureDirectionalLights(),
	}
	var hitParts []linkagePart
	for _, part := range s.parts {
		node := rig.Root()
		if part.link != nil {
			node = nodes[part.link]
		}
		if part.link == nil {
			if err := scene.AddPart(part.name, node, part.body); err != nil {
				return nil, render.Style{}, err
			}
		}
		style.Parts[part.name] = matte(part.color)
		if hit != nil && part.body == hit.A {
			if err := scene.AddPart(part.name+"-hit", node, part.body); err != nil {
				return nil, render.Style{}, err
			}
			hitParts = append(hitParts, part)
		}
	}
	if err := scene.SetCamera(rig.Root(), s.camera); err != nil {
		return nil, render.Style{}, err
	}
	clip, err := kinetograph.NewClip(scene, linkageFPS, length)
	if err != nil {
		return nil, render.Style{}, err
	}
	if len(hitParts) == 0 {
		return clip, style, nil
	}
	before, after, err := hitFades(clip, hit.At.Mag())
	if err != nil {
		return nil, render.Style{}, err
	}
	for _, part := range hitParts {
		own := matte(part.color)
		own.Fade = before
		style.Parts[part.name] = own
		tinted := matte(linkageHitColor)
		tinted.Fade = after
		style.Parts[part.name+"-hit"] = tinted
	}
	return clip, style, nil
}

// linkNodes is one driven node per link directly under the rig's root, with
// every link body added as a part: kinetograph's AddLinkage, reading
// Linkage.PoseAt, for a tree linkage, and its AddSchedule, reading the
// scene's Schedule.PoseAt, for a looped one.
func (s *linkageScene) linkNodes(scene *kinetograph.Scene,
	fraction *kinetograph.Channel) (map[*decad.Link]*kinetograph.Node, error) {
	names := make(map[*decad.Body]string)
	for _, part := range s.parts {
		names[part.body] = part.name
	}
	var linkNodes []*kinetograph.Node
	var err error
	if s.schedule == nil {
		linkNodes, err = scene.AddLinkage(s.linkage, s.drive, fraction, names)
	} else {
		linkNodes, err = scene.AddSchedule(s.schedule, fraction, names)
	}
	if err != nil {
		return nil, err
	}
	links := s.linkage.Links()
	nodes := make(map[*decad.Link]*kinetograph.Node, len(links))
	for i, link := range links {
		nodes[link] = linkNodes[i]
	}
	return nodes, nil
}

// hitFades returns the two Fade channels that switch a part's colour at the
// first frame of clip at or past the drive fraction s. The first is 1 through
// the previous frame and 0 from that frame on, the second the reverse. Each
// channel steps between two adjacent frame times, so every frame draws one
// copy whole and the other not at all. A fraction no frame reaches keeps the
// own colour throughout.
func hitFades(clip *kinetograph.Clip, s float64) (*kinetograph.Channel, *kinetograph.Channel, error) {
	frame, err := firstFrameAt(clip, s)
	if err != nil {
		return nil, nil, err
	}
	switch frame {
	case -1:
		return kinetograph.Constant(units.Scalar(1)), kinetograph.Constant(units.Scalar(0)), nil
	case 0:
		return kinetograph.Constant(units.Scalar(0)), kinetograph.Constant(units.Scalar(1)), nil
	}
	prev, at := clip.FrameTime(frame-1), clip.FrameTime(frame)
	before, err := kinetograph.NewChannel(
		kinetograph.Keyframe{At: prev, Value: units.Scalar(1)},
		kinetograph.Keyframe{At: at, Value: units.Scalar(0)})
	if err != nil {
		return nil, nil, err
	}
	after, err := kinetograph.NewChannel(
		kinetograph.Keyframe{At: prev, Value: units.Scalar(0)},
		kinetograph.Keyframe{At: at, Value: units.Scalar(1)})
	if err != nil {
		return nil, nil, err
	}
	return before, after, nil
}

// linkageOptions are the linkage subcommand's parsed flags.
type linkageOptions struct {
	scene         string
	out           string
	width, height int
	workers       int
	smoke         bool
}

// runLinkage renders a mechanism as a PNG sequence: the folding arm
// (foldingArmScene, the joints and drive of docs/linkage-check-design.md §11
// scene 1) by default, or with -scene rocker the crank-rocker
// (crankRockerScene, the four-bar of §15.10 scene 7 turned a full turn), a
// closed loop posed through a decad.Schedule. It first runs VerifyLinkage over the drive at
// a resolution of 1/256, then films the same drive at 64 frames per second,
// one driven node per link — kinetograph's AddLinkage reading Linkage.PoseAt
// for the arm, its AddSchedule reading the schedule's PoseAt for the rocker — so
// frame i of the drive shows the pose VerifyLinkage evaluates at s = i/256.
// From the frame of the report's first collision on, the colliding body is
// drawn in coral. The drive runs 4 s and the clip holds its end for 1 s more.
//
//	go run . linkage
//
// writes out/linkage_000000.png and on, and prints the verdict, the first
// collision's fraction and the frame it marks.
//
// Flags:
//
//   - -scene arm|rocker picks the mechanism (default arm).
//   - -out <dir> is where the frames go (default "out").
//   - -width and -height set the frame size (default 1280x720).
//   - -workers sets how many frames render at once (default: the CPU count).
//   - -smoke renders the first frame alone at 160x90, after the check.
//
// The frame rate is fixed: any other rate would put frames between the
// check's grid points.
func runLinkage(ctx context.Context, args []string, stderr io.Writer) error {
	var opts linkageOptions
	fs := flag.NewFlagSet("linkage", flag.ContinueOnError)
	fs.StringVar(&opts.scene, "scene", "arm", "the mechanism to film: arm (the folding arm) or rocker (the crank-rocker, a closed loop)")
	fs.StringVar(&opts.out, "out", "out", "directory to write the frames to")
	fs.IntVar(&opts.width, "width", 1280, "frame width in pixels")
	fs.IntVar(&opts.height, "height", 720, "frame height in pixels")
	fs.IntVar(&opts.workers, "workers", runtime.NumCPU(), "frames rendered at once")
	fs.BoolVar(&opts.smoke, "smoke", false, "render the first frame alone at 160x90")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() > 0 {
		return fmt.Errorf("linkage: unexpected argument %q", fs.Arg(0))
	}
	if opts.smoke {
		opts.width, opts.height = smokeWidth, smokeHeight
	}
	if opts.width < 1 || opts.height < 1 || opts.workers < 1 {
		return fmt.Errorf("linkage: -width, -height and -workers must be positive")
	}

	var scene *linkageScene
	var err error
	switch opts.scene {
	case "arm":
		scene, err = foldingArmScene(ctx)
	case "rocker":
		scene, err = crankRockerScene(ctx)
	default:
		return fmt.Errorf("linkage: -scene is arm or rocker, got %q", opts.scene)
	}
	if err != nil {
		return err
	}
	report, err := scene.verify(ctx)
	if err != nil {
		return err
	}
	length := linkageClipLength
	if opts.smoke {
		length = time.Second / linkageFPS
	}
	clip, style, err := scene.clip(report, length, opts.width, opts.height)
	if err != nil {
		return err
	}
	fmt.Fprintf(stderr, "linkage: VerifyLinkage reads %s over %d poses\n", report.Status, len(report.Poses))
	if len(report.Collisions) > 0 {
		first := report.Collisions[0]
		marked, err := firstFrameAt(clip, first.At.Mag())
		if err != nil {
			return err
		}
		fmt.Fprintf(stderr, "linkage: first collision %s/%s at s = %g, drawn from frame %d\n",
			scene.partName(first.A), scene.partName(first.B), first.At.Mag(), marked)
	}
	renderer, err := render.New(ctx, clip, style)
	if err != nil {
		return err
	}
	fmt.Fprintf(stderr, "linkage: rendering %d frames\n", clip.FrameCount())
	seq, err := renderer.Sequence(ctx, opts.out, render.WithWorkers(opts.workers), render.WithPrefix("linkage_"))
	if err != nil {
		return err
	}
	fmt.Fprintf(stderr, "linkage: wrote %d frames to %s\n", seq.Frames, seq.Dir)
	return nil
}

// partName is the name of body's part, or "?" for a body the scene does not
// draw.
func (s *linkageScene) partName(body *decad.Body) string {
	for _, part := range s.parts {
		if part.body == body {
			return part.name
		}
	}
	return "?"
}
