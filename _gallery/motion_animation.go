package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"time"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// The README's motion GIFs are motionWidth×motionHeight, the 16:9 frame the
// linkage and dynamics cameras are set for.
const (
	motionWidth  = 480
	motionHeight = 270
)

// motionClip is one GIF of the README's "Motion and collisions" section: a
// kinetograph clip of a verified mechanism or a certified dynamics timeline.
// The GIF keeps every stride-th frame of the clip, starting at frame 0, and
// holds the last kept frame for hold.
type motionClip struct {
	name   string
	build  func(ctx context.Context, width, height int) (*kinetograph.Clip, render.Style, error)
	stride int
	hold   time.Duration
}

// motionClips are the GIFs `go run . motion` writes, in README order. Each
// caption in the README states only what the scene's tests assert:
//
//   - arm: TestLinkageClipTurnsAtTheStop (linkage_clip_test.go).
//   - rocker: TestLinkageLoopClipTurnsAtTheStop and
//     TestLinkageLoopClipMatchesSchedule (linkage_loop_clip_test.go).
//   - tumble: TestTumbleTimeline (dynamics_clip_test.go).
//   - parts-bin: TestPartsBinTimeline (dynamics_clip_test.go).
//
// The linkage clips run at 64 fps and each GIF keeps every fourth frame, so
// each keeps the first and last frames of the hold at the turnaround;
// TestMotionClipsKeepTheTurnaround checks this.
var motionClips = []motionClip{
	{name: "arm", build: linkageMotion(foldingArmScene, 0), stride: 4},
	{name: "rocker", build: linkageMotion(crankRockerScene, 0), stride: 4},
	{name: "tumble", build: dynamicsMotion("tumble", 30), stride: 1, hold: time.Second},
	{name: "parts-bin", build: dynamicsMotion("parts-bin", 27), stride: 1, hold: time.Second},
}

// motionDynamicsFPS is the rate the dynamics GIFs sample their timelines at:
// every eighth step of 1/256 s.
const motionDynamicsFPS = 32

// motionFOV narrows camera's field of view to degrees, so the scene fills
// the GIF's small frame from the same viewpoint. Zero keeps the camera's own.
func motionFOV(camera *kinetograph.Camera, degrees float64) {
	if degrees != 0 {
		camera.FOV = kinetograph.Constant(units.Degrees(degrees))
	}
}

// linkageMotion builds a linkage scene, runs VerifyLinkage over its drive and
// films it as the linkage subcommand does: out to the turnaround the check
// proves clear and back, the first collision's bodies flashing coral at the
// turnaround. fov is passed to motionFOV.
func linkageMotion(build func(context.Context) (*linkageScene, error),
	fov float64) func(context.Context, int, int) (*kinetograph.Clip, render.Style, error) {
	return func(ctx context.Context, width, height int) (*kinetograph.Clip, render.Style, error) {
		scene, err := build(ctx)
		if err != nil {
			return nil, render.Style{}, err
		}
		motionFOV(&scene.camera, fov)
		report, err := scene.verify(ctx)
		if err != nil {
			return nil, render.Style{}, err
		}
		turn, err := turnaroundOf(report)
		if err != nil {
			return nil, render.Style{}, err
		}
		clip, style, _, err := scene.clip(turn, linkageClipLength, width, height)
		return clip, style, err
	}
}

// dynamicsMotion builds the dynamics scene registered as name, advances its
// timeline to the clip length and films it at motionDynamicsFPS. A timeline
// that stops early is an error: the GIF shows only certified poses. fov is
// passed to motionFOV.
func dynamicsMotion(name string, fov float64) func(context.Context, int, int) (*kinetograph.Clip, render.Style, error) {
	return func(ctx context.Context, width, height int) (*kinetograph.Clip, render.Style, error) {
		build, ok := dynamicsScenes[name]
		if !ok {
			return nil, render.Style{}, fmt.Errorf("no dynamics scene %q", name)
		}
		scene, err := build(ctx)
		if err != nil {
			return nil, render.Style{}, err
		}
		motionFOV(&scene.camera, fov)
		timeline, err := scene.advance(ctx)
		if err != nil {
			return nil, render.Style{}, err
		}
		if err := scene.stopError(timeline); err != nil {
			return nil, render.Style{}, err
		}
		clipScene, err := scene.clipScene(timeline)
		if err != nil {
			return nil, render.Style{}, err
		}
		clip, err := kinetograph.NewClip(clipScene, motionDynamicsFPS, scene.length)
		if err != nil {
			return nil, render.Style{}, err
		}
		return clip, scene.style(width, height), nil
	}
}

// runMotionAnimations writes the README's motion GIFs under
// docs/images/motion: for each clip it renders the PNG frames under
// out/motion/<name>, then has ffmpeg keep every stride-th frame, hold the
// last, and encode a looping GIF with a 96-colour palette, as the features
// subcommand does.
//
//	go run . motion
//
// Flags:
//
//   - -only <names> writes just the named GIFs, comma separated (arm,
//     rocker, tumble, parts-bin).
//   - -out <dir> writes the GIFs there instead of docs/images/motion.
//   - -workers sets how many frames render at once (default: the CPU count).
func runMotionAnimations(ctx context.Context, args []string) error {
	fs := flag.NewFlagSet("motion", flag.ContinueOnError)
	only := fs.String("only", "", "comma-separated GIF names (default: all): "+strings.Join(motionClipNames(), ", "))
	out := fs.String("out", "", "GIF directory (default: docs/images/motion)")
	workers := fs.Int("workers", runtime.NumCPU(), "frames rendered at once")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() > 0 {
		return fmt.Errorf("motion: unexpected argument %q", fs.Arg(0))
	}
	if *workers < 1 {
		return fmt.Errorf("motion: -workers must be positive")
	}
	selected, err := selectMotionClips(*only)
	if err != nil {
		return err
	}
	root := *out
	if root == "" {
		images, err := imagesRoot("")
		if err != nil {
			return err
		}
		root = filepath.Join(images, "motion")
	}
	if err := os.MkdirAll(root, 0o755); err != nil {
		return err
	}
	for _, mc := range selected {
		if err := renderMotionAnimation(ctx, mc, root, *workers); err != nil {
			return fmt.Errorf("motion %s: %w", mc.name, err)
		}
	}
	return nil
}

// selectMotionClips filters motionClips to the comma-separated names in only;
// an empty only keeps them all, and an unknown name is an error.
func selectMotionClips(only string) ([]motionClip, error) {
	if only == "" {
		return motionClips, nil
	}
	var selected []motionClip
	for name := range strings.SplitSeq(only, ",") {
		name = strings.TrimSpace(name)
		found := false
		for _, mc := range motionClips {
			if mc.name == name {
				selected = append(selected, mc)
				found = true
				break
			}
		}
		if !found {
			return nil, fmt.Errorf("unknown motion GIF %q; valid names are %s", name, strings.Join(motionClipNames(), ", "))
		}
	}
	return selected, nil
}

func motionClipNames() []string {
	names := make([]string, len(motionClips))
	for i, mc := range motionClips {
		names[i] = mc.name
	}
	return names
}

func renderMotionAnimation(ctx context.Context, mc motionClip, root string, workers int) error {
	clip, style, err := mc.build(ctx, motionWidth, motionHeight)
	if err != nil {
		return err
	}
	if clip.FPS()%mc.stride != 0 {
		return fmt.Errorf("stride %d does not divide the clip's %d fps", mc.stride, clip.FPS())
	}
	renderer, err := render.New(ctx, clip, style)
	if err != nil {
		return err
	}
	frames := filepath.Join("out", "motion", mc.name)
	seq, err := renderer.Sequence(ctx, frames, render.WithWorkers(workers), render.WithPrefix(mc.name+"_"))
	if err != nil {
		return err
	}
	fps := clip.FPS() / mc.stride
	filter := fmt.Sprintf("[0:v]select='not(mod(n\\,%d))',setpts=N/(%d*TB)", mc.stride, fps)
	if mc.hold > 0 {
		filter += fmt.Sprintf(",tpad=stop_mode=clone:stop_duration=%g", mc.hold.Seconds())
	}
	filter += ",split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];" +
		"[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle[v]"
	output := filepath.Join(root, mc.name+".gif")
	cmd := exec.CommandContext(ctx, "ffmpeg", "-y", "-loglevel", "error",
		"-framerate", strconv.Itoa(clip.FPS()), "-i", filepath.Join(seq.Dir, seq.Pattern),
		"-filter_complex", filter, "-map", "[v]", "-r", strconv.Itoa(fps), "-loop", "0", output) //nolint:gosec
	if ffmpegOutput, err := cmd.CombinedOutput(); err != nil {
		return fmt.Errorf("encode GIF: %w: %s", err, ffmpegOutput)
	}
	fmt.Fprintf(os.Stderr, "%s: %s\n", mc.name, output)
	return nil
}
