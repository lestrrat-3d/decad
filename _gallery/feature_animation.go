package main

import (
	"context"
	"flag"
	"fmt"
	"image/png"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"slices"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

const (
	featureAnimationSteps      = 24
	featureAnimationHoldFrames = 12
	featureAnimationFPS        = 12
	sweepAnimationSteps        = 48
)

// runFeatureAnimations renders each feature's intermediate decad bodies and
// assembles a looping GIF. The final body holds long enough to inspect it.
func runFeatureAnimations(ctx context.Context, args []string) error {
	fs := flag.NewFlagSet("features", flag.ContinueOnError)
	only := fs.String("only", "", "comma-separated feature names (default: all)")
	out := fs.String("out", "", "GIF directory (default: docs/images/features)")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() > 0 {
		return fmt.Errorf("features: unexpected argument %q", fs.Arg(0))
	}
	renders := slices.DeleteFunc(featureRenders(), func(render imageRender) bool {
		return render.name() == "verify"
	})
	selected, err := selectRenders(renders, *only)
	if err != nil {
		return err
	}
	root := *out
	if root == "" {
		images, err := imagesRoot("")
		if err != nil {
			return err
		}
		root = filepath.Join(images, "features")
	}
	if err := os.MkdirAll(root, 0o755); err != nil {
		return err
	}
	for _, render := range selected {
		if err := renderFeatureAnimation(ctx, render.name(), root); err != nil {
			return fmt.Errorf("feature %s: %w", render.name(), err)
		}
	}
	return nil
}

func renderFeatureAnimation(ctx context.Context, name, root string) error {
	frames := filepath.Join("out", "features", name)
	if err := os.MkdirAll(frames, 0o755); err != nil {
		return err
	}
	steps := featureAnimationSteps
	if name == "sweep" {
		steps = sweepAnimationSteps
	}
	for stage := range steps {
		models, err := featureAnimationModels(ctx, name, stage, units.Millimeters(0.08))
		if err != nil {
			return fmt.Errorf("stage %d: %w", stage, err)
		}
		scene := featureScene(models...)
		for i := range scene.Models {
			edge := edgeColor
			if name == "boolean" && i > 0 {
				edge.A *= scene.Models[i].Material.Color.A
			}
			scene.Models[i].Edges = solidlens.Outline(edge)
		}
		path := filepath.Join(frames, fmt.Sprintf("%s_%03d.png", name, stage))
		file, err := os.Create(path) //nolint:gosec
		if err != nil {
			return err
		}
		if name == "boolean" && len(models) > 1 {
			err = renderBooleanFrame(ctx, file, scene)
		} else {
			err = solidlens.RenderPNG(ctx, file, scene, solidlens.Settings{Width: 480, Height: 360})
		}
		closeErr := file.Close()
		if err != nil {
			return err
		}
		if closeErr != nil {
			return closeErr
		}
	}
	finalPath := filepath.Join(frames, fmt.Sprintf("%s_%03d.png", name, steps-1))
	finalPNG, err := os.ReadFile(finalPath) //nolint:gosec
	if err != nil {
		return err
	}
	for frame := steps; frame < steps+featureAnimationHoldFrames; frame++ {
		path := filepath.Join(frames, fmt.Sprintf("%s_%03d.png", name, frame))
		if err := os.WriteFile(path, finalPNG, 0o644); err != nil {
			return err
		}
	}
	input := filepath.Join(frames, name+"_%03d.png")
	output := filepath.Join(root, name+".gif")
	filter := "[0:v]split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];" +
		"[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle[v]"
	cmd := exec.CommandContext(ctx, "ffmpeg", "-y", "-loglevel", "error", "-framerate",
		strconv.Itoa(featureAnimationFPS), "-i", input,
		"-filter_complex", filter, "-map", "[v]", "-frames:v",
		strconv.Itoa(steps+featureAnimationHoldFrames), "-loop", "0", output) //nolint:gosec
	if ffmpegOutput, err := cmd.CombinedOutput(); err != nil {
		return fmt.Errorf("encode GIF: %w: %s", err, ffmpegOutput)
	}
	fmt.Fprintf(os.Stderr, "%s: %s\n", name, output)
	return nil
}

// renderBooleanFrame blends the cutters over the plate so the holes remain
// visible while all three tools pass through them.
func renderBooleanFrame(ctx context.Context, writer io.Writer, scene solidlens.Scene) error {
	settings := solidlens.Settings{Width: 480, Height: 360}
	plateScene := scene
	plateScene.Models = scene.Models[:1]
	plate, err := solidlens.Render(ctx, plateScene, settings)
	if err != nil {
		return err
	}
	toolScene := scene
	toolScene.Models = scene.Models[1:]
	toolScene.Background = solidlens.Color{}
	tools, err := solidlens.Render(ctx, toolScene, settings)
	if err != nil {
		return err
	}
	for offset := 0; offset < len(plate.Pix); offset += 4 {
		alpha := 0.45 * float64(tools.Pix[offset+3]) / 255
		for channel := range 3 {
			plate.Pix[offset+channel] = uint8(float64(plate.Pix[offset+channel])*(1-alpha) +
				float64(tools.Pix[offset+channel])*alpha + 0.5)
		}
	}
	return png.Encode(writer, plate)
}

func featureAnimationModels(ctx context.Context, name string, stage int, chord units.Value) ([]solidlens.Model, error) {
	switch name {
	case "extrude":
		w := sketch.NewWorld()
		body, err := prism(ctx, decad.New(), w, w.XY(), float64(stage+1)*44/featureAnimationSteps,
			[]point{{-40, -28}, {40, -28}, {40, -8}, {-20, -8}, {-20, 28}, {-40, 28}})
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, cyan, chord)
	case "revolve":
		body, err := ringBodyAtAngle(ctx, float64(stage+1)*360/featureAnimationSteps)
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, blue, chord)
	case "sweep":
		const (
			riseSteps       = 6
			firstTurnSteps  = 18
			runSteps        = 6
			secondTurnSteps = 18
		)
		rise, firstTurn, run, secondTurn := 20.0, 0.0, 0.0, 0.0
		switch {
		case stage < riseSteps:
			rise = float64(stage+1) * 20 / riseSteps
		case stage < riseSteps+firstTurnSteps:
			firstTurn = float64(stage-riseSteps+1) * 90 / firstTurnSteps
		case stage < riseSteps+firstTurnSteps+runSteps:
			firstTurn = 90
			run = float64(stage-riseSteps-firstTurnSteps+1) * 30 / runSteps
		default:
			firstTurn = 90
			run = 30
			secondTurn = float64(stage-riseSteps-firstTurnSteps-runSteps+1) * 90 / secondTurnSteps
		}
		body, err := curvedDuct(ctx, rise, firstTurn, run, secondTurn)
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, cyan, chord)
	case "loft":
		body, err := loftDuctAtHeight(ctx, float64(stage+1)*46/featureAnimationSteps)
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, violet, chord)
	case "freeform":
		const profileSteps = 10
		height := 1.5
		profileScale := float64(stage+1) / profileSteps
		if stage >= profileSteps {
			profileScale = 1
			height += float64(stage-profileSteps+1) * (30 - height) / (featureAnimationSteps - profileSteps)
		}
		body, err := bladeBodyAtShape(ctx, height, profileScale)
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, coral, chord)
	case "fillet", "chamfer", "cap-chamfer":
		body, err := lateralEdgePlate(ctx)
		if err != nil {
			return nil, err
		}
		color := coral
		if name == "chamfer" {
			color = gold
		}
		if name == "cap-chamfer" {
			color = cyan
		}
		if stage > 0 {
			size := float64(stage) / (featureAnimationSteps - 1)
			switch name {
			case "fillet":
				body, err = body.Fillet(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))),
					units.Millimeters(16*size))
			case "chamfer":
				body, err = body.Chamfer(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))),
					units.Millimeters(16*size))
			case "cap-chamfer":
				body, err = body.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(body))),
					units.Millimeters(0.6+9.4*float64(stage-1)/(featureAnimationSteps-2)))
			}
			if err != nil {
				return nil, err
			}
		}
		return oneModel(ctx, body, color, chord)
	case "shell":
		body, err := trayBodyAtThickness(ctx, 28-float64(stage)*21/(featureAnimationSteps-1))
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, blue, chord)
	case "boolean":
		return booleanAnimationModels(ctx, stage, chord)
	case "surface":
		body, err := dishBodyAtAngle(ctx, float64(stage+1)*150/featureAnimationSteps)
		if err != nil {
			return nil, err
		}
		return twoSidedModel(ctx, body, violet, gold, chord)
	default:
		return nil, fmt.Errorf("unknown feature %q", name)
	}
}

// booleanAnimationModels holds all three cutters in place, switches the plate
// to its fully cut result in one frame, then fades the cutters away.
func booleanAnimationModels(ctx context.Context, stage int, chord units.Value) ([]solidlens.Model, error) {
	const (
		cutFrame   = 8
		fadeEnd    = 16
		toolBottom = -holeClearance
		toolLength = 48.0
	)

	shape := flangeShape{height: flangeThickness}
	if stage >= cutFrame {
		shape = throughFlange()
	}
	plate, err := flangeBody(ctx, shape)
	if err != nil {
		return nil, err
	}
	models, err := oneModel(ctx, plate, violet, chord)
	if err != nil || stage >= fadeEnd {
		return models, err
	}

	w := sketch.NewWorld()
	doc := decad.New()
	plane, err := w.CreateOffsetPlane(w.XY(), toolBottom)
	if err != nil {
		return nil, err
	}
	opacity := 1.0
	if stage > cutFrame {
		opacity = float64(fadeEnd-stage) / (fadeEnd - cutFrame)
	}
	for _, d := range drills {
		tool, err := cylinder(ctx, doc, w, plane, point{d.x, 0}, d.radius,
			decad.Distance{D: units.Millimeters(toolLength), Dir: decad.Along})
		if err != nil {
			return nil, err
		}
		toolModels, err := oneModel(ctx, tool, gold, chord)
		if err != nil {
			return nil, err
		}
		toolModels[0].Material.Color.A = opacity
		models = append(models, toolModels...)
	}
	return models, nil
}
