package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
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
	selected, err := selectRenders(featureRenders(), *only)
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
	for stage := range featureAnimationSteps {
		models, err := featureAnimationModels(ctx, name, stage, units.Millimeters(0.08))
		if err != nil {
			return fmt.Errorf("stage %d: %w", stage, err)
		}
		scene := featureScene(models...)
		for i := range scene.Models {
			scene.Models[i].Edges = solidlens.Outline(edgeColor)
		}
		path := filepath.Join(frames, fmt.Sprintf("%s_%03d.png", name, stage))
		file, err := os.Create(path) //nolint:gosec
		if err != nil {
			return err
		}
		err = solidlens.RenderPNG(ctx, file, scene, solidlens.Settings{Width: 480, Height: 360})
		closeErr := file.Close()
		if err != nil {
			return err
		}
		if closeErr != nil {
			return closeErr
		}
	}
	finalPath := filepath.Join(frames, fmt.Sprintf("%s_%03d.png", name, featureAnimationSteps-1))
	finalPNG, err := os.ReadFile(finalPath) //nolint:gosec
	if err != nil {
		return err
	}
	for frame := featureAnimationSteps; frame < featureAnimationSteps+featureAnimationHoldFrames; frame++ {
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
		strconv.Itoa(featureAnimationSteps+featureAnimationHoldFrames), "-loop", "0", output) //nolint:gosec
	if ffmpegOutput, err := cmd.CombinedOutput(); err != nil {
		return fmt.Errorf("encode GIF: %w: %s", err, ffmpegOutput)
	}
	fmt.Fprintf(os.Stderr, "%s: %s\n", name, output)
	return nil
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
		body, err := mitredDuct(ctx, float64(stage+1)*120/featureAnimationSteps)
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
		body, err := bladeBodyAtHeight(ctx, float64(stage+1)*30/featureAnimationSteps)
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
		shape := flangeShape{height: flangeThickness, depths: map[string]float64{}}
		for i, drill := range drills {
			start := i * featureAnimationSteps / len(drills)
			local := stage - start
			switch {
			case local >= featureAnimationSteps/len(drills)-1:
				shape.depths[drill.name] = flangeThickness + holeClearance
			case local > 0:
				shape.depths[drill.name] = float64(local) * 2.3
			}
		}
		body, err := flangeBody(ctx, shape)
		if err != nil {
			return nil, err
		}
		return oneModel(ctx, body, violet, chord)
	case "verify":
		return verifyAnimationModels(ctx, stage, chord)
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

func verifyAnimationModels(ctx context.Context, stage int, chord units.Value) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	plate, err := prism(ctx, doc, w, w.XY(), 16, rectangle(-44, -38, 44, 38))
	if err != nil {
		return nil, err
	}
	if stage > 0 {
		depth := float64(min(stage, 7)) * 2.3
		if stage >= 8 {
			depth = flangeThickness + holeClearance
		}
		bore, err := holeTool(ctx, doc, w, drill{name: "fit", radius: 22}, 16, depth)
		if err != nil {
			return nil, err
		}
		plate, err = decad.Cut(ctx, plate, bore)
		if err != nil {
			return nil, err
		}
	}
	models, err := oneModel(ctx, plate, gold, chord)
	if err != nil || stage < 8 {
		return models, err
	}
	pin, err := verifyPin(ctx, doc, w)
	if err != nil {
		return nil, err
	}
	travel := 80 * float64(featureAnimationSteps-1-stage) / (featureAnimationSteps - 9)
	if travel > 0 {
		translation, err := r3.Translation(r3.NewVec(0, 0, travel))
		if err != nil {
			return nil, err
		}
		pin, err = pin.PlacedCopy(ctx, translation)
		if err != nil {
			return nil, err
		}
	}
	part, err := oneModel(ctx, pin, cyan, chord)
	return append(models, part...), err
}
