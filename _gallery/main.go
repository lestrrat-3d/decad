// Command gallery renders decad's README images with SolidLens, and its
// landing-page clip with kinetograph.
//
// It lives in its own module so that SolidLens and kinetograph stay out of
// the decad library's dependency list. Run it from this directory with `go run .`; with
// no flags it writes the still hero and every feature-table thumbnail under
// the repository's docs/images, at each shot's own chord tolerance and size,
// exactly as committed.
//
// The features subcommand renders each animated feature at successive
// construction stages and writes looping GIFs under docs/images/features.
// Verify uses a static thumbnail. The subcommand requires ffmpeg and accepts
// -only and -out flags.
//
// Flags let one invocation try a shot at a different tolerance or size
// without touching the committed images:
//
//   - -chord <mm> overrides the chord tolerance for every shot rendered.
//     Unset, each shot keeps its own default.
//   - -scale <n> multiplies both raster dimensions of every shot.
//   - -only <names> renders just the named shots, comma separated (see
//     -list for the names).
//   - -out <dir> writes under that directory instead of the repository's
//     docs/images, keeping the same relative layout beneath it.
//   - -list prints each shot's name, default chord tolerance and default
//     size, then exits without rendering.
//
// Run `go run . -h` for the full flag reference.
//
// The hero subcommand renders the README's animated hero frames and prints
// the ffmpeg command that writes docs/images/hero.gif.
//
// The clip subcommand renders the landing-page clip instead: `go run . clip`
// animates the boolean plate, selected feature parts, a curved duct and the
// hero wordmark with kinetograph, writes each shot's PNG frames under out/,
// and prints the two ffmpeg commands that assemble them into
// out/decad-landing.mp4 and out/decad-landing.gif, so
// `go run . clip > assemble.sh && sh assemble.sh` writes both videos. Its own
// flags (-out, -width, -height, -fps, -workers, -only, -smoke, -probe) follow
// the subcommand; runClip documents them and the shots. The flags above do
// not apply to it.
//
// The dynamics subcommand films one exit scene of the multibody dynamics
// program: `go run . dynamics -scene stack-and-drop` (or `-scene tumble`, or
// `-scene parts-bin`) advances the scene's certified dynamics.Timeline to the
// clip length and renders it with one kinetograph driven node per body under
// out/. Its flags (-scene, -out, -fps, -width, -height, -workers, -smoke)
// follow the subcommand; runDynamics documents them.
//
// The linkage subcommand films a mechanism decad verifies: `go run . linkage`
// runs Document.VerifyLinkage over a two-link folding arm swinging into a
// wall, then renders the same drive with one kinetograph driven node per link
// under out/, the colliding link turning coral from the frame of the first
// collision the check proves. Its flags (-out, -width, -height, -workers,
// -smoke) follow the subcommand; runLinkage documents them.
//
// The motion subcommand writes the GIFs of the README's "Motion and
// collisions" section under docs/images/motion: `go run . motion` films the
// folding arm, the crank-rocker and the tumble and parts-bin dynamics scenes
// at 480x270, then has ffmpeg encode each as a looping GIF. It requires
// ffmpeg and accepts -only, -out and -workers; runMotionAnimations documents
// them.
package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"sort"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// maxRasterDimension is solidlens's own limit on Settings.Width and
// Settings.Height. scaledSettings enforces it up front so -scale fails with a
// gallery-level message instead of solidlens's own refusal.
const maxRasterDimension = 16384

// imageRender is one checked-in image: where it lives beneath docs/images,
// how large the raster is by default, the chord tolerance it tessellates at
// by default, and how to build the scene it holds at a given tolerance.
type imageRender struct {
	rel      string
	settings solidlens.Settings
	chord    units.Value
	scene    func(context.Context, units.Value) (solidlens.Scene, error)
}

// name is the shot's identity for -only and -list: the base of rel with its
// extension removed, e.g. "hero" or "surface".
func (r imageRender) name() string {
	base := filepath.Base(r.rel)
	return strings.TrimSuffix(base, filepath.Ext(base))
}

func (r imageRender) write(ctx context.Context, chord units.Value, settings solidlens.Settings, root string) error {
	scene, err := r.scene(ctx, chord)
	if err != nil {
		return err
	}
	for i := range scene.Models {
		scene.Models[i].Edges = solidlens.Outline(edgeColor)
	}
	out := filepath.Join(root, filepath.FromSlash(r.rel))
	if err := os.MkdirAll(filepath.Dir(out), 0o755); err != nil {
		return fmt.Errorf("create image directory: %w", err)
	}
	file, err := os.Create(out) //nolint:gosec
	if err != nil {
		return fmt.Errorf("create output: %w", err)
	}
	err = solidlens.RenderPNG(ctx, file, scene, settings)
	closeErr := file.Close()
	if err != nil {
		return fmt.Errorf("render: %w", err)
	}
	if closeErr != nil {
		return fmt.Errorf("close output: %w", closeErr)
	}
	return nil
}

func main() {
	if err := run(context.Background()); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// run renders the hero still and each feature thumbnail. A first argument of
// "hero", "clip", "features", "dynamics", "linkage" or "motion" renders the
// corresponding animation instead.
func run(ctx context.Context) error {
	if len(os.Args) > 1 && os.Args[1] == "motion" {
		return runMotionAnimations(ctx, os.Args[2:])
	}
	if len(os.Args) > 1 && os.Args[1] == "linkage" {
		return runLinkage(ctx, os.Args[2:], os.Stderr)
	}
	if len(os.Args) > 1 && os.Args[1] == "dynamics" {
		return runDynamics(ctx, os.Args[2:], os.Stderr)
	}
	if len(os.Args) > 1 && os.Args[1] == "clip" {
		return runClip(ctx, os.Args[2:], os.Stdout)
	}
	if len(os.Args) > 1 && os.Args[1] == "hero" {
		return runHero(ctx, os.Args[2:], os.Stdout)
	}
	if len(os.Args) > 1 && os.Args[1] == "features" {
		return runFeatureAnimations(ctx, os.Args[2:])
	}
	flag.Usage = func() {
		out := flag.CommandLine.Output()
		fmt.Fprintln(out, "Usage: go run . [flags]")
		fmt.Fprintln(out, "       go run . clip [clip flags]   (go run . clip -h lists the clip flags)")
		fmt.Fprintln(out, "       go run . hero [hero flags]   (go run . hero -h lists the hero flags)")
		fmt.Fprintln(out, "       go run . features [feature flags]   (go run . features -h lists the feature flags)")
		fmt.Fprintln(out, "       go run . dynamics -scene <name> [dynamics flags]   (go run . dynamics -h lists them)")
		fmt.Fprintln(out, "       go run . linkage [linkage flags]   (go run . linkage -h lists them)")
		fmt.Fprintln(out, "       go run . motion [motion flags]   (go run . motion -h lists them)")
		fmt.Fprintln(out, "Flags:")
		flag.PrintDefaults()
	}
	var chordOverride *units.Value
	flag.Func("chord", "override the chord tolerance in mm for every shot rendered (default: each shot's own tolerance)",
		func(s string) error {
			v, err := strconv.ParseFloat(s, 64)
			if err != nil {
				return fmt.Errorf("not a number: %w", err)
			}
			mm := units.Millimeters(v)
			chordOverride = &mm
			return nil
		})
	scale := flag.Float64("scale", 1, "multiply both raster dimensions of every shot by this factor")
	only := flag.String("only", "", "comma-separated shot names to render (default: all); see -list for the names")
	outDir := flag.String("out", "", "write images under this directory instead of the repository's docs/images")
	list := flag.Bool("list", false, "print each shot's name, default chord tolerance and default size, then exit")
	flag.Parse()
	if flag.NArg() > 0 {
		return fmt.Errorf("unexpected argument %q; the subcommands are clip, hero, features, dynamics, linkage and motion, and one must come first", flag.Arg(0))
	}

	renders := append([]imageRender{heroRender()}, featureRenders()...)

	if *list {
		printRenderList(renders)
		return nil
	}

	selected, err := selectRenders(renders, *only)
	if err != nil {
		return err
	}

	root, err := imagesRoot(*outDir)
	if err != nil {
		return err
	}

	for _, render := range selected {
		chord := render.chord
		if chordOverride != nil {
			chord = *chordOverride
		}
		settings, err := scaledSettings(render.settings, *scale)
		if err != nil {
			return fmt.Errorf("%s: %w", render.name(), err)
		}
		if err := render.write(ctx, chord, settings, root); err != nil {
			return fmt.Errorf("render %s: %w", render.rel, err)
		}
	}
	return nil
}

// selectRenders filters renders to the comma-separated names in only. An
// empty only keeps every render. A name that matches no render is an error
// naming the valid names, rather than a silent skip.
func selectRenders(renders []imageRender, only string) ([]imageRender, error) {
	if only == "" {
		return renders, nil
	}
	byName := make(map[string]imageRender, len(renders))
	valid := make([]string, 0, len(renders))
	for _, r := range renders {
		byName[r.name()] = r
		valid = append(valid, r.name())
	}
	sort.Strings(valid)

	names := strings.Split(only, ",")
	selected := make([]imageRender, 0, len(names))
	for _, n := range names {
		n = strings.TrimSpace(n)
		render, ok := byName[n]
		if !ok {
			return nil, fmt.Errorf("unknown shot %q; valid names are %s", n, strings.Join(valid, ", "))
		}
		selected = append(selected, render)
	}
	return selected, nil
}

// scaledSettings multiplies settings' raster dimensions by scale, and refuses
// a scale that would push either dimension past solidlens's own limit rather
// than letting solidlens refuse deeper in.
func scaledSettings(settings solidlens.Settings, scale float64) (solidlens.Settings, error) {
	width := int(math.Round(float64(settings.Width) * scale))
	height := int(math.Round(float64(settings.Height) * scale))
	if width > maxRasterDimension || height > maxRasterDimension {
		return solidlens.Settings{}, fmt.Errorf(
			"scale %g renders %dx%d, past solidlens's %d-pixel limit", scale, width, height, maxRasterDimension)
	}
	return solidlens.Settings{Width: width, Height: height}, nil
}

// printRenderList prints one line per render: its name, default chord
// tolerance and default raster size.
func printRenderList(renders []imageRender) {
	for _, r := range renders {
		fmt.Printf("%-12s chord=%-8s size=%dx%d\n", r.name(), r.chord, r.settings.Width, r.settings.Height)
	}
}

// imagesRoot is the directory every render's rel is resolved against. With
// outDir empty it resolves against the repository root via runtime.Caller,
// so it ignores the working directory the command was started from; this
// module lives at <repo>/_gallery, so the root is the parent of this source
// file's own directory. A non-empty outDir replaces the repository's
// docs/images outright, so -out never touches the committed images.
func imagesRoot(outDir string) (string, error) {
	if outDir != "" {
		return outDir, nil
	}
	_, self, _, ok := runtime.Caller(0)
	if !ok || !filepath.IsAbs(self) {
		return "", fmt.Errorf("cannot locate this command's source file; build without -trimpath")
	}
	return filepath.Join(filepath.Dir(filepath.Dir(self)), "docs", "images"), nil
}
