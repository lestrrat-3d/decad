// Command gallery renders decad's README images with SolidLens.
//
// It lives in its own module so that SolidLens stays out of the decad library's
// dependency list. Run it from this directory with `go run .`; it writes the
// hero image and every feature-table thumbnail under the repository's
// docs/images.
package main

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"runtime"

	"github.com/lestrrat-3d/solidlens"
)

// imageRender is one checked-in image: where it lives in the repository, how
// large the raster is, and how to build the scene it holds.
type imageRender struct {
	rel      string
	settings solidlens.Settings
	scene    func(context.Context) (solidlens.Scene, error)
}

func main() {
	if err := run(context.Background()); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// run renders every image the README references: the hero wordmark first, then
// one thumbnail per feature-table row.
func run(ctx context.Context) error {
	renders := append([]imageRender{heroRender()}, featureRenders()...)
	for _, render := range renders {
		if err := render.write(ctx); err != nil {
			return fmt.Errorf("render %s: %w", render.rel, err)
		}
	}
	return nil
}

func (r imageRender) write(ctx context.Context) error {
	scene, err := r.scene(ctx)
	if err != nil {
		return err
	}
	out, err := outputPath(r.rel)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(out), 0o755); err != nil {
		return fmt.Errorf("create image directory: %w", err)
	}
	file, err := os.Create(out) //nolint:gosec
	if err != nil {
		return fmt.Errorf("create output: %w", err)
	}
	err = solidlens.RenderPNG(ctx, file, scene, r.settings)
	closeErr := file.Close()
	if err != nil {
		return fmt.Errorf("render: %w", err)
	}
	if closeErr != nil {
		return fmt.Errorf("close output: %w", closeErr)
	}
	return nil
}

// outputPath resolves rel against the repository root rather than the working
// directory. This module lives at <repo>/_gallery, so the root is the parent of
// this source file's own directory, and every image lands in the repository no
// matter where the command was started from.
func outputPath(rel string) (string, error) {
	_, self, _, ok := runtime.Caller(0)
	if !ok || !filepath.IsAbs(self) {
		return "", fmt.Errorf("cannot locate this command's source file; build without -trimpath")
	}
	return filepath.Join(filepath.Dir(filepath.Dir(self)), filepath.FromSlash(rel)), nil
}
