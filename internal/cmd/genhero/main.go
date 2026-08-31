// Command genhero renders decad's README hero image with SolidLens.
package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

const outputPath = "docs/images/hero.png"

type point struct {
	x, y float64
}

type letter struct {
	color  solidlens.Color
	shapes [][][]point
}

func main() {
	if err := run(context.Background()); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(ctx context.Context) error {
	base, err := extrudeLoops(ctx, [][]point{rectangle(-151, -84, 151, 84)}, decad.Symmetric{D: units.Millimeters(3)})
	if err != nil {
		return fmt.Errorf("build backing plate: %w", err)
	}
	models := []solidlens.Model{{Mesh: base, Material: solidlens.Matte(solidlens.RGB(0.015, 0.06, 0.18))}}

	for _, item := range decadLetters() {
		for _, shape := range item.shapes {
			mesh, err := extrudeLoops(ctx, shape, decad.Distance{D: units.Millimeters(14), Dir: decad.Along})
			if err != nil {
				return fmt.Errorf("build letter: %w", err)
			}
			models = append(models, solidlens.Model{Mesh: mesh, Material: solidlens.Matte(item.color)})
		}
	}

	for _, accent := range []struct {
		x, y, radius float64
		color        solidlens.Color
	}{
		{-126, -70, 4.5, solidlens.RGB(1, 0.58, 0.08)},
		{126, -70, 4.5, solidlens.RGB(0.1, 0.78, 0.95)},
	} {
		mesh, err := extrudeLoops(ctx, [][]point{circle(accent.x, accent.y, accent.radius, 24)}, decad.Distance{
			D: units.Millimeters(8), Dir: decad.Along,
		})
		if err != nil {
			return fmt.Errorf("build accent: %w", err)
		}
		models = append(models, solidlens.Model{Mesh: mesh, Material: solidlens.Matte(accent.color)})
	}

	if err := os.MkdirAll(filepath.Dir(outputPath), 0o755); err != nil {
		return fmt.Errorf("create image directory: %w", err)
	}
	file, err := os.Create(outputPath) //nolint:gosec
	if err != nil {
		return fmt.Errorf("create output: %w", err)
	}
	scene := solidlens.Scene{
		Camera: solidlens.Camera{
			Position: solidlens.Vec{X: 18, Y: -360, Z: 68},
			Target:   solidlens.Vec{Z: 4},
			Up:       solidlens.Vec{Z: 1},
			FOV:      30,
		},
		Models: models,
		DirectionalLights: []solidlens.DirectionalLight{
			{
				Direction: solidlens.Vec{X: -0.7, Y: 0.35, Z: -1},
				Color:     solidlens.RGB(1, 1, 1),
				Intensity: 1.25,
			},
			{
				Direction: solidlens.Vec{X: 0.6, Y: -0.2, Z: -0.6},
				Color:     solidlens.RGB(0.25, 0.55, 1),
				Intensity: 0.4,
			},
		},
		PointLights: []solidlens.PointLight{{
			Position:  solidlens.Vec{X: -90, Y: -135, Z: 160},
			Color:     solidlens.RGB(0.45, 0.75, 1),
			Intensity: 1800,
		}},
		Background: solidlens.RGB(0.82, 0.86, 0.93),
	}
	err = solidlens.RenderPNG(ctx, file, scene, solidlens.Settings{Width: 1440, Height: 810})
	closeErr := file.Close()
	if err != nil {
		return fmt.Errorf("render: %w", err)
	}
	if closeErr != nil {
		return fmt.Errorf("close output: %w", closeErr)
	}
	return nil
}

func decadLetters() []letter {
	const (
		width  = 45.0
		height = 120.0
		stroke = 9.0
		gap    = 9.0
	)
	x := -130.0
	placeLoops := func(loops [][]point) [][]point {
		placed := make([][]point, len(loops))
		for i, loop := range loops {
			placed[i] = make([]point, len(loop))
			for j, p := range loop {
				placed[i][j] = point{x: p.x + x, y: p.y}
			}
		}
		return placed
	}
	place := func(loops ...[]point) [][]point {
		placed := placeLoops(loops)
		x += width + gap
		return placed
	}
	placeShapes := func(shapes ...[][]point) [][][]point {
		placed := make([][][]point, len(shapes))
		for i, shape := range shapes {
			placed[i] = placeLoops(shape)
		}
		x += width + gap
		return placed
	}

	dOuter := []point{
		{0, -height / 2}, {width - stroke, -height / 2}, {width, -height/2 + stroke},
		{width, height/2 - stroke}, {width - stroke, height / 2}, {0, height / 2},
	}
	dInner := []point{
		{stroke, -height/2 + stroke}, {stroke, height/2 - stroke},
		{width - 2*stroke, height/2 - stroke}, {width - 2*stroke, -height/2 + stroke},
	}
	e := []point{
		{0, -height / 2}, {width, -height / 2}, {width, -height/2 + stroke},
		{stroke, -height/2 + stroke}, {stroke, -stroke / 2}, {width - stroke, -stroke / 2},
		{width - stroke, stroke / 2}, {stroke, stroke / 2}, {stroke, height/2 - stroke},
		{width, height/2 - stroke}, {width, height / 2}, {0, height / 2},
	}
	c := []point{
		{width, height / 2}, {stroke, height / 2}, {0, height/2 - stroke},
		{0, -height/2 + stroke}, {stroke, -height / 2}, {width, -height / 2},
		{width, -height/2 + stroke}, {2 * stroke, -height/2 + stroke},
		{stroke, -height/2 + 2*stroke}, {stroke, height/2 - 2*stroke},
		{2 * stroke, height/2 - stroke}, {width, height/2 - stroke},
	}
	aLeft := []point{{0, -height / 2}, {stroke, -height / 2}, {width/2 + stroke/2, height / 2}, {width/2 - stroke/2, height / 2}}
	aRight := []point{{width - stroke, -height / 2}, {width, -height / 2}, {width/2 + stroke/2, height / 2}, {width/2 - stroke/2, height / 2}}
	aBar := []point{{stroke, -stroke / 2}, {width - stroke, -stroke / 2}, {width - stroke, stroke / 2}, {stroke, stroke / 2}}

	return []letter{
		{color: solidlens.RGB(0.05, 0.85, 0.96), shapes: [][][]point{place(dOuter, dInner)}},
		{color: solidlens.RGB(0.18, 0.47, 1), shapes: [][][]point{place(e)}},
		{color: solidlens.RGB(0.58, 0.24, 1), shapes: [][][]point{place(c)}},
		{color: solidlens.RGB(1, 0.25, 0.2), shapes: placeShapes([][]point{aLeft}, [][]point{aRight}, [][]point{aBar})},
		{color: solidlens.RGB(1, 0.68, 0.08), shapes: [][][]point{place(dOuter, dInner)}},
	}
}

func extrudeLoops(ctx context.Context, loops [][]point, extent decad.Extent) (*decad.Mesh, error) {
	w := sketch.NewWorld()
	// XZ makes the wordmark face the camera; extrusion then gives each stroke
	// depth along Y without relying on a steep viewing angle.
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		return nil, err
	}
	for loopIndex, loop := range loops {
		if len(loop) < 3 {
			return nil, fmt.Errorf("loop %d has fewer than three points", loopIndex)
		}
		points := make([]*sketch.Point, len(loop))
		for i, p := range loop {
			points[i] = s.CreatePoint(p.x, p.y)
		}
		if loopIndex == 0 {
			s.Fix(points[0])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes)+1 == len(loops) {
			profile = candidate
			break
		}
	}
	if profile == nil {
		return nil, fmt.Errorf("sketch produced no profile for %d loops", len(loops))
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	body, err := decad.New().Extrude(s, profile, extent) //nolint:contextcheck
	if err != nil {
		return nil, err
	}
	return body.TessellateContext(ctx, units.Millimeters(0.4))
}

func rectangle(x0, y0, x1, y1 float64) []point {
	return []point{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
}

func circle(cx, cy, radius float64, segments int) []point {
	points := make([]point, segments)
	for i := range points {
		angle := 2 * math.Pi * float64(i) / float64(segments)
		points[i] = point{cx + radius*math.Cos(angle), cy + radius*math.Sin(angle)}
	}
	return points
}
