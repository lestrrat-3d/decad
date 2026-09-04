package main

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

const letterFilletRadius = 2.0

type letter struct {
	color  solidlens.Color
	shapes [][][]point
}

// heroRender is the README's masthead: the wordmark itself is decad geometry,
// each letter an extruded profile with its outside corners filleted.
func heroRender() imageRender {
	return imageRender{
		rel:      "docs/images/hero.png",
		settings: solidlens.Settings{Width: 1440, Height: 810},
		scene:    heroScene,
	}
}

func heroScene(ctx context.Context) (solidlens.Scene, error) {
	base, err := extrudeLoops(ctx, [][]point{rectangle(-151, -84, 151, 84)}, decad.Symmetric{D: units.Millimeters(3)})
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build backing plate: %w", err)
	}
	models := []solidlens.Model{{Mesh: base, Material: solidlens.Matte(solidlens.RGB(0.015, 0.06, 0.18))}}

	for _, item := range decadLetters() {
		for _, shape := range item.shapes {
			mesh, err := extrudeLetterLoops(ctx, shape, decad.Distance{D: units.Millimeters(14), Dir: decad.Along})
			if err != nil {
				return solidlens.Scene{}, fmt.Errorf("build letter: %w", err)
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
			return solidlens.Scene{}, fmt.Errorf("build accent: %w", err)
		}
		models = append(models, solidlens.Model{Mesh: mesh, Material: solidlens.Matte(accent.color)})
	}

	return solidlens.Scene{
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
		Background: backgroundColor,
	}, nil
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
	return extrudeLoopsWithFillet(ctx, loops, extent, 0)
}

func extrudeLetterLoops(ctx context.Context, loops [][]point, extent decad.Extent) (*decad.Mesh, error) {
	return extrudeLoopsWithFillet(ctx, loops, extent, letterFilletRadius)
}

func extrudeLoopsWithFillet(ctx context.Context, loops [][]point, extent decad.Extent, filletRadius float64) (*decad.Mesh, error) {
	w := sketch.NewWorld()
	// XZ makes the wordmark face the camera; extrusion then gives each stroke
	// depth along Y without relying on a steep viewing angle.
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		return nil, err
	}
	profile, err := polylineProfile(ctx, s, loops)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	body, err := decad.New().Extrude(s, profile, extent) //nolint:contextcheck
	if err != nil {
		return nil, err
	}
	if filletRadius > 0 {
		// Round the exposed outside corners while keeping the counters and
		// interior cut-ins crisp for a legible wordmark.
		body, err = body.FilletContext(ctx, decad.Edges(
			decad.ParallelTo(r3.NewVec(0, 1, 0)),
			decad.Convex(),
		), units.Millimeters(filletRadius))
		if err != nil {
			return nil, fmt.Errorf("fillet extruded loops: %w", err)
		}
	}
	return body.TessellateContext(ctx, units.Millimeters(0.4))
}
