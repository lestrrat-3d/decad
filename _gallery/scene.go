package main

import "github.com/lestrrat-3d/solidlens"

// backgroundColor is the pale blue-grey every gallery image sits on, so the
// hero and the feature thumbnails read as one piece.
var backgroundColor = solidlens.RGB(0.82, 0.86, 0.93)

// The hero's letter palette, reused one colour per feature row so a reader can
// tell two neighbouring thumbnails apart at a glance.
var (
	cyan   = solidlens.RGB(0.05, 0.85, 0.96)
	blue   = solidlens.RGB(0.18, 0.47, 1)
	violet = solidlens.RGB(0.58, 0.24, 1)
	coral  = solidlens.RGB(1, 0.25, 0.2)
	gold   = solidlens.RGB(1, 0.68, 0.08)
)

// featureScene frames one feature shot. Every shot shares this camera, so the
// parts are drawn at one scale from one viewpoint and the table compares them
// directly; the lighting and background are the hero's.
func featureScene(models ...solidlens.Model) solidlens.Scene {
	return solidlens.Scene{
		Camera: solidlens.Camera{
			Position: solidlens.Vec{X: 105, Y: -165, Z: 110},
			Target:   solidlens.Vec{Z: 12},
			Up:       solidlens.Vec{Z: 1},
			FOV:      30,
		},
		Models: models,
		DirectionalLights: []solidlens.DirectionalLight{
			{
				Direction: solidlens.Vec{X: -0.7, Y: 0.12, Z: -1},
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
			Position:  solidlens.Vec{X: -140, Y: -190, Z: 240},
			Color:     solidlens.RGB(0.45, 0.75, 1),
			Intensity: 3000,
		}},
		Background: backgroundColor,
	}
}
