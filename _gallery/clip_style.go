package main

import (
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// edgeColor is the outline every clip part is drawn with.
var edgeColor = solidlens.RGB(0.08, 0.08, 0.12)

// clipPartChord is the chord tolerance, in millimetres, of the flange and the
// shelf parts. The wordmark tessellates at heroChordTolerance.
const clipPartChord = 0.05

// matte is a part drawn in color with the clip's outline.
func matte(color solidlens.Color) render.Appearance {
	return render.Appearance{Material: solidlens.Matte(color), Edges: solidlens.Outline(edgeColor)}
}

// whiteLamp is a moving light's look: white, at intensity over time.
// solidlens reads a light colour as its luminance only, so a coloured lamp
// would light the parts exactly as a dimmer white one does.
func whiteLamp(intensity *kinetograph.Channel) render.LightAppearance {
	return render.LightAppearance{Color: solidlens.RGB(1, 1, 1), Intensity: intensity}
}

// buildStyle is act A's look: the feature thumbnails' background and lights,
// the plate violet and the tools and pin gold. lamps are its moving lights.
func buildStyle(parts map[string]render.Appearance, lamps map[string]render.LightAppearance) render.Style {
	return render.Style{
		Chord:             units.Millimeters(clipPartChord),
		Background:        backgroundColor,
		Default:           matte(violet),
		Parts:             parts,
		DirectionalLights: featureDirectionalLights(),
		PointLights:       featurePointLights(),
		Lights:            lamps,
	}
}

// shapesStyle is act B's look. It leaves out the feature point light: solidlens
// divides a point light by the squared distance in millimetres, so at the
// shelf's distances, 336 mm and more, it adds under 0.03 to any face, and
// only to the parts nearest it.
func shapesStyle(parts map[string]render.Appearance) render.Style {
	return render.Style{
		Chord:             units.Millimeters(clipPartChord),
		Background:        backgroundColor,
		Default:           matte(blue),
		Parts:             parts,
		DirectionalLights: featureDirectionalLights(),
	}
}

// wordmarkStyle is act C's look: the hero's lights and chord, plus lamps,
// its moving lights.
func wordmarkStyle(parts map[string]render.Appearance, lamps map[string]render.LightAppearance) render.Style {
	return render.Style{
		Chord:             units.Millimeters(heroChordTolerance),
		Background:        backgroundColor,
		Default:           matte(navy),
		Parts:             parts,
		DirectionalLights: heroDirectionalLights(),
		PointLights:       heroPointLights(),
		Lights:            lamps,
	}
}
