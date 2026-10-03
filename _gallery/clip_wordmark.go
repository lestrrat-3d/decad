package main

import (
	"context"
	"fmt"
	"math"
	"strconv"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// Each letter starts at (0, -letterOffsetY, letterOffsetZ) off its landed
// pose, above the frame and in front of the plate, and moves straight to it.
const (
	letterOffsetY = 80.0
	letterOffsetZ = 200.0
)

// letterTravel is the length of a letter's path, from (0, -80, 200) off its
// landed pose to the pose itself.
var letterTravel = math.Hypot(letterOffsetY, letterOffsetZ)

// The word lamp is a white point light that starts at wordLampStart, left of
// the plate and 46 mm in front of the letters' front faces (y = -14 mm), and
// slides wordLampTravel millimetres along +X. At wordLampIntensity it adds
// about 2.4 to a letter face it passes.
var wordLampStart = r3.NewVec(-170, -60, 40)

const (
	wordLampTravel    = 340.0
	wordLampIntensity = 5000.0
)

// wordLamp is the name of act C's moving light.
const wordLamp = "lamp.word"

// letterTrack is the name of letter i's descent track.
func letterTrack(i int) string {
	return "word." + strconv.Itoa(i) + ".in"
}

// wordmarkTake is act C: the hero's shelled plate, peg and dome on root, and
// its five letters, which drop in one after another while the camera creeps
// closer.
func wordmarkTake(ctx context.Context, ch *Channels) (*Take, error) {
	rig := kinetograph.NewRig()
	scene := kinetograph.NewScene(rig)
	parts := map[string]render.Appearance{}

	plate, err := heroPlate(ctx)
	if err != nil {
		return nil, fmt.Errorf("backing plate: %w", err)
	}
	if err := scene.AddPart("plate", rig.Root(), plate); err != nil {
		return nil, err
	}
	parts["plate"] = matte(navy)

	for i, l := range decadLetters() {
		node, err := letterNode(rig.Root(), ch, i)
		if err != nil {
			return nil, fmt.Errorf("letter %d: %w", i, err)
		}
		for j, shape := range l.shapes {
			body, err := letterBody(ctx, shape, l.chamferCap)
			if err != nil {
				return nil, fmt.Errorf("letter %d shape %d: %w", i, j, err)
			}
			name := "letter." + strconv.Itoa(i) + "." + strconv.Itoa(j)
			if err := scene.AddPart(name, node, body); err != nil {
				return nil, err
			}
			parts[name] = matte(l.color)
		}
	}

	peg, err := heroPeg(ctx)
	if err != nil {
		return nil, fmt.Errorf("peg: %w", err)
	}
	if err := scene.AddPart("peg", rig.Root(), peg); err != nil {
		return nil, err
	}
	parts["peg"] = matte(orange)

	dome, err := heroStud(ctx, 126, -70, studRadius)
	if err != nil {
		return nil, fmt.Errorf("dome: %w", err)
	}
	if err := scene.AddPart("dome", rig.Root(), dome); err != nil {
		return nil, err
	}
	parts["dome"] = matte(sky)

	lamp, err := addWordLamp(scene, rig.Root(), ch)
	if err != nil {
		return nil, fmt.Errorf("word lamp: %w", err)
	}
	if err := setWordCamera(scene, rig.Root(), ch); err != nil {
		return nil, fmt.Errorf("camera: %w", err)
	}
	return &Take{Scene: scene, Style: wordmarkStyle(parts, map[string]render.LightAppearance{wordLamp: lamp})}, nil
}

// addWordLamp hangs the word lamp off root -> Fixed translation to
// wordLampStart -> Prismatic +X (lamp.word.slide) and returns its look:
// white, at the intensity of track lamp.word.intensity.
func addWordLamp(scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels) (render.LightAppearance, error) {
	slide, err := ch.Get("lamp.word.slide")
	if err != nil {
		return render.LightAppearance{}, err
	}
	intensity, err := ch.Get("lamp.word.intensity")
	if err != nil {
		return render.LightAppearance{}, err
	}
	start, err := r3.Translation(wordLampStart)
	if err != nil {
		return render.LightAppearance{}, err
	}
	at, err := root.Fixed(start)
	if err != nil {
		return render.LightAppearance{}, err
	}
	node, err := at.Prismatic(r3.NewVec(1, 0, 0), slide)
	if err != nil {
		return render.LightAppearance{}, err
	}
	if err := scene.AddLight(wordLamp, node, kinetograph.Light{Kind: kinetograph.PointLight}); err != nil {
		return render.LightAppearance{}, err
	}
	return whiteLamp(intensity), nil
}

// letterNode is root -> Fixed translation (0, -80, 200) -> Prismatic along
// (0, 80, -200) by letter i's track, which ends at that vector's length.
func letterNode(root *kinetograph.Node, ch *Channels, i int) (*kinetograph.Node, error) {
	start, err := r3.Translation(r3.NewVec(0, -letterOffsetY, letterOffsetZ))
	if err != nil {
		return nil, err
	}
	above, err := root.Fixed(start)
	if err != nil {
		return nil, err
	}
	in, err := ch.Get(letterTrack(i))
	if err != nil {
		return nil, err
	}
	return above.Prismatic(r3.NewVec(0, letterOffsetY, -letterOffsetZ), in)
}

// setWordCamera puts the hero camera on root -> Prismatic along its view
// direction by track word.dolly.
func setWordCamera(scene *kinetograph.Scene, root *kinetograph.Node, ch *Channels) error {
	dolly, err := ch.Get("word.dolly")
	if err != nil {
		return err
	}
	node, err := root.Prismatic(heroCameraTarget.Sub(heroCameraPosition), dolly)
	if err != nil {
		return err
	}
	return scene.SetCamera(node, kinetograph.Camera{
		Position: heroCameraPosition,
		Target:   heroCameraTarget,
		Up:       r3.NewVec(0, 0, 1),
		FOV:      kinetograph.Constant(units.Degrees(30)),
	})
}
