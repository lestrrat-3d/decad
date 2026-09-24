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

const (
	letterFilletRadius = 2.0
	// letterCapChamfer is the setback of the cap-loop bevel every letter's
	// front face carries, well under half the 9mm stroke width so the bevel
	// reads as a lead-in rather than eating the stroke.
	letterCapChamfer = 1.5

	// plateDepth is the backing plate's total thickness before shelling.
	plateDepth = 12.0
	// plateShellThickness is the wall Shell leaves once the plate's
	// camera-facing cap is opened.
	plateShellThickness = 1.8
	// plateHalfDepth is the plate's front face position along Y (Along on
	// its XZ sketch is -Y, so the face nearest the camera sits at -plateHalfDepth).
	plateHalfDepth = plateDepth / 2

	// studRadius is the revolved dome accent's radius.
	studRadius = 6.0
)

type letter struct {
	color  solidlens.Color
	shapes [][][]point
	// chamferCap is true for a letter whose shapes should get a cap-loop
	// chamfer instead of a lateral fillet. It is a per-LETTER choice, not a
	// per-shape one: A's three strokes (aLeft, aRight, aBar) are each a
	// single, hole-free loop, but the crossbar's own rectangle overlaps each
	// leg's trapezoid — both occupy the same X range at crossbar height AND
	// the same Y depth, by the original (pre-hero-rework) design of these
	// three shapes. Two bodies that already coincide there is not itself a
	// defect; it only became visible once each stroke got its OWN
	// independent cap-loop chamfer: the two bevelled cap faces are different
	// sloped surfaces at that shared depth, so whichever one the renderer
	// resolves as farther back shows a sliver of its slope past the nearer
	// one's edge. A lateral fillet never touches the cap face's interior, so
	// it never exposes this — the same reason D (a hole-bearing single
	// shape) keeps a fillet instead of a chamfer.
	chamferCap bool
}

// heroChordTolerance is the chord tolerance the hero shot tessellates at
// unless -chord overrides it.
const heroChordTolerance = 0.02

// heroRender is the README's masthead: the wordmark itself is decad geometry.
func heroRender() imageRender {
	return imageRender{
		rel:      "hero.png",
		settings: solidlens.Settings{Width: 2880, Height: 1620},
		chord:    units.Millimeters(heroChordTolerance),
		scene:    heroScene,
	}
}

// heroScene builds the masthead from five of decad's features rather than
// extrude and fillet alone: the backing plate is a shelled shadow-box whose
// rim carries a real wall thickness; "E" and "C" are extruded and have their
// whole front face bevelled by a cap-loop chamfer, while "D" (an outer+inner
// loop pair) and "A" (three separately overlapping strokes) are extruded and
// have their outside corners filleted instead (see letter.chamferCap and
// extrudeLetterLoops for why each keeps the fillet); and one accent is a
// revolved dome rather than a plain extruded cylinder.
func heroScene(ctx context.Context, chord units.Value) (solidlens.Scene, error) {
	base, err := heroPlate(ctx, chord)
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build backing plate: %w", err)
	}
	models := []solidlens.Model{{Mesh: base, Material: solidlens.Matte(solidlens.RGB(0.015, 0.06, 0.18))}}

	for li, item := range decadLetters() {
		for si, shape := range item.shapes {
			mesh, err := extrudeLetterLoops(ctx, shape, decad.Distance{D: units.Millimeters(14), Dir: decad.Along}, chord, item.chamferCap)
			if err != nil {
				return solidlens.Scene{}, fmt.Errorf("build letter %d shape %d: %w", li, si, err)
			}
			models = append(models, solidlens.Model{Mesh: mesh, Material: solidlens.Matte(item.color)})
		}
	}

	// The left accent stays a plain extruded peg; the right one is a
	// revolved dome, so the masthead shows both an extrude and a revolve at
	// the same small scale.
	peg, err := extrudeLoops(ctx, [][]point{circle(-126, -70, 4.5, 24)}, decad.Distance{
		D: units.Millimeters(8), Dir: decad.Along,
	}, chord)
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build peg accent: %w", err)
	}
	models = append(models, solidlens.Model{Mesh: peg, Material: solidlens.Matte(solidlens.RGB(1, 0.58, 0.08))})

	dome, err := heroStud(ctx, 126, -70, studRadius, chord)
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build dome accent: %w", err)
	}
	models = append(models, solidlens.Model{Mesh: dome, Material: solidlens.Matte(solidlens.RGB(0.1, 0.78, 0.95))})

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

// heroPlate builds the backing plate as a shallow shadow-box: a solid slab
// extruded from the wordmark's outline, then shelled through the cap facing
// the camera so the plate's rim shows a real wall thickness instead of a flat
// backdrop.
func heroPlate(ctx context.Context, chord units.Value) (*decad.Mesh, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		return nil, err
	}
	profile, err := polylineProfile(ctx, s, [][]point{rectangle(-151, -84, 151, 84)})
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	slab, err := decad.New().Extrude(s, profile, decad.Symmetric{D: units.Millimeters(plateDepth)}) //nolint:contextcheck
	if err != nil {
		return nil, err
	}
	// Facing(0,-1,0) is the cap nearest the camera (Along on this XZ sketch
	// is -Y); opening it turns the slab into a shadow-box whose side walls
	// and back keep a uniform 1.8mm thickness.
	shelled, err := slab.Shell(ctx, decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Millimeters(plateShellThickness))
	if err != nil {
		return nil, fmt.Errorf("shell backing plate: %w", err)
	}
	return shelled.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
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
		{color: solidlens.RGB(0.05, 0.85, 0.96), shapes: [][][]point{place(dOuter, dInner)}, chamferCap: false},
		{color: solidlens.RGB(0.18, 0.47, 1), shapes: [][][]point{place(e)}, chamferCap: true},
		{color: solidlens.RGB(0.58, 0.24, 1), shapes: [][][]point{place(c)}, chamferCap: true},
		{color: solidlens.RGB(1, 0.25, 0.2), shapes: placeShapes([][]point{aLeft}, [][]point{aRight}, [][]point{aBar}), chamferCap: false},
		{color: solidlens.RGB(1, 0.68, 0.08), shapes: [][][]point{place(dOuter, dInner)}, chamferCap: false},
	}
}

func extrudeLoops(ctx context.Context, loops [][]point, extent decad.Extent, chord units.Value) (*decad.Mesh, error) {
	w := sketch.NewWorld()
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
	// This mesh is drawn, never proven: VerifyNone skips the facet-contact
	// audit's own work ceiling, which a fine chord tolerance can otherwise hit.
	return body.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
}

// extrudeLetterLoops extrudes one letter shape, then either fillets its
// outside vertical corners (chamferCap false) or bevels its whole front face
// with a cap-loop chamfer (chamferCap true) — the caller decides per LETTER,
// not per shape; see letter.chamferCap for why D and A both keep the fillet.
// Chamfering D's own cap loop (an outer+inner loop pair) panics decad's own
// cap-blend tessellator regardless (it indexes past a band slice,
// decad-side), so D's shape could not take a chamfer even loop-by-loop.
// Fillet-then-chamfer on a hole-free shape was tried too — it does not
// panic, but every setback tried (0.3mm through 1.5mm) hit "the offset
// changes the section's topology; a trimmed-offset kernel is not available"
// on at least one letter, so a chamfered shape goes straight from extrude to
// chamfer with no fillet.
func extrudeLetterLoops(
	ctx context.Context, loops [][]point, extent decad.Extent, chord units.Value, chamferCap bool,
) (*decad.Mesh, error) {
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
	extruded, err := decad.New().Extrude(s, profile, extent) //nolint:contextcheck
	if err != nil {
		return nil, err
	}
	if !chamferCap {
		// Round the exposed outside corners while keeping the counters and
		// interior cut-ins crisp for a legible wordmark.
		filleted, err := extruded.Fillet(ctx, decad.Edges(
			decad.ParallelTo(r3.NewVec(0, 1, 0)),
			decad.Convex(),
		), units.Millimeters(letterFilletRadius))
		if err != nil {
			return nil, fmt.Errorf("fillet extruded loops: %w", err)
		}
		return filleted.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
	}
	beveled, err := extruded.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(extruded))), units.Millimeters(letterCapChamfer))
	if err != nil {
		return nil, fmt.Errorf("chamfer letter cap: %w", err)
	}
	// These meshes are drawn, never proven: VerifyNone skips the facet-contact
	// audit's own work ceiling, which a fine chord tolerance can otherwise hit.
	return beveled.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
}

// heroStud revolves a quarter-disc profile — an on-axis base point, its rim,
// and the arc between the rim and the on-axis tip — into a solid dome
// standing on the plate's front face at plane-local (x, z). Full 360°, the
// profile's own arc is a quarter circle, so the sweep is a hemisphere rather
// than surfaceShot's full-sphere dish wall.
func heroStud(ctx context.Context, x, z, radius float64, chord units.Value) (*decad.Mesh, error) {
	w := sketch.NewWorld()
	// Offsetting the YZ datum by x puts the sketch's own U axis on world Y,
	// with the plane positioned at world X=x — the axis this stud revolves
	// about needs to run along Y, not the vertical Z axis revolveShot and
	// surfaceShot both revolve about.
	plane, err := w.CreateOffsetPlane(w.YZ(), x)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	const base = -plateHalfDepth // flush with the shelled plate's open front
	center := s.CreatePoint(base, z)
	s.Fix(center)
	rim := s.CreatePoint(base, z+radius)
	tip := s.CreatePoint(base-radius, z)
	s.CreateLine(center, rim)
	s.CreateArc(center, rim, tip)
	s.CreateLine(tip, center)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: z}, End: decad.Point2{U: 1, V: z}}
	// Revolve has no context-aware variant; Solve above is the cancellable phase.
	dome, err := decad.New().Revolve(s, profile, axis, decad.FullRevolution{}) //nolint:contextcheck
	if err != nil {
		return nil, fmt.Errorf("revolve the stud: %w", err)
	}
	return dome.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
}
