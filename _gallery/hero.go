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

	// The swept rail reuses sweepShot's own proven path shape (see
	// heroRailModels) exactly, moved by translation alone — a rotation- and
	// scale-free shift keeps every tangent match Sweep's P5 join gate found
	// exact. Only the rendered profile is drawn narrower than sweepShot's own
	// 12mm square (railProfileHalf below): the profile's size plays no part
	// in Sweep's own path validation, and a narrower rail is what clears both
	// the letters' top (Z=60) and the plate's own top edge (Z=84) alongside
	// the path's own 20mm rise. The rail sits in the gap above "C" and "A" as
	// a corner accent rather than spanning the full width.
	railShiftX      = 75.0
	railShiftZ      = 52.0
	railProfileHalf = 0.8

	// studRadius is the revolved dome accent's radius.
	studRadius = 6.0
)

type letter struct {
	color  solidlens.Color
	shapes [][][]point
}

// heroChordTolerance is the chord tolerance the hero shot tessellates at
// unless -chord overrides it.
const heroChordTolerance = 0.02

// railColor is the swept rail's accent color, distinct from every letter and
// from the two remaining boss accents.
var railColor = solidlens.RGB(0.74, 0.78, 0.86)

// heroRender is the README's masthead: the wordmark itself is decad geometry.
func heroRender() imageRender {
	return imageRender{
		rel:      "hero.png",
		settings: solidlens.Settings{Width: 2880, Height: 1620},
		chord:    units.Millimeters(heroChordTolerance),
		scene:    heroScene,
	}
}

// heroScene builds the masthead from six of decad's features rather than
// extrude and fillet alone: the backing plate is a shelled shadow-box whose
// rim carries a real wall thickness; every hole-free letter (E, C and A's
// three strokes) is extruded and has its whole front face bevelled by a
// cap-loop chamfer, while D's outer+inner loop pair is extruded and has its
// outside corners filleted instead (see extrudeLetterLoops for why the two
// treatments differ); a swept rail — the same composite path sweepShot
// proves, bending through more than one plane — sits in the gap above the
// wordmark; and one accent is a revolved dome rather than a plain extruded
// cylinder.
func heroScene(ctx context.Context, chord units.Value) (solidlens.Scene, error) {
	base, err := heroPlate(ctx, chord)
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build backing plate: %w", err)
	}
	models := []solidlens.Model{{Mesh: base, Material: solidlens.Matte(solidlens.RGB(0.015, 0.06, 0.18))}}

	for li, item := range decadLetters() {
		for si, shape := range item.shapes {
			mesh, err := extrudeLetterLoops(ctx, shape, decad.Distance{D: units.Millimeters(14), Dir: decad.Along}, chord)
			if err != nil {
				return solidlens.Scene{}, fmt.Errorf("build letter %d shape %d: %w", li, si, err)
			}
			models = append(models, solidlens.Model{Mesh: mesh, Material: solidlens.Matte(item.color)})
		}
	}

	railModels, err := heroRailModels(ctx, chord)
	if err != nil {
		return solidlens.Scene{}, fmt.Errorf("build rail: %w", err)
	}
	models = append(models, railModels...)

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
		{color: solidlens.RGB(0.05, 0.85, 0.96), shapes: [][][]point{place(dOuter, dInner)}},
		{color: solidlens.RGB(0.18, 0.47, 1), shapes: [][][]point{place(e)}},
		{color: solidlens.RGB(0.58, 0.24, 1), shapes: [][][]point{place(c)}},
		{color: solidlens.RGB(1, 0.25, 0.2), shapes: placeShapes([][]point{aLeft}, [][]point{aRight}, [][]point{aBar})},
		{color: solidlens.RGB(1, 0.68, 0.08), shapes: [][][]point{place(dOuter, dInner)}},
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
// outside vertical corners (a shape with an inner loop, i.e. D's outer+inner
// pair) or bevels its whole front face with a cap-loop chamfer (every other
// shape). The two treatments are mutually exclusive here, not by design
// choice: chamfering D's cap loop panics decad's own cap-blend tessellator
// (it indexes past a band slice, decad-side), so D keeps the plain filleted
// cap instead of triggering it. Fillet-then-chamfer on the hole-free shapes
// was tried too — it does not panic, but every setback tried (0.3mm through
// 1.5mm) hit "the offset changes the section's topology; a trimmed-offset
// kernel is not available" on at least one letter, so those shapes go
// straight from extrude to chamfer with no fillet.
func extrudeLetterLoops(ctx context.Context, loops [][]point, extent decad.Extent, chord units.Value) (*decad.Mesh, error) {
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
	if len(loops) > 1 {
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

// heroRailModels builds the rail above the wordmark from the same composite
// path sweepShot proves: an arc, a flat run, and a second arc that leaves the
// first arc's plane — what a single extrude cannot draw. Composite-Sweep
// tessellation is still staged (docs/sweep-design.md Table D), so the picture
// is drawn from the equivalent Revolve/Extrude/Revolve spans after the real
// Sweep has passed its own topology, measurement and contact audits, exactly
// as sweepShot does.
func heroRailModels(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	// x and z translate sweepShot's own path/axis coordinates by a fixed
	// shift alone: translation changes no tangent direction, so every join
	// Sweep's P5 gate already accepted for these numbers stays accepted here.
	x := func(v float64) float64 { return v + railShiftX }
	z := func(v float64) float64 { return v + railShiftZ }

	w := sketch.NewWorld()

	firstPlane, err := w.CreateOffsetPlane(w.XY(), z(0))
	if err != nil {
		return nil, err
	}
	s, profile, err := sketchLoops(ctx, w, firstPlane, rectangle(x(-46), -railProfileHalf, x(-34), railProfileHalf))
	if err != nil {
		return nil, err
	}
	path, err := decad.NewPath(
		r3.NewVec(x(-40), 0, z(0)),
		decad.ArcThrough{
			Through: r3.NewVec(x(-32), 0, z(16)),
			End:     r3.NewVec(x(-20), 0, z(20)),
		},
		decad.LineTo{End: r3.NewVec(x(20), 0, z(20))},
		decad.ArcThrough{
			Through: r3.NewVec(x(36), 8, z(20)),
			End:     r3.NewVec(x(40), 20, z(20)),
		},
	)
	if err != nil {
		return nil, fmt.Errorf("record the rail's sweep path: %w", err)
	}
	if _, err := decad.New().Sweep(ctx, s, profile, path); err != nil {
		return nil, fmt.Errorf("sweep the rail's spatial path: %w", err)
	}

	first, err := decad.New().Revolve(s, profile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.SketchLine{Start: decad.Point2{U: x(-20), V: -1}, End: decad.Point2{U: x(-20), V: 1}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the rail's first span: %w", err)
	}

	middleFrame, err := r3.NewFrame(
		r3.NewVec(x(-20), 0, z(-20)),
		r3.NewVec(0, 0, -1),
		r3.NewVec(0, 1, 0),
	)
	if err != nil {
		return nil, err
	}
	middlePlane, err := w.CreatePlaneFromFrame(middleFrame)
	if err != nil {
		return nil, err
	}
	middleSketch, middleProfile, err := sketchLoops(ctx, w, middlePlane, rectangle(-46, -railProfileHalf, -34, railProfileHalf))
	if err != nil {
		return nil, err
	}
	middle, err := decad.New().Extrude(middleSketch, middleProfile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the rail's straight span: %w", err)
	}

	lastFrame, err := r3.NewFrame(
		r3.NewVec(x(20), 0, z(-20)),
		r3.NewVec(0, 0, -1),
		r3.NewVec(0, 1, 0),
	)
	if err != nil {
		return nil, err
	}
	lastPlane, err := w.CreatePlaneFromFrame(lastFrame)
	if err != nil {
		return nil, err
	}
	lastSketch, lastProfile, err := sketchLoops(ctx, w, lastPlane, rectangle(-46, -railProfileHalf, -34, railProfileHalf))
	if err != nil {
		return nil, err
	}
	last, err := decad.New().Revolve(lastSketch, lastProfile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.SketchLine{Start: decad.Point2{U: -39, V: 20}, End: decad.Point2{U: -41, V: 20}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the rail's second span: %w", err)
	}

	models := make([]solidlens.Model, 0, 3)
	for _, body := range []*decad.Body{first, middle, last} {
		spanModels, modelErr := oneModel(ctx, body, railColor, chord)
		if modelErr != nil {
			return nil, modelErr
		}
		models = append(models, spanModels...)
	}
	return models, nil
}
