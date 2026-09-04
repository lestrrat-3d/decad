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

// Every feature thumbnail is rendered at this size and chorded at this
// tolerance. The parts are all drawn to fit one shared camera, so a reader
// comparing two rows of the README table is comparing the geometry and not the
// framing.
const (
	featureWidth          = 640
	featureHeight         = 480
	featureChordTolerance = 0.2
)

// featureRenders is one image per row of the README's feature table, in the
// order the table lists them.
func featureRenders() []imageRender {
	shots := []struct {
		name  string
		build func(context.Context) ([]solidlens.Model, error)
	}{
		{"extrude", extrudeShot},
		{"revolve", revolveShot},
		{"loft", loftShot},
		{"fillet", filletShot},
		{"chamfer", chamferShot},
		{"cap-chamfer", capChamferShot},
		{"shell", shellShot},
		{"boolean", booleanShot},
		{"freeform", freeformShot},
		{"verify", verifyShot},
	}
	renders := make([]imageRender, len(shots))
	for i, shot := range shots {
		renders[i] = imageRender{
			rel:      "docs/images/features/" + shot.name + ".png",
			settings: solidlens.Settings{Width: featureWidth, Height: featureHeight},
			scene: func(ctx context.Context) (solidlens.Scene, error) {
				models, err := shot.build(ctx)
				if err != nil {
					return solidlens.Scene{}, err
				}
				return featureScene(models...), nil
			},
		}
	}
	return renders
}

// extrudeShot sweeps one L-shaped section straight up into an angle bracket.
func extrudeShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	bracket, err := prism(ctx, doc, w, w.XY(), 44, []point{
		{-40, -28}, {40, -28}, {40, -8}, {-20, -8}, {-20, 28}, {-40, 28},
	})
	if err != nil {
		return nil, fmt.Errorf("extrude the bracket: %w", err)
	}
	return oneModel(ctx, bracket, cyan)
}

// revolveShot spins a circle offset from the axis into a torus — the curved
// generator, where a revolve says something a straight sweep cannot.
func revolveShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	// The XZ plane puts the sketch's v axis on world Z, so the ring lies flat.
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		return nil, err
	}
	center := s.CreatePoint(38, 0)
	s.Fix(center)
	s.CreateCircle(center, 14)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	// Revolve has no context-aware variant; Solve above is the cancellable phase.
	ring, err := decad.New().Revolve(s, profile, axis, decad.FullRevolution{}) //nolint:contextcheck
	if err != nil {
		return nil, fmt.Errorf("revolve the ring: %w", err)
	}
	return oneModel(ctx, ring, blue)
}

// loftShot rules a wall between two rectangles on different planes, the top one
// smaller and offset — a transition duct rather than a plain pyramid.
func loftShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	bottom, bottomProfile, err := sketchLoops(ctx, w, w.XY(), rectangle(-42, -30, 42, 30))
	if err != nil {
		return nil, err
	}
	topPlane, err := w.CreateOffsetPlane(w.XY(), 46)
	if err != nil {
		return nil, err
	}
	top, topProfile, err := sketchLoops(ctx, w, topPlane, rectangle(-4, -14, 32, 14))
	if err != nil {
		return nil, err
	}
	duct, err := decad.New().LoftContext(ctx, bottom, bottomProfile, top, topProfile)
	if err != nil {
		return nil, fmt.Errorf("loft the duct: %w", err)
	}
	return oneModel(ctx, duct, violet)
}

// filletShot rounds the four lateral edges of a plate into tangent cylinders.
func filletShot(ctx context.Context) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	rounded, err := plate.FilletContext(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
	if err != nil {
		return nil, fmt.Errorf("fillet the plate: %w", err)
	}
	return oneModel(ctx, rounded, coral)
}

// chamferShot bevels the same four lateral edges the fillet rounds, so the two
// thumbnails differ only in what the modify op put there.
func chamferShot(ctx context.Context) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	bevelled, err := plate.ChamferContext(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
	if err != nil {
		return nil, fmt.Errorf("chamfer the plate: %w", err)
	}
	return oneModel(ctx, bevelled, gold)
}

// capChamferShot bevels a complete cap loop — the lead-in a bore or a keycap
// carries — which the evaluator builds along its own path, not the lateral one.
func capChamferShot(ctx context.Context) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	bevelled, err := plate.ChamferContext(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(plate))), units.Millimeters(10))
	if err != nil {
		return nil, fmt.Errorf("chamfer the cap loop: %w", err)
	}
	return oneModel(ctx, bevelled, cyan)
}

// shellShot removes one cap and offsets the section inward, leaving an open
// tray whose wall thickness is the section's own exact offset.
func shellShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	block, err := prism(ctx, doc, w, w.XY(), 34, rectangle(-46, -32, 46, 32))
	if err != nil {
		return nil, err
	}
	tray, err := block.ShellContext(ctx, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(7))
	if err != nil {
		return nil, fmt.Errorf("shell the block: %w", err)
	}
	return oneModel(ctx, tray, blue)
}

// booleanShot drills a flange plate: one central bore and two bolt holes,
// one Cut per hole.
func booleanShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	plate, err := prism(ctx, doc, w, w.XY(), 16, rectangle(-48, -34, 48, 34))
	if err != nil {
		return nil, err
	}
	for _, hole := range []struct {
		at     point
		radius float64
	}{{point{0, 0}, 18}, {point{-36, 0}, 7}, {point{36, 0}, 7}} {
		// The drill runs clear past both plate faces: a tool cap resting ON a
		// face is a face-on-face contact the boolean refuses.
		tool, err := boreTool(ctx, doc, w, hole.at, hole.radius)
		if err != nil {
			return nil, err
		}
		plate, err = decad.CutContext(ctx, plate, tool)
		if err != nil {
			return nil, fmt.Errorf("drill the plate: %w", err)
		}
	}
	return oneModel(ctx, plate, violet)
}

// freeformShot extrudes a section whose curved wall is a fit spline, closed by
// a straight chord: a blade the evaluator measures exactly rather than
// approximating.
func freeformShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	fit := make([]*sketch.Point, 0, 5)
	for _, p := range []point{{-46, 14}, {-26, -6}, {0, -14}, {26, -6}, {46, 14}} {
		fit = append(fit, s.CreatePoint(p.x, p.y))
	}
	s.Fix(fit[0])
	if _, err := s.CreateFitSpline(fit...); err != nil {
		return nil, err
	}
	s.CreateLine(fit[len(fit)-1], fit[0])
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	blade, err := decad.New().Extrude(s, profile, //nolint:contextcheck
		decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("extrude the blade: %w", err)
	}
	return oneModel(ctx, blade, coral)
}

// verifyShot poses the question verification answers: a pin standing in a bore
// it must not touch, with the clearance ring visible all the way round.
func verifyShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	plate, err := prism(ctx, doc, w, w.XY(), 16, rectangle(-44, -38, 44, 38))
	if err != nil {
		return nil, err
	}
	bore, err := boreTool(ctx, doc, w, point{0, 0}, 22)
	if err != nil {
		return nil, err
	}
	housing, err := decad.CutContext(ctx, plate, bore)
	if err != nil {
		return nil, fmt.Errorf("bore the housing: %w", err)
	}

	pinSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	center := pinSketch.CreatePoint(0, 0)
	pinSketch.Fix(center)
	pinSketch.CreateCircle(center, 15)
	if _, err := pinSketch.Solve(ctx); err != nil {
		return nil, err
	}
	pinProfile, err := validProfile(pinSketch)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	pin, err := doc.Extrude(pinSketch, pinProfile, //nolint:contextcheck
		decad.Distance{D: units.Millimeters(46), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("extrude the pin: %w", err)
	}

	housingModel, err := oneModel(ctx, housing, gold)
	if err != nil {
		return nil, err
	}
	pinModel, err := oneModel(ctx, pin, cyan)
	if err != nil {
		return nil, err
	}
	return append(housingModel, pinModel...), nil
}

// lateralEdgePlate is the receiver the three modify shots share: one plate with
// four lateral edges and two cap loops, so each op is seen on the same part.
func lateralEdgePlate(ctx context.Context) (*decad.Body, error) {
	w := sketch.NewWorld()
	return prism(ctx, decad.New(), w, w.XY(), 26, rectangle(-50, -34, 50, 34))
}

// boreTool extrudes a circular drill through the sketch plane in both
// directions, long enough to clear any plate in this gallery.
func boreTool(ctx context.Context, doc *decad.Document, w *sketch.World, center point, radius float64) (*decad.Body, error) {
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	origin := s.CreatePoint(center.x, center.y)
	s.Fix(origin)
	s.CreateCircle(origin, radius)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	return doc.Extrude(s, profile, decad.Symmetric{D: units.Millimeters(30)}) //nolint:contextcheck
}

// prism sweeps the given plane-local loops straight along the plane normal.
func prism(ctx context.Context, doc *decad.Document, w *sketch.World, plane *sketch.Plane, height float64, loops ...[]point) (*decad.Body, error) {
	s, profile, err := sketchLoops(ctx, w, plane, loops...)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; the sketch solve is the cancellable phase.
	return doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(height), Dir: decad.Along}) //nolint:contextcheck
}

func sketchLoops(ctx context.Context, w *sketch.World, plane *sketch.Plane, loops ...[]point) (*sketch.Sketch, *sketch.Profile, error) {
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, nil, err
	}
	profile, err := polylineProfile(ctx, s, loops)
	if err != nil {
		return nil, nil, err
	}
	return s, profile, nil
}

// validProfile picks the one region a single-loop sketch bounds.
func validProfile(s *sketch.Sketch) (*sketch.Profile, error) {
	for _, candidate := range s.Profiles() {
		if candidate.Valid {
			return candidate, nil
		}
	}
	return nil, fmt.Errorf("sketch bounds no valid profile")
}

// oneModel tessellates a body into the single matte model one shot renders.
func oneModel(ctx context.Context, body *decad.Body, color solidlens.Color) ([]solidlens.Model, error) {
	mesh, err := body.TessellateContext(ctx, units.Millimeters(featureChordTolerance))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	return []solidlens.Model{{Mesh: mesh, Material: solidlens.Matte(color)}}, nil
}
