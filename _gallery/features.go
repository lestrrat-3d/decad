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

// featureRenders is one image per README feature entry.
func featureRenders() []imageRender {
	shots := []struct {
		name  string
		build func(context.Context) ([]solidlens.Model, error)
	}{
		{"extrude", extrudeShot},
		{"revolve", revolveShot},
		{"sweep", sweepShot},
		{"loft", loftShot},
		{"fillet", filletShot},
		{"chamfer", chamferShot},
		{"cap-chamfer", capChamferShot},
		{"shell", shellShot},
		{"boolean", booleanShot},
		{"freeform", freeformShot},
		{"surface", surfaceShot},
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

// sweepShot carries a square section through two orthogonal bend planes. The
// current Sweep payload deliberately stages tessellation, so the scene renders
// the same two Revolve spans and intervening Extrude after the real composite
// Sweep has passed its topology, measurement, and contact audits.
func sweepShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	s, profile, err := sketchLoops(ctx, w, w.XY(), rectangle(-46, -6, -34, 6))
	if err != nil {
		return nil, err
	}
	path, err := decad.NewPath(
		r3.NewVec(-40, 0, 0),
		decad.ArcThrough{
			Through: r3.NewVec(-32, 0, 16),
			End:     r3.NewVec(-20, 0, 20),
		},
		decad.LineTo{End: r3.NewVec(20, 0, 20)},
		decad.ArcThrough{
			Through: r3.NewVec(36, 8, 20),
			End:     r3.NewVec(40, 20, 20),
		},
	)
	if err != nil {
		return nil, fmt.Errorf("record the sweep path: %w", err)
	}
	if _, err := decad.New().Sweep(ctx, s, profile, path); err != nil {
		return nil, fmt.Errorf("sweep the spatial path: %w", err)
	}

	first, err := decad.New().Revolve(s, profile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.SketchLine{Start: decad.Point2{U: -20, V: -1}, End: decad.Point2{U: -20, V: 1}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the first render span: %w", err)
	}

	middleFrame, err := r3.NewFrame(
		r3.NewVec(-20, 0, -20),
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
	middleSketch, middleProfile, err := sketchLoops(ctx, w, middlePlane, rectangle(-46, -6, -34, 6))
	if err != nil {
		return nil, err
	}
	middle, err := decad.New().Extrude(middleSketch, middleProfile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the straight render span: %w", err)
	}

	lastFrame, err := r3.NewFrame(
		r3.NewVec(20, 0, -20),
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
	lastSketch, lastProfile, err := sketchLoops(ctx, w, lastPlane, rectangle(-46, -6, -34, 6))
	if err != nil {
		return nil, err
	}
	last, err := decad.New().Revolve(lastSketch, lastProfile, //nolint:contextcheck // Sweep and sketch solve above are cancellable.
		decad.SketchLine{Start: decad.Point2{U: -39, V: 20}, End: decad.Point2{U: -41, V: 20}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("build the second render span: %w", err)
	}

	models := make([]solidlens.Model, 0, 3)
	for _, body := range []*decad.Body{first, middle, last} {
		spanModels, modelErr := oneModel(ctx, body, cyan)
		if modelErr != nil {
			return nil, modelErr
		}
		models = append(models, spanModels...)
	}
	return models, nil
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
	duct, err := decad.New().Loft(ctx, bottom, bottomProfile, top, topProfile)
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
	rounded, err := plate.Fillet(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
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
	bevelled, err := plate.Chamfer(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
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
	bevelled, err := plate.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(plate))), units.Millimeters(10))
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
	tray, err := block.Shell(ctx, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(7))
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
		plate, err = decad.Cut(ctx, plate, tool)
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

// surfaceShot revolves a half-disc standing on the axis through part of a turn
// and keeps only the swept wall, so the result is a dish: one sheet face, open
// along both cap-plane rims. The opening faces the gallery camera, which is
// what makes the shot worth taking — the far half of the dish is seen from its
// inner side, and the two materials say which side of the sheet a reader is
// looking at.
func surfaceShot(ctx context.Context) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	// The XZ plane puts the sketch's v axis on world Z, so the dish stands
	// upright and its axis is vertical. Offsetting that plane carries the axis
	// with it, which slides the swept half of the dish back over the camera's
	// target and frames it like every other shot.
	plane, err := w.CreateOffsetPlane(w.XZ(), 26)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	const radius, rise = 42.0, 20.0
	center := s.CreatePoint(0, rise)
	s.Fix(center)
	bottom := s.CreatePoint(0, rise-radius)
	top := s.CreatePoint(0, rise+radius)
	// The diameter lies on the axis and sweeps nothing, so the arc is the
	// whole of the wall and the sheet carries one face.
	s.CreateLine(top, bottom)
	s.CreateArc(center, bottom, top)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	// Revolve has no context-aware variant; Solve above is the cancellable phase.
	dish, err := decad.New().Revolve(s, profile, axis, //nolint:contextcheck
		decad.AngleExtent{A: units.Degrees(150), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		return nil, fmt.Errorf("revolve the dish: %w", err)
	}
	if err := requireSheet(dish); err != nil {
		return nil, err
	}
	return twoSidedModel(ctx, dish, violet, gold)
}

// requireSheet refuses a body that is not an open sheet, so the thumbnail
// cannot quietly become a closed solid whose inner side no reader ever sees.
func requireSheet(body *decad.Body) error {
	if body.Kind() != decad.BodySheet {
		return fmt.Errorf("body is %v, want a sheet", body.Kind())
	}
	free, err := decad.Edges(decad.Free()).SelectEdges(body)
	if err != nil {
		return fmt.Errorf("select the free edges: %w", err)
	}
	if len(free) == 0 {
		return fmt.Errorf("sheet has no free edge, so nothing opens onto its inner side")
	}
	return nil
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
	housing, err := decad.Cut(ctx, plate, bore)
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
	mesh, err := body.Tessellate(ctx, units.Millimeters(featureChordTolerance))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	return []solidlens.Model{{Mesh: mesh, Material: solidlens.Matte(color)}}, nil
}

// twoSidedModel tessellates a sheet and shades its two sides apart: front is
// the positive side every surface-result wall inherits from the solid's
// outward normal, back is the side a reader sees through the sheet's opening.
func twoSidedModel(ctx context.Context, body *decad.Body, front, back solidlens.Color) ([]solidlens.Model, error) {
	mesh, err := body.TessellateContext(ctx, units.Millimeters(featureChordTolerance))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	inner := solidlens.Matte(back)
	return []solidlens.Model{{Mesh: mesh, Material: solidlens.Matte(front), BackMaterial: &inner}}, nil
}
