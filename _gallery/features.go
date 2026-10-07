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
// tolerance unless -chord or -scale override it. The parts are all drawn to
// fit one shared camera, so a reader comparing two rows of the README table
// is comparing the geometry and not the framing.
const (
	featureWidth          = 1280
	featureHeight         = 960
	featureChordTolerance = 0.02
)

// featureRenders is one image per README feature entry.
func featureRenders() []imageRender {
	shots := []struct {
		name  string
		build func(context.Context, units.Value) ([]solidlens.Model, error)
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
			rel:      "features/" + shot.name + ".png",
			settings: solidlens.Settings{Width: featureWidth, Height: featureHeight},
			chord:    units.Millimeters(featureChordTolerance),
			scene: func(ctx context.Context, chord units.Value) (solidlens.Scene, error) {
				models, err := shot.build(ctx, chord)
				if err != nil {
					return solidlens.Scene{}, err
				}
				return featureScene(models...), nil
			},
		}
	}
	return renders
}

// sweepShot renders one mitred Sweep along a path that turns in two planes.
func sweepShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	body, err := mitredDuct(ctx, 120)
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, body, cyan, chord)
}

// mitredDuct sweeps a square along up to 120mm of a three-span spatial path.
// Every partial path is one real Sweep body with the same recorded section.
func mitredDuct(ctx context.Context, length float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, profile, err := sketchLoops(ctx, w, w.XY(), rectangle(-34, -24, -26, -16))
	if err != nil {
		return nil, err
	}
	segments := []decad.PathSegment{
		decad.LineTo{End: r3.NewVec(-30, -20, min(length, 30))},
	}
	if length > 30 {
		segments = append(segments, decad.LineTo{End: r3.NewVec(-30+min(length-30, 50), -20, 30)})
	}
	if length > 80 {
		segments = append(segments, decad.LineTo{End: r3.NewVec(20, -20+min(length-80, 40), 30)})
	}
	path, err := decad.NewPath(r3.NewVec(-30, -20, 0), segments...)
	if err != nil {
		return nil, fmt.Errorf("record duct path: %w", err)
	}
	body, err := decad.New().Sweep(ctx, s, profile, path, decad.WithMitredJoins())
	if err != nil {
		return nil, fmt.Errorf("sweep duct: %w", err)
	}
	return body, nil
}

// ductSpans supplies the landing clip's curved shelf part. The composite
// arc Sweep's tessellation is staged, so that clip draws equivalent spans.
func ductSpans(ctx context.Context) ([]*decad.Body, error) {
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

	return []*decad.Body{first, middle, last}, nil
}

// extrudeShot sweeps one L-shaped section straight up into an angle bracket.
func extrudeShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	bracket, err := prism(ctx, doc, w, w.XY(), 44, []point{
		{-40, -28}, {40, -28}, {40, -8}, {-20, -8}, {-20, 28}, {-40, 28},
	})
	if err != nil {
		return nil, fmt.Errorf("extrude the bracket: %w", err)
	}
	return oneModel(ctx, bracket, cyan, chord)
}

// revolveShot spins a circle offset from the axis into a torus — the curved
// generator, where a revolve says something a straight sweep cannot.
func revolveShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	ring, err := ringBody(ctx)
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, ring, blue, chord)
}

// ringBody revolves a circle of radius 14mm, 38mm off the axis, into a flat
// torus.
func ringBody(ctx context.Context) (*decad.Body, error) {
	return ringBodyAtAngle(ctx, 360)
}

func ringBodyAtAngle(ctx context.Context, angle float64) (*decad.Body, error) {
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
	var extent decad.AngularExtent = decad.AngleExtent{A: units.Degrees(angle), Dir: decad.Along}
	if angle == 360 {
		extent = decad.FullRevolution{}
	}
	ring, err := decad.New().Revolve(s, profile, axis, extent) //nolint:contextcheck
	if err != nil {
		return nil, fmt.Errorf("revolve the ring: %w", err)
	}
	return ring, nil
}

// loftShot rules a wall between two rectangles on different planes, the top one
// smaller and offset — a transition duct rather than a plain pyramid.
func loftShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	duct, err := loftDuct(ctx)
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, duct, violet, chord)
}

// loftDuct lofts an 84×60mm rectangle into a 36×28mm one, offset along X,
// 46mm above it.
func loftDuct(ctx context.Context) (*decad.Body, error) {
	return loftDuctAtHeight(ctx, 46)
}

// loftDuctAtHeight caps the final loft at height. Its top rectangle follows
// the corresponding section of the final ruled walls.
func loftDuctAtHeight(ctx context.Context, height float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	bottom, bottomProfile, err := sketchLoops(ctx, w, w.XY(), rectangle(-42, -30, 42, 30))
	if err != nil {
		return nil, err
	}
	topPlane, err := w.CreateOffsetPlane(w.XY(), height)
	if err != nil {
		return nil, err
	}
	t := height / 46
	topSection := rectangle(-42+38*t, -30+16*t, 42-10*t, 30-16*t)
	top, topProfile, err := sketchLoops(ctx, w, topPlane, topSection)
	if err != nil {
		return nil, err
	}
	duct, err := decad.New().Loft(ctx, bottom, bottomProfile, top, topProfile)
	if err != nil {
		return nil, fmt.Errorf("loft the duct: %w", err)
	}
	return duct, nil
}

// filletShot rounds the four lateral edges of a plate into tangent cylinders.
func filletShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	rounded, err := plate.Fillet(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
	if err != nil {
		return nil, fmt.Errorf("fillet the plate: %w", err)
	}
	return oneModel(ctx, rounded, coral, chord)
}

// chamferShot bevels the same four lateral edges the fillet rounds, so the two
// thumbnails differ only in what the modify op put there.
func chamferShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	bevelled, err := plate.Chamfer(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(16))
	if err != nil {
		return nil, fmt.Errorf("chamfer the plate: %w", err)
	}
	return oneModel(ctx, bevelled, gold, chord)
}

// capChamferShot bevels a complete cap loop — the lead-in a bore or a keycap
// carries — which the evaluator builds along its own path, not the lateral one.
func capChamferShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	plate, err := lateralEdgePlate(ctx)
	if err != nil {
		return nil, err
	}
	bevelled, err := plate.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(plate))), units.Millimeters(10))
	if err != nil {
		return nil, fmt.Errorf("chamfer the cap loop: %w", err)
	}
	return oneModel(ctx, bevelled, cyan, chord)
}

// shellShot removes one cap and offsets the section inward, leaving an open
// tray whose wall thickness is the section's own exact offset.
func shellShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	tray, err := trayBody(ctx)
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, tray, blue, chord)
}

// trayBody shells a 92×64×34mm block through its top face, leaving a 7mm
// wall.
func trayBody(ctx context.Context) (*decad.Body, error) {
	return trayBodyAtThickness(ctx, 7)
}

func trayBodyAtThickness(ctx context.Context, thickness float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	block, err := prism(ctx, doc, w, w.XY(), 34, rectangle(-46, -32, 46, 32))
	if err != nil {
		return nil, err
	}
	tray, err := block.Shell(ctx, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(thickness))
	if err != nil {
		return nil, fmt.Errorf("shell the block: %w", err)
	}
	return tray, nil
}

// booleanShot drills the flange plate: one central bore and two bolt holes,
// one Cut per hole (flangeBody).
func booleanShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	plate, err := flangeBody(ctx, throughFlange())
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, plate, violet, chord)
}

// freeformShot extrudes a section bounded by two fit splines: a blade the
// evaluator measures exactly rather than approximating.
func freeformShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	blade, err := bladeBody(ctx)
	if err != nil {
		return nil, err
	}
	return oneModel(ctx, blade, coral, chord)
}

// bladeBody extrudes a 30mm section bounded by two fit splines.
func bladeBody(ctx context.Context) (*decad.Body, error) {
	return bladeBodyAtShape(ctx, 30, 1)
}

// bladeBodyAtShape scales the section's Y coordinates before extruding it.
func bladeBodyAtShape(ctx context.Context, height, profileScale float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	start := s.CreatePoint(-48, 0)
	end := s.CreatePoint(48, 0)
	s.Fix(start)
	upper := []*sketch.Point{start, s.CreatePoint(-10, 28*profileScale), end}
	if _, err := s.CreateFitSpline(upper...); err != nil {
		return nil, err
	}
	lower := []*sketch.Point{end, s.CreatePoint(12, -22*profileScale), start}
	if _, err := s.CreateFitSpline(lower...); err != nil {
		return nil, err
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profile, err := validProfile(s)
	if err != nil {
		return nil, err
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	blade, err := decad.New().Extrude(s, profile, //nolint:contextcheck
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("extrude the blade: %w", err)
	}
	return blade, nil
}

// surfaceShot revolves a half-disc standing on the axis through part of a turn
// and keeps only the swept wall, so the result is a dish: one sheet face, open
// along both cap-plane rims. The opening faces the gallery camera, which is
// what makes the shot worth taking — the far half of the dish is seen from its
// inner side, and the two materials say which side of the sheet a reader is
// looking at.
func surfaceShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
	dish, err := dishBody(ctx)
	if err != nil {
		return nil, err
	}
	return twoSidedModel(ctx, dish, violet, gold, chord)
}

// dishBody revolves a half-disc of radius 42mm through 150° about a vertical
// axis 26mm behind the origin and keeps only the swept wall. It refuses a
// result that is not an open sheet (requireSheet).
func dishBody(ctx context.Context) (*decad.Body, error) {
	return dishBodyAtAngle(ctx, 150)
}

func dishBodyAtAngle(ctx context.Context, angle float64) (*decad.Body, error) {
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
		decad.AngleExtent{A: units.Degrees(angle), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		return nil, fmt.Errorf("revolve the dish: %w", err)
	}
	if err := requireSheet(dish); err != nil {
		return nil, err
	}
	return dish, nil
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
func verifyShot(ctx context.Context, chord units.Value) ([]solidlens.Model, error) {
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

	pin, err := verifyPin(ctx, doc, w)
	if err != nil {
		return nil, err
	}

	housingModel, err := oneModel(ctx, housing, gold, chord)
	if err != nil {
		return nil, err
	}
	pinModel, err := oneModel(ctx, pin, cyan, chord)
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

// The verify pin: a cylinder standing on XY at the origin.
const (
	pinRadius = 15.0
	pinLength = 46.0
)

// verifyPin extrudes the pin from XY along +Z.
func verifyPin(ctx context.Context, doc *decad.Document, w *sketch.World) (*decad.Body, error) {
	pin, err := cylinder(ctx, doc, w, w.XY(), point{}, pinRadius, decad.Distance{
		D:   units.Millimeters(pinLength),
		Dir: decad.Along,
	})
	if err != nil {
		return nil, fmt.Errorf("extrude the pin: %w", err)
	}
	return pin, nil
}

// boreTool extrudes a circular drill through the sketch plane in both
// directions, long enough to clear any plate in this gallery.
func boreTool(ctx context.Context, doc *decad.Document, w *sketch.World, center point, radius float64) (*decad.Body, error) {
	return cylinder(ctx, doc, w, w.XY(), center, radius, decad.Symmetric{D: units.Millimeters(30)})
}

// cylinder extrudes the circle of radius about plane-local center on plane
// by extent.
func cylinder(
	ctx context.Context, doc *decad.Document, w *sketch.World, plane *sketch.Plane,
	center point, radius float64, extent decad.Extent,
) (*decad.Body, error) {
	s, err := w.CreateSketch(plane)
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
	return doc.Extrude(s, profile, extent) //nolint:contextcheck
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
// These meshes are drawn, never proven: VerifyNone skips the facet-contact
// audit's own work ceiling, which a fine chord tolerance can otherwise hit.
func oneModel(ctx context.Context, body *decad.Body, color solidlens.Color, chord units.Value) ([]solidlens.Model, error) {
	mesh, err := body.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	return []solidlens.Model{{Mesh: mesh, Material: solidlens.Matte(color)}}, nil
}

// twoSidedModel tessellates a sheet and shades its two sides apart: front is
// the positive side every surface-result wall inherits from the solid's
// outward normal, back is the side a reader sees through the sheet's opening.
func twoSidedModel(
	ctx context.Context, body *decad.Body, front, back solidlens.Color, chord units.Value,
) ([]solidlens.Model, error) {
	mesh, err := body.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
	if err != nil {
		return nil, fmt.Errorf("tessellate: %w", err)
	}
	inner := solidlens.Matte(back)
	return []solidlens.Model{{Mesh: mesh, Material: solidlens.Matte(front), BackMaterial: &inner}}, nil
}
