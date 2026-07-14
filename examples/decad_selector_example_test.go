package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Selectors name topology by intent — geometric predicates and provenance —
// never by pointer or index, so a query resolves identically on a recipe
// replay. Here one selects the drilled plate's hole wall by surface kind plus
// the feature role that created it, and another names a revolve axis as "the
// one bottom edge of the gusset parallel to x".
func Example_decad_selectors() {
	w := sketch.NewWorld()

	// A 100 x 60 plate with a Ø20 hole, extruded 8 mm.
	ps, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := ps.CreateRectangle(0, 0, 100, 60)
	ps.Fix(rect.A)
	ps.CreateCircle(ps.CreatePoint(70, 30), 10)
	if _, err := ps.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var prof *sketch.Profile
	for _, p := range ps.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}

	doc := decad.New()
	plate, err := doc.Extrude(ps, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The hole wall is the face that is cylindrical AND was created by the
	// hole loop's side role — provenance is structural, so the same query
	// selects the same face under every evaluation.
	holeRole := decad.FeatureRef{Step: plate.Origin().Step, Role: "side(1,0)"}
	walls, err := decad.Faces(decad.Cylindrical(), decad.FaceCreatedBy(holeRole)).Exactly(1).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select the hole wall: %s\n", err)
		return
	}
	cyl := walls[0].Surface().(decad.Cylinder)
	fmt.Printf("plate faces: %d\n", len(plate.Faces()))
	fmt.Printf("hole wall radius: %s\n", cyl.Radius)

	// A triangular gusset: its bottom cap holds exactly one edge parallel to
	// x — the world x axis — which an EdgeAxis names as a revolve axis.
	gs, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := gs.CreatePoint(0, 0)
	gs.Fix(a)
	b := gs.CreatePoint(100, 0)
	c := gs.CreatePoint(0, 60)
	gs.CreateLine(a, b)
	gs.CreateLine(b, c)
	gs.CreateLine(c, a)
	if _, err := gs.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	gusset, err := doc.Extrude(gs, gs.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// A 10 x 10 ring section 5 mm off the axis, revolved a full turn about
	// the selected edge: Pappus gives 2π · 10 · 100 = 2000π mm³.
	rs, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	ring := rs.CreateRectangle(0, 5, 10, 15)
	rs.Fix(ring.A)
	if _, err := rs.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	axis := decad.EdgeAxis{
		Body: gusset,
		Edge: decad.Edges(
			decad.CreatedBy(decad.FeatureRef{Step: gusset.Origin().Step, Role: "capStart"}),
			decad.ParallelTo(r3.NewVec(1, 0, 0)),
		).Exactly(1),
	}
	body, err := doc.Revolve(rs, rs.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}
	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	fmt.Printf("revolved volume: %.2f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("recipe steps: %d\n", len(doc.Recipe().Steps))
	// Output:
	// plate faces: 7
	// hole wall radius: 10 mm
	// revolved volume: 6283.19 mm^3 (Exact)
	// recipe steps: 3
}
