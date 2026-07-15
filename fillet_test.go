package decad_test

import (
	"encoding/json"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// filletBoxHeight is the sweep height of the plate every fillet test rounds.
const filletBoxHeight = 20.0

// filletBox extrudes the 100×60 plate by filletBoxHeight — a straight prism
// with four convex lateral edges, one per corner of its rectangular section.
func filletBox(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(filletBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// verticalEdges selects a prism's lateral edges: straight and parallel to the
// sweep, which for an XY-plane extrude is the z axis.
func verticalEdges() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)))
}

func TestFilletBoxAllConvexEdges(t *testing.T) {
	const r = 10.0
	h := filletBoxHeight
	doc, box := filletBox(t)

	// The selector resolves to exactly the four lateral edges, not the cap
	// rims — so filleting them fillets exactly the four corners.
	edges, err := verticalEdges().SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, edges, 4, `a box has four lateral edges`)

	body, err := box.Fillet(verticalEdges(), units.Millimeters(r))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume: the square-corner prism minus a (1 − π/4)r² wedge per rounded
	// corner, hand-derived and exact.
	wantVol := (100.0*60.0 - (4-math.Pi)*r*r) * h
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12), `a fillet introduces no bound`)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// Area: the rounded-rectangle perimeter (straight runs plus one full
	// circle of quarter arcs) swept over h, plus two caps.
	perimeter := 2*(100-2*r) + 2*(60-2*r) + 2*math.Pi*r
	capArea := 100.0*60.0 - (4-math.Pi)*r*r
	wantArea := perimeter*h + 2*capArea
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, gotArea, 1e-9)

	// Topology: four straight walls + four blend cylinders + two caps.
	require.Len(t, body.Faces(), 10)
	cylinders := 0
	for _, f := range body.Faces() {
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		cylinders++
		require.True(t, cyl.Radius.Equal(units.Millimeters(r), 1e-9), `the blend wall is a cylinder of the fillet radius`)
	}
	require.Equal(t, 4, cylinders, `one blend cylinder per rounded corner`)

	// Each blend cylinder carries a second fillet(i,j) role naming the same
	// (loop, segment) its side(i,j) role does.
	blendRoles := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); !ok {
			continue
		}
		hasFillet, hasSide := false, false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "fillet(") {
				hasFillet = true
			}
			if strings.HasPrefix(o.Role, "side(") {
				hasSide = true
			}
		}
		require.True(t, hasSide, `a blend wall keeps its side role`)
		require.True(t, hasFillet, `a blend wall carries the fillet role`)
		blendRoles++
	}
	require.Equal(t, 4, blendRoles)

	// A convex fillet adds a convex cylinder — not a concave feature — so the
	// minimum-radius survey rightly does not report it (Table D, D3).
	rep, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.Nil(t, rep.Bodies[0].MinRadius, `a convex fillet is not a concave feature`)
	require.True(t, rep.Trustworthy(), `a filleted prism is Sound on the same terms an extrude is`)
}

func TestFilletRecipeAndRetire(t *testing.T) {
	const r = 8.0
	doc, box := filletBox(t)
	body, err := box.Fillet(verticalEdges(), units.Millimeters(r))
	require.NoError(t, err)

	// The receiver is retired; the document holds the filleted body.
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	_, err = box.Fillet(verticalEdges(), units.Millimeters(r))
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	// The step records the op, the receiver as its input, the unresolved
	// selector, the radius value, and no options — and it round-trips.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	fillet := recipe.Steps[1]
	require.Equal(t, decad.OpFillet, fillet.Op)
	require.Equal(t, []decad.StepRef{0}, fillet.Inputs)
	require.Len(t, fillet.Selectors, 1)
	require.Len(t, fillet.Values, 1)
	require.True(t, fillet.Values[0].Equal(units.Millimeters(r), 1e-12))
	require.Nil(t, fillet.Opts, `a fillet Step takes no options this increment`)

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded fillet recipe round-trips`)
}

func TestFilletSelectorIsRecordedUnresolved(t *testing.T) {
	// The step stores a clone of the query, not the caller's, and never the
	// edges it resolved to (core §9 / §11).
	doc, box := filletBox(t)
	q := verticalEdges()
	_, err := box.Fillet(q, units.Millimeters(5))
	require.NoError(t, err)

	sel := doc.Recipe().Steps[1].Selectors[0]
	require.NotSame(t, decad.Selector(q), sel, `the recorded selector is a deep copy`)
	buf, err := json.Marshal(doc.Recipe())
	require.NoError(t, err)
	require.Contains(t, string(buf), "parallel_to", `the query's predicate is recorded, unresolved`)
}

func TestFilletConcaveEdgeReadsMinRadius(t *testing.T) {
	// An L-shaped section has one reflex (concave) lateral edge; rounding it
	// fills material in with a concave arc, and the minimum-radius survey reads
	// its radius (Table D, D3).
	const r = 5.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 0}, {40, 0}, {40, 20}, {20, 20}, {20, 40}, {0, 40}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// Exactly one concave lateral edge — the reflex corner.
	concave := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave())
	picked, err := concave.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, picked, 1, `an L has one reflex corner`)

	filleted, err := body.Fillet(concave, units.Millimeters(r))
	require.NoError(t, err)
	requireManifold(t, filleted)

	// The reflex fillet ADDS material: a (1 − π/4)r² wedge per corner.
	lArea := 40.0*40.0 - 20.0*20.0
	wantVol := (lArea + (1-math.Pi/4)*r*r) * 10
	vol, err := filleted.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// The blend wall is a concave cylinder of radius r, and the survey reads it.
	cylinders := 0
	for _, f := range filleted.Faces() {
		if cyl, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
			require.True(t, cyl.Radius.Equal(units.Millimeters(r), 1e-9))
		}
	}
	require.Equal(t, 1, cylinders)

	rep, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.NotNil(t, rep.Bodies[0].MinRadius, `a concave fillet is a concave feature`)
	got, err := rep.Bodies[0].MinRadius.Value.In(units.Millimeter)
	require.NoError(t, err)
	require.InDelta(t, r, got, 1e-9, `the survey sees the fillet's radius`)
}

func TestFilletLineArcCorner(t *testing.T) {
	// A quarter-disk prism: the corner where the flat wall meets the curved
	// wall is a line/arc corner. Rounding it builds, exactly, adding one blend
	// cylinder to the section's own quarter cylinder.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	origVol, err := body.Volume()
	require.NoError(t, err)
	origV, err := origVol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)

	// The convex corner at (20, 0) where the +x wall meets the arc.
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()).AtLeast(1)
	before, err := corner.SelectEdges(body)
	require.NoError(t, err)
	require.NotEmpty(t, before)

	filleted, err := body.Fillet(corner, units.Millimeters(4))
	require.NoError(t, err)
	requireManifold(t, filleted)

	vol, err := filleted.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	gotV, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, gotV, origV, `rounding convex corners removes material`)

	cylinders := 0
	for _, f := range filleted.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
	}
	require.GreaterOrEqual(t, cylinders, 2, `the section's own quarter cylinder plus the blend`)
}

func TestFilletArcArcCorner(t *testing.T) {
	// A lens of two circular arcs: each of its two pointed corners is an
	// arc/arc corner. Rounding both builds exactly, replacing each corner with
	// a blend cylinder.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p := s.CreatePoint(-20, 0)
	s.Fix(p)
	q := s.CreatePoint(20, 0)
	s.Fix(q)
	c1 := s.CreatePoint(0, -10)
	s.Fix(c1)
	c2 := s.CreatePoint(0, 10)
	s.Fix(c2)
	s.CreateArc(c1, q, p) // the upper arc
	s.CreateArc(c2, p, q) // the lower arc
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	origVol, err := body.Volume()
	require.NoError(t, err)
	origV, err := origVol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)

	corners := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	picked, err := corners.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, picked, 2, `a lens has two arc/arc corners`)

	filleted, err := body.Fillet(corners, units.Millimeters(3))
	require.NoError(t, err)
	requireManifold(t, filleted)

	require.Len(t, filleted.Faces(), 6, `two arc walls + two blend cylinders + two caps`)
	cylinders := 0
	for _, f := range filleted.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
	}
	require.Equal(t, 4, cylinders)

	vol, err := filleted.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	gotV, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, gotV, origV, `rounding the convex corners removes material`)
}

func TestFilletRefusals(t *testing.T) {
	_, box := filletBox(t)

	// A zero radius is the body the caller already holds: S13.
	_, err := box.Fillet(verticalEdges(), units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A wrong-kind magnitude is S15.
	_, err = box.Fillet(verticalEdges(), units.Degrees(5))
	require.ErrorIs(t, err, decad.ErrUnitKind)

	// A negative magnitude is S15.
	_, err = box.Fillet(verticalEdges(), units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)

	// A selector matching nothing is loud: S16.
	_, err = box.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave()), units.Millimeters(5))
	require.ErrorIs(t, err, decad.ErrNoMatch, `a box has no concave lateral edge`)

	// A cap-edge selector is the vertex-blend problem: S1, staged.
	capEdges := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)))
	_, err = box.Fillet(capEdges, units.Millimeters(5))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a fillet of a cap edge is not supported`)

	// The refusals left the document untouched — the box is still live.
	require.Equal(t, []*decad.Body{box}, box.Document().Bodies())
}

func TestFilletTooLargeRadius(t *testing.T) {
	// A radius larger than a circular carrier's own radius has no blend centre:
	// its inward offset is empty (S5, ErrDegenerate). The caller is refused,
	// never handed a clipped body.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()).AtLeast(1)
	_, err = body.Fillet(corner, units.Millimeters(25))
	require.ErrorIs(t, err, decad.ErrDegenerate, `no blend of radius 25 fits inside a wall of radius 20`)
	require.Equal(t, []*decad.Body{body}, doc.Bodies(), `a refused fillet retires nothing`)
}

// plateWithSquareHole extrudes a 100×100 plate with a 10×10 square hole whose
// lower-left corner is at (off, off), by 20 mm — a straight prism whose section
// is a rectangle with a rectangular hole.
func plateWithSquareHole(t *testing.T, off float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	s.CreateRectangle(off, off, off+10, off+10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-square-hole region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

func TestFilletBoundaryContactRefused(t *testing.T) {
	// A large fillet can bring the rewritten section's loops into BOUNDARY
	// CONTACT — here the outer corner's blend arc passes exactly through the
	// hole's corner (the hole sits at r·(1 − 1/√2) on the corner diagonal), a
	// shared boundary point with no interior crossing. The §5 audit must refuse
	// it: without the contact test Fillet returns a body Verify calls Sound but
	// Tessellate refuses, a silently inconsistent solid.
	const r = 20.0
	off := r * (1 - 1/math.Sqrt2)
	_, body := plateWithSquareHole(t, off)

	convex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	picked, err := convex.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, picked, 4, `the four outer corners are the convex lateral edges`)

	_, err = body.Fillet(convex, units.Millimeters(r))
	require.Error(t, err, `a fillet that pinches two boundaries must be refused, not returned`)
	require.ErrorIs(t, err, decad.ErrUnsupported, `boundary contact is the boundary case of a crossing: S7, ErrUnsupported`)
	require.Equal(t, []*decad.Body{body}, body.Document().Bodies(), `a refused fillet retires nothing`)
}

func TestFilletClearOfHoleBuilds(t *testing.T) {
	// The same plate-with-hole, but a radius whose blend arcs stay well clear of
	// the hole: the rewritten loops are disjoint, so the fillet builds and the
	// body is watertight and Sound — the widened audit does not over-reject.
	const r = 5.0
	off := 20 * (1 - 1/math.Sqrt2) // the hole that r = 20 would have touched
	doc, body := plateWithSquareHole(t, off)

	filleted, err := body.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()), units.Millimeters(r))
	require.NoError(t, err, `a fillet clear of the hole must still build`)
	requireManifold(t, filleted)

	mesh, err := filleted.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err, `the clear fillet's loops are disjoint, so it tessellates`)
	require.NotEmpty(t, mesh.Triangles())

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, rep.Trustworthy(), `a fillet clear of the hole is Sound`)
}

func TestFilletNonPrismReceiver(t *testing.T) {
	// A revolve is not a prismPayload, so a fillet of it is staged: S3.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	uAxis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = body.Fillet(decad.Edges(decad.Circular()), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported, `this evaluator fillets a straight prism only`)
}
