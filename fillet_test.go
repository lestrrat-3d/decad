package decad_test

import (
	"context"
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

type commitBoundaryCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	calls           int
}

func (c *commitBoundaryCancelContext) Err() error {
	c.calls++
	if c.calls >= 2 {
		return context.Canceled
	}
	return nil
}

// retiringEdgeSelector is a foreign selector implementation that embeds the
// built-in query to promote Selector's sealed marker, then overrides resolution
// with a callback that would retire the receiver.
type retiringEdgeSelector struct {
	*decad.EdgeQuery
	calls *int
}

func (s retiringEdgeSelector) SelectEdges(body *decad.Body) ([]*decad.Edge, error) {
	(*s.calls)++
	edges, err := s.EdgeQuery.SelectEdges(body)
	if err != nil {
		return nil, err
	}
	_, err = body.Placed(r3.Identity())
	return edges, err
}

func TestFilletContextCancellationAtCommitLeavesReceiverLive(t *testing.T) {
	doc, box := filletBox(t)
	before := doc.Recipe()
	ctx := &commitBoundaryCancelContext{Context: t.Context()}

	body, err := box.FilletContext(ctx, verticalEdges(), units.Millimeters(10))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestFilletSelectorAdmission(t *testing.T) {
	t.Run("BuiltInQuery", func(t *testing.T) {
		doc, box := filletBox(t)

		_, err := box.Fillet(verticalEdges(), units.Millimeters(5))
		require.NoError(t, err)
		require.Len(t, doc.Recipe().Steps, 2)
		_, err = json.Marshal(doc.Recipe())
		require.NoError(t, err)
	})

	t.Run("ForeignRetiringImplementation", func(t *testing.T) {
		doc, box := filletBox(t)
		before := doc.Recipe()
		calls := 0
		foreign := retiringEdgeSelector{
			EdgeQuery: verticalEdges(),
			calls:     &calls,
		}

		_, err := box.Fillet(foreign, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Zero(t, calls, `Fillet rejects a foreign selector before it can retire the receiver`)
		require.Equal(t, before, doc.Recipe(), `a rejected selector records no step`)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a rejected selector leaves the receiver live`)
		_, err = json.Marshal(doc.Recipe())
		require.NoError(t, err)
	})

	t.Run("TypedNilQuery", func(t *testing.T) {
		doc, box := filletBox(t)
		before := doc.Recipe()
		var query *decad.EdgeQuery
		var selector decad.EdgeSelector = query

		_, err := box.Fillet(selector, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, before, doc.Recipe(), `a typed nil selector records no step`)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a typed nil selector leaves the receiver live`)
	})
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
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
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
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
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
	require.False(t, rep.Trustworthy(), `bounded circular mass results need a nonzero tolerance`)
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
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
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
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
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
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
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
	require.ErrorContains(t, err, `selector `+corner.String(),
		`a multi-edge construction failure retains the query that selected the corners`)
	require.Regexp(t, `selected edge\[\d+\] from \([^)]+\) to \([^)]+\) maps to loop 0 corner \d+ at \(u, v\) = \([^)]+\)`,
		err.Error(), `the failing selected edge retains its result ordinal and matched corner coordinate`)
	require.Equal(t, []*decad.Body{body}, doc.Bodies(), `a refused fillet retires nothing`)
}

func TestFilletOverLargeRadiusFlippingLoopIsUnsupported(t *testing.T) {
	// The fillet analogue of the chamfer overrun: a radius whose tangent feet run
	// past their adjacent walls' far ends is S6 (ErrUnsupported), even when the
	// rewrite also turns the loop inside out. At r = 120 on the 100×60 box each
	// corner's feet overrun both its walls and the rewritten section flips sign,
	// so the audit's S8 (asked first) would grab it if S6 did not own it. (This
	// is a different refusal from TestFilletTooLargeRadius, whose S5 rejects a
	// radius that exceeds a CIRCULAR carrier before any audit runs.)
	_, box := filletBox(t)
	_, err := box.Fillet(verticalEdges(), units.Millimeters(120))
	require.ErrorIs(t, err, decad.ErrUnsupported, `an overrun that also flips the loop is still S6`)
	require.NotErrorIs(t, err, decad.ErrDegenerate, `the overrun must not read as the S8 inside-out verdict`)
	require.Equal(t, []*decad.Body{box}, box.Document().Bodies(), `a refused fillet retires nothing`)
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
	require.ErrorContains(t, err, `selector `+convex.String(),
		`an audit failure retains the multi-edge query`)
	require.Regexp(t, `rewritten loop \d+ segment \d+ and loop \d+ segment \d+ are in contact`,
		err.Error(), `a pairwise audit failure names both implicated loop segments`)
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
	require.False(t, rep.Trustworthy(), `bounded circular mass results need a nonzero tolerance`)
}

// plateWithDiskHole extrudes a 100×100 plate with a circular hole of radius rho
// centred at (cx, cy), by 10 mm — a straight prism whose section is a rectangle
// with a circular hole.
func plateWithDiskHole(t *testing.T, cx, cy, rho float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 100)
	s.Fix(outer.A)
	s.CreateCircle(s.CreatePoint(cx, cy), rho)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the rectangle-with-disk region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

func TestFilletHoleOutsideRoundedLoopRefused(t *testing.T) {
	// A large fillet can shrink the outer loop PAST a near-corner hole without the
	// hole ever crossing or touching an outer segment: the (0,0) corner's r = 20
	// blend arc has centre (20,20), and the hole at (3,3) r = 1 lies at distance
	// √(17²+17²) − 1 ≈ 23 > 20 from that centre — entirely in the removed corner,
	// OUTSIDE the rounded material, yet ≈3 mm from the nearest outer segment (no
	// crossing, no contact). S8/S6/S7 all pass; only the §5 test-4 containment
	// audit catches it. The hole is PROVABLY outside the rewritten outer loop, so
	// the nesting is decidably broken — the fillet consumed the region the hole
	// lived in — and no such body exists: S8-family ErrDegenerate.
	_, body := plateWithDiskHole(t, 3, 3, 1)

	convex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	_, err := body.Fillet(convex, units.Millimeters(20))
	require.Error(t, err, `a fillet that leaves the hole outside the outer loop must be refused, not returned`)
	require.ErrorIs(t, err, decad.ErrDegenerate,
		`a hole proven outside the rounded outer loop is nesting decidably broken: no such body, ErrDegenerate`)
	require.ErrorContains(t, err, `hole loop 1 at (u, v) = `,
		`the nesting audit identifies the failed hole and its classification point`)
	require.ErrorContains(t, err, `outside outer loop 0`,
		`the nesting audit identifies the outer loop used for classification`)
	require.Equal(t, []*decad.Body{body}, body.Document().Bodies(), `a refused fillet retires nothing`)
}

func TestFilletHoleWellInsideRoundedLoopBuilds(t *testing.T) {
	// The mirror of the refusal: a hole comfortably inside the rounded outer loop
	// still builds and tessellates Sound. A small r = 5 fillet at (0,0) leaves the
	// hole at the plate's centre untouched and well contained, so the containment
	// audit passes and does not over-reject.
	doc, body := plateWithDiskHole(t, 50, 50, 5)

	convex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	filleted, err := body.Fillet(convex, units.Millimeters(5))
	require.NoError(t, err, `a fillet with the hole well inside the outer loop must build`)
	requireManifold(t, filleted)

	mesh, err := filleted.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err, `a well-nested filleted section tessellates`)
	require.NotEmpty(t, mesh.Triangles())

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.False(t, rep.Trustworthy(), `bounded circular mass results need a nonzero tolerance`)
}

// scaledDiskInCornerFillet builds a k-scaled plate whose (0,0) outer corner is
// rounded by a radius-5k convex fillet, with a circular hole placed ENTIRELY
// inside that fillet's disk so the rewritten arc clears the hole by a gap of
// gapMult × the section's scale-anchored noise floor (δ = ε·D, ε = 1e-9, D the
// section's (u, v) bounding-box diagonal). The hole sits at distance L on the
// corner diagonal with L + ρ = r − gap, so every hole point is within r of the
// arc centre and the hole circle never crosses the arc — a clean near-miss the
// §5 contact test judges against the floor. The whole figure scales with k, so
// the RELATIVE geometry — and the verdict — must not depend on k. It returns the
// fillet error (nil ⇒ built). Because the tiny gap is created by decad's own
// rewrite (not present in the sketch), the sketch sees only well-separated,
// normal-sized features and never collapses them.
func scaledDiskInCornerFillet(t *testing.T, k, gapMult float64) error {
	t.Helper()
	const eps = 1e-9
	d := math.Sqrt2 * 100 * k // the section's bounding-box diagonal, its scale
	floor := eps * d
	gap := gapMult * floor
	const r = 5.0
	const rho = 2.0            // hole radius, well inside the fillet disk
	l := (r - rho - gap/k) * k // centre distance along the diagonal; gap scales with k
	// hole centre on the (0,0) corner diagonal, at (r,r) − (L/√2)(1,1).
	cx := (r*k - l/math.Sqrt2)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100*k, 100*k)
	s.Fix(outer.A)
	s.CreateCircle(s.CreatePoint(cx, cx), rho*k)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Outer) == 4 && len(p.Holes) == 1 {
			prof = p
			break
		}
	}
	require.NotNil(t, prof, `the plate-with-disk region should exist`)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(20 * k), Dir: decad.Along})
	require.NoError(t, err)

	_, err = body.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()), units.Millimeters(r*k))
	return err
}

func TestFilletContactToleranceScaleInvariant(t *testing.T) {
	// The boundary-contact test is anchored to the SECTION'S scale (δ = ε·D),
	// not a fixed absolute band, so the SAME relative geometry decides the same
	// way at any absolute size. Here a gap of 10× the noise floor — comfortably
	// above it — clears at both a 1 mm part (k = 0.01) and a 1e6 mm part
	// (k = 1e4): both BUILD.
	//
	// This is exactly what the old fixed 1e-7 mm band violated. At that band the
	// 1 mm part's 1.41e-8 mm gap reads as contact and REFUSES, while the 1e6 mm
	// part's 1.41e-2 mm gap builds — the verdict FLIPS with absolute scale, and
	// at large scale the old band even ACCEPTS a genuine sub-floor pinch. The
	// build/refuse verdict below is the audit's; it holds identically at both
	// scales only for the scale-anchored threshold. (The gap is deliberately
	// near the floor, far below any chord tolerance, so this exercises the audit,
	// not tessellation — a macroscopic-gap build that also tessellates Sound is
	// TestFilletClearOfHoleBuilds.)
	const above = 10.0 // gap = 10× the floor: comfortably clear
	for _, k := range []float64{0.01, 1e4} {
		err := scaledDiskInCornerFillet(t, k, above)
		require.NoErrorf(t, err, `a gap 10× the floor must build at scale k=%g`, k)
	}

	// A true pinch (a gap far below the floor) refuses at BOTH scales: the
	// verdict tracks the relative geometry, not the absolute size — including the
	// large scale where the old fixed band would have accepted it.
	const pinch = 1e-4 // 1e-4 × floor: indistinguishable from contact
	for _, k := range []float64{0.01, 1e4} {
		err := scaledDiskInCornerFillet(t, k, pinch)
		require.ErrorIsf(t, err, decad.ErrUnsupported,
			`a sub-floor gap is a pinch and must refuse at scale k=%g`, k)
	}
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
	sel := decad.Edges(decad.Circular())
	_, err = body.Fillet(sel, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported, `this evaluator fillets a straight prism only`)
	require.ErrorContains(t, err, `selector `+sel.String())
	for _, want := range []string{
		`selected edge[0] from (0,5,0) to (0,5,0)`,
		`selected edge[1] from (10,5,0) to (10,5,0)`,
		`selected edge[2] from (10,15,0) to (10,15,0)`,
		`selected edge[3] from (0,15,0) to (0,15,0)`,
	} {
		require.ErrorContains(t, err, want)
	}
}
