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

func TestChamferContextCancellationLeavesReceiverLive(t *testing.T) {
	doc, box := filletBox(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	body, err := box.ChamferContext(ctx, verticalEdges(), units.Millimeters(10))
	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
	body, err = box.ChamferContext(t.Context(), verticalEdges(), units.Millimeters(10))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
}

func TestChamferContextCancellationAtCommitLeavesReceiverLive(t *testing.T) {
	doc, box := filletBox(t)
	before := doc.Recipe()
	ctx := &commitBoundaryCancelContext{Context: t.Context()}

	body, err := box.ChamferContext(ctx, verticalEdges(), units.Millimeters(10))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestChamferSelectorAdmission(t *testing.T) {
	t.Run("BuiltInQuery", func(t *testing.T) {
		doc, box := filletBox(t)

		_, err := box.Chamfer(verticalEdges(), units.Millimeters(5))
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

		_, err := box.Chamfer(foreign, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Zero(t, calls, `Chamfer rejects a foreign selector before it can retire the receiver`)
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

		_, err := box.Chamfer(selector, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, before, doc.Recipe(), `a typed nil selector records no step`)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a typed nil selector leaves the receiver live`)
	})
}

func TestChamferBoxAllConvexEdges(t *testing.T) {
	const d = 10.0
	h := filletBoxHeight
	doc, box := filletBox(t)

	// The selector resolves to exactly the four lateral edges, not the cap
	// rims — so chamfering them bevels exactly the four corners.
	edges, err := verticalEdges().SelectEdges(box)
	require.NoError(t, err)
	require.Len(t, edges, 4, `a box has four lateral edges`)

	body, err := box.Chamfer(verticalEdges(), units.Millimeters(d))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume: an equal-setback chamfer of a right-angle corner removes a right
	// isosceles triangle of legs d,d — area d²/2 per corner — so the section
	// loses 4·(d²/2), hand-derived and exact.
	capArea := 100.0*60.0 - 4*(d*d/2)
	wantVol := capArea * h
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12), `a chamfer introduces no bound`)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// Area: each corner replaces a right angle by trimming d off two walls and
	// adding a chord of length d√2, so the perimeter loses 8d and gains 4d√2.
	perimeter := 2*(100.0+60.0) - 8*d + 4*d*math.Sqrt2
	wantArea := perimeter*h + 2*capArea
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, gotArea, 1e-9)

	// Topology: four straight walls + four planar bevels + two caps — every one
	// a Plane, no cylinder anywhere (the bevel is a chord, not a rolling ball).
	require.Len(t, body.Faces(), 10)
	planes, cylinders := 0, 0
	for _, f := range body.Faces() {
		switch f.Surface().(type) {
		case decad.Plane:
			planes++
		case decad.Cylinder:
			cylinders++
		}
	}
	require.Equal(t, 10, planes, `every face of a chamfered box is a plane`)
	require.Equal(t, 0, cylinders, `a chamfer bevel is a plane, never a cylinder`)

	// Each bevel wall is a Plane carrying a second chamfer(i,j) role naming the
	// same (loop, segment) its side(i,j) role does.
	bevels := 0
	for _, f := range body.Faces() {
		hasChamfer, hasSide := false, false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamfer(") {
				hasChamfer = true
			}
			if strings.HasPrefix(o.Role, "side(") {
				hasSide = true
			}
		}
		if !hasChamfer {
			continue
		}
		_, isPlane := f.Surface().(decad.Plane)
		require.True(t, isPlane, `a chamfer bevel wall is a Plane`)
		require.True(t, hasSide, `a bevel wall keeps its side role`)
		bevels++
	}
	require.Equal(t, 4, bevels, `one bevel plane per chamfered corner`)

	// A planar bevel has no concave principal radius, so the minimum-radius
	// survey rightly reports nothing (Table D, D3) — unlike a concave fillet.
	rep, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.Nil(t, rep.Bodies[0].MinRadius, `a planar chamfer bevel is not a concave radius`)
	require.False(t, rep.Trustworthy(), `the irrational bevel area carries a nonzero bound`)
}

func TestChamferRecipeAndRetire(t *testing.T) {
	const d = 8.0
	doc, box := filletBox(t)
	body, err := box.Chamfer(verticalEdges(), units.Millimeters(d))
	require.NoError(t, err)

	// The receiver is retired; the document holds the chamfered body.
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	_, err = box.Chamfer(verticalEdges(), units.Millimeters(d))
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	// The step records the op, the receiver as its input, the unresolved
	// selector, the distance value, and no options — and it round-trips.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	chamfer := recipe.Steps[1]
	require.Equal(t, decad.OpChamfer, chamfer.Op)
	require.Equal(t, []decad.StepRef{0}, chamfer.Inputs)
	require.Len(t, chamfer.Selectors, 1)
	require.Len(t, chamfer.Values, 1)
	require.True(t, chamfer.Values[0].Equal(units.Millimeters(d), 1e-12))
	require.Nil(t, chamfer.Opts, `a chamfer Step takes no options this increment`)

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded chamfer recipe round-trips`)
}

func TestChamferSelectorIsRecordedUnresolved(t *testing.T) {
	// The step stores a clone of the query, not the caller's, and never the
	// edges it resolved to (core §9 / §11).
	doc, box := filletBox(t)
	q := verticalEdges()
	_, err := box.Chamfer(q, units.Millimeters(5))
	require.NoError(t, err)

	sel := doc.Recipe().Steps[1].Selectors[0]
	require.NotSame(t, decad.Selector(q), sel, `the recorded selector is a deep copy`)
	buf, err := json.Marshal(doc.Recipe())
	require.NoError(t, err)
	require.Contains(t, string(buf), "parallel_to", `the query's predicate is recorded, unresolved`)
}

func TestChamferConcaveEdgeAddsWedge(t *testing.T) {
	// An L-shaped section has one reflex (concave) lateral edge; chamfering it
	// fills material in with a planar bevel, ADDING a right isosceles wedge of
	// area d²/2 — the mirror of a convex corner, which removes one.
	const d = 5.0
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

	// Exactly one concave lateral edge — the reflex corner at (20,20).
	concave := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave())
	picked, err := concave.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, picked, 1, `an L has one reflex corner`)

	chamfered, err := body.Chamfer(concave, units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	// The reflex chamfer ADDS material: a d²/2 wedge for the one corner.
	lArea := 40.0*40.0 - 20.0*20.0
	wantVol := (lArea + d*d/2) * 10
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// The bevel is a plane, not a cylinder — even at a concave corner — so the
	// minimum-radius survey reports nothing (a sharp concave edge has no radius).
	for _, f := range chamfered.Faces() {
		_, isCyl := f.Surface().(decad.Cylinder)
		require.False(t, isCyl, `a concave chamfer is a plane, never a cylinder`)
	}
	rep, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.Nil(t, rep.Bodies[0].MinRadius, `a planar chamfer bevel has no concave radius`)
}

func TestChamferLineArcCorner(t *testing.T) {
	// A quarter-disk prism has three convex corners: the right angle at the
	// origin (line/line) and the two junctions at (20,0) and (0,20) where a flat
	// wall meets the curved wall (line/arc). Bevelling all three builds, exactly —
	// each chord from a point on the line to a point on the arc is a plane, and
	// the section's own quarter cylinder stays a cylinder. (S5 has no chamfer
	// case: a chord exists between the two feet even though the neighbour is
	// circular, §7.)
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

	// All three convex corners — two of them line/arc junctions.
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	before, err := corner.SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, before, 3, `a quarter disk has three convex corners`)

	chamfered, err := body.Chamfer(corner, units.Millimeters(4))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotV, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, gotV, origV, `bevelling a convex corner removes material`)

	// The section's own quarter cylinder survives; the bevel adds a plane.
	cylinders, bevelPlanes := 0, 0
	for _, f := range chamfered.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
		for _, ro := range f.Origins() {
			if strings.HasPrefix(ro.Role, "chamfer(") {
				bevelPlanes++
			}
		}
	}
	require.Equal(t, 1, cylinders, `the quarter cylinder stays one cylinder wall; every bevel is a plane`)
	require.Equal(t, 3, bevelPlanes, `three planar bevels, two of them from line/arc corners`)
}

func TestChamferArcArcCorner(t *testing.T) {
	// A lens of two circular arcs: each of its two pointed corners is an arc/arc
	// corner. Bevelling both builds exactly, replacing each corner with a planar
	// chord between the two arc feet.
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

	chamfered, err := body.Chamfer(corners, units.Millimeters(3))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	// Two arc walls + two bevel planes + two caps.
	require.Len(t, chamfered.Faces(), 6, `two arc walls + two bevel planes + two caps`)
	cylinders, bevels := 0, 0
	for _, f := range chamfered.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
		for _, ro := range f.Origins() {
			if strings.HasPrefix(ro.Role, "chamfer(") {
				bevels++
			}
		}
	}
	require.Equal(t, 2, cylinders, `the two arc walls stay cylinders`)
	require.Equal(t, 2, bevels, `each pointed corner gets one planar bevel`)

	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotV, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, gotV, origV, `bevelling the convex corners removes material`)
}

func TestChamferHasNoS5Path(t *testing.T) {
	// A fillet of a circular carrier refuses when the radius exceeds the wall's
	// own radius: its inward offset is empty (S5, ErrDegenerate). A chamfer has
	// NO such refusal — a chord exists between any two distinct feet — so the same
	// corner with a large-but-fitting setback builds. Any chamfer refusal comes
	// from the §5 audit (S4/S6/S7/S8/S9), never an S5 "no blend of that radius".
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

	// A fillet radius of 25 is S5 (ErrDegenerate) on this wall of radius 20.
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()).AtLeast(1)
	_, err = body.Fillet(corner, units.Millimeters(25))
	require.ErrorIs(t, err, decad.ErrDegenerate, `fillet r=25 has no blend centre on a wall of radius 20: S5`)

	// The same convex corner takes a chamfer with a comfortable setback: a chord
	// always exists, so there is no S5 gate to fail.
	chamfered, err := body.Chamfer(corner, units.Millimeters(4))
	require.NoError(t, err, `a chamfer has no S5 path — a chord exists between any two feet`)
	requireManifold(t, chamfered)
}

func TestChamferOverLargeSetbackRefused(t *testing.T) {
	// A setback so large its two feet run past the far end of a wall is S6: the
	// wall is consumed by its own corners. The caller is refused (ErrUnsupported),
	// never handed a clipped body. Here the 60 mm walls each carry two corners at
	// d = 40 → 80 > 60, so the audit refuses.
	_, box := filletBox(t)
	selector := verticalEdges()
	_, err := box.Chamfer(selector, units.Millimeters(40))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a setback that consumes a wall from both ends is S6`)
	require.ErrorContains(t, err, `selector `+selector.String(),
		`an audit failure retains the multi-edge query`)
	require.Equal(t, 4, strings.Count(err.Error(), `selected edge[`),
		`the audit failure retains all four selected-edge to corner mappings`)
	require.Regexp(t, `loop 0 walk \d+ from corner \d+ at \(u, v\) = \([^)]+\) to corner \d+ at \(u, v\) = \([^)]+\)`,
		err.Error(), `the overrun identifies the consumed walk and both corner coordinates`)
	require.Equal(t, []*decad.Body{box}, box.Document().Bodies(), `a refused chamfer retires nothing`)
}

func TestChamferOverLargeSetbackFlippingLoopIsUnsupported(t *testing.T) {
	// An over-large setback whose feet run past their adjacent walls' far ends is
	// S6 (ErrUnsupported), even when the same overrun ALSO turns the rewritten
	// loop inside out — Table S assigns the overrun to S6, so it must NOT be
	// misfiled as the S8 inside-out verdict (ErrDegenerate). At d = 120 on the
	// 100×60 box every corner's own chord runs past both its walls, and the
	// rewritten section's signed area flips sign, so the audit's S8 (asked first)
	// would grab it if S6 did not own it.
	_, box := filletBox(t)
	_, err := box.Chamfer(verticalEdges(), units.Millimeters(120))
	require.ErrorIs(t, err, decad.ErrUnsupported, `an overrun that also flips the loop is still S6`)
	require.NotErrorIs(t, err, decad.ErrDegenerate, `the overrun must not read as the S8 inside-out verdict`)
	require.Equal(t, []*decad.Body{box}, box.Document().Bodies(), `a refused chamfer retires nothing`)
}

func TestChamferRefusals(t *testing.T) {
	_, box := filletBox(t)

	// A zero distance is the body the caller already holds: S13.
	_, err := box.Chamfer(verticalEdges(), units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A wrong-kind magnitude is S15.
	_, err = box.Chamfer(verticalEdges(), units.Degrees(5))
	require.ErrorIs(t, err, decad.ErrUnitKind)

	// A negative magnitude is S15.
	_, err = box.Chamfer(verticalEdges(), units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)

	// A selector matching nothing is loud: S16.
	_, err = box.Chamfer(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave()), units.Millimeters(5))
	require.ErrorIs(t, err, decad.ErrNoMatch, `a box has no concave lateral edge`)

	// A cap-edge selector is the vertex-blend problem: S1, staged.
	capEdges := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)))
	_, err = box.Chamfer(capEdges, units.Millimeters(5))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a chamfer of a cap edge is not supported`)

	// The refusals left the document untouched — the box is still live.
	require.Equal(t, []*decad.Body{box}, box.Document().Bodies())
}

func TestChamferNonPrismReceiver(t *testing.T) {
	// A revolve is not a prismPayload, so a chamfer of it is staged: S3.
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
	_, err = body.Chamfer(sel, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported, `this evaluator chamfers a straight prism only`)
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

func TestChamferBreaksNestingRefused(t *testing.T) {
	// A large corner chamfer can shrink the outer loop PAST a near-corner hole: the
	// (0,0) corner's d = 20 chord runs from (0,20) to (20,0), and the hole at
	// (3,3) r = 1 lies entirely inside the removed triangle — OUTSIDE the bevelled
	// outer loop, yet never crossing or touching an outer segment. Only the §5
	// test-4 containment audit catches it: the hole is PROVABLY outside, so the
	// nesting is decidably broken — no such body exists — and it is refused
	// (ErrDegenerate), never returned as a body Verify would call Sound but
	// Tessellate could not build.
	_, body := plateWithDiskHole(t, 3, 3, 1)

	convex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	_, err := body.Chamfer(convex, units.Millimeters(20))
	require.Error(t, err, `a chamfer that leaves the hole outside the outer loop must be refused`)
	require.ErrorIs(t, err, decad.ErrDegenerate,
		`a hole proven outside the bevelled outer loop is nesting decidably broken: ErrDegenerate`)
	require.ErrorContains(t, err, `hole loop 1 at (u, v) = `,
		`the nesting audit identifies the failed hole and its classification point`)
	require.ErrorContains(t, err, `outside outer loop 0`,
		`the nesting audit identifies the outer loop used for classification`)
	require.Equal(t, []*decad.Body{body}, body.Document().Bodies(), `a refused chamfer retires nothing`)
}

func TestChamferClearOfHoleBuilds(t *testing.T) {
	// The mirror of the refusal: a corner chamfer whose chord stays well clear of a
	// central hole builds, tessellates and is Sound — the shared audit does not
	// over-reject.
	doc, body := plateWithDiskHole(t, 50, 50, 5)

	convex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	chamfered, err := body.Chamfer(convex, units.Millimeters(5))
	require.NoError(t, err, `a chamfer clear of the hole must build`)
	requireManifold(t, chamfered)

	mesh, err := chamfered.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err, `a well-nested chamfered section tessellates`)
	require.NotEmpty(t, mesh.Triangles())

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.False(t, rep.Trustworthy(), `bounded analytic mass results need a nonzero tolerance`)
}
