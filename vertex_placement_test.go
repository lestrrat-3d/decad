package decad_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's T96-T100: the vertex-bound
// repair for prism_build.go, revolve_build.go and capblend_geom.go, the
// three analytic builders that published Vertex.Position() Exact with a
// zero bound for a placed body or a body on a tilted sketch plane, although
// pp.point/rp.point's own frame lift and accumulated placement rounds under
// exactly those conditions (docs/evaluator-design.md §8; topology.go's
// Vertex.Position contract).

// ratVecOf lifts a world point to the exact rational vector it is.
func ratVecOf(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)}
}

// exactApplyRat composes xform's own basis and translation over big.Rat.
func exactApplyRat(xform r3.Transform, local [3]*big.Rat) [3]*big.Rat {
	basis := xform.Basis()
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	tr := ratVecOf(xform.Translation())
	out := [3]*big.Rat{}
	for i := range out {
		out[i] = new(big.Rat).Add(
			new(big.Rat).Add(new(big.Rat).Mul(ex[i], local[0]), new(big.Rat).Mul(ey[i], local[1])),
			new(big.Rat).Add(new(big.Rat).Mul(ez[i], local[2]), tr[i]),
		)
	}
	return out
}

// exactFramePoint computes, entirely over big.Rat, the world point a
// plane-local (u, v, z) denotes under frame then xform — prismPayload.point's
// own construction (prism_payload.go), reproduced without its float
// rounding, so it states the value a published Vertex.Bound must cover the
// gap to.
func exactFramePoint(frame r3.Frame, xform r3.Transform, u, v, z float64) [3]*big.Rat {
	origin, fu, fv, fn := ratVecOf(frame.Origin()), ratVecOf(frame.U()), ratVecOf(frame.V()), ratVecOf(frame.N())
	ru, rv, rz := ratOf(u), ratOf(v), ratOf(z)
	local := [3]*big.Rat{}
	for i := range local {
		local[i] = new(big.Rat).Add(
			new(big.Rat).Add(origin[i], new(big.Rat).Mul(fu[i], ru)),
			new(big.Rat).Add(new(big.Rat).Mul(fv[i], rv), new(big.Rat).Mul(fn[i], rz)),
		)
	}
	return exactApplyRat(xform, local)
}

// vertexResidual is the L-infinity distance between a held vertex value and
// the exact-rational truth, rounded UP into a float64 — the same discipline
// exactResidual (prism_boolean_displacement_test.go) takes for a scalar, so a
// bound that failed to contain its error cannot be flattered into passing by
// reading half an ulp off the wrong side.
func vertexResidual(held r3.Vec, truth [3]*big.Rat) float64 {
	comps := [3]float64{held.X, held.Y, held.Z}
	worst := 0.0
	for i, c := range comps {
		d := new(big.Rat).Sub(ratOf(c), truth[i])
		d.Abs(d)
		f, exact := d.Float64()
		if !exact {
			f = math.Nextafter(f, math.Inf(1))
		}
		worst = math.Max(worst, f)
	}
	return worst
}

// closestVertex is the vertex among vs whose position sits nearest truth —
// the fixture's own corner correspondence, found by distance rather than by
// index, since a build's own vertex order is not part of its contract.
func closestVertex(t *testing.T, vs []*decad.Vertex, truth [3]*big.Rat) *decad.Vertex {
	t.Helper()
	var best *decad.Vertex
	bestRes := math.Inf(1)
	for _, v := range vs {
		if r := vertexResidual(v.Position().Value, truth); r < bestRes {
			bestRes = r
			best = v
		}
	}
	require.NotNil(t, best)
	return best
}

// xyFrame is the world XY plane at the origin — w.XY()'s own frame, restated
// so a test can compute an exact-rational truth without reading it back off
// a body's private payload.
func xyFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// tiltedPlaneSketch builds a solved 100x60 rectangle on a genuinely tilted,
// non-origin sketch plane — the same frame offAxisPlateSketch (stitch_test.go)
// builds its own sketch on, restated here so this file can also hand back the
// r3.Frame a truth computation needs without reading a body's private payload.
func tiltedPlaneSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile, r3.Frame) {
	t.Helper()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	ref := r3.NewVec(1, 0, 0)
	u, ok := ref.Sub(axis.Scale(ref.Dot(axis))).Normalize()
	require.True(t, ok)
	frame, err := r3.NewFrame(r3.NewVec(7, -3, 11), u, axis.Cross(u))
	require.NoError(t, err)

	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0], frame
}

// generalMotion is a rotation about a general (non-axis, non-origin) axis
// composed with a translation — the shape the investigation's own measured
// placed-body cases use.
func generalMotion(t *testing.T, degrees, shift float64) r3.Transform {
	t.Helper()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	rot, err := r3.Rotation(axis, units.Degrees(degrees))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	motion, err := rot.Then(trans)
	require.NoError(t, err)
	return motion
}

// requireEnclosedVertex is T96/T97/T98/T99's shared assertion: the vertex
// nearest truth is Approximate, its bound is strictly positive, and the
// exact-rational residual against truth sits inside that bound.
func requireEnclosedVertex(t *testing.T, vs []*decad.Vertex, truth [3]*big.Rat) float64 {
	t.Helper()
	v := closestVertex(t, vs, truth)
	pos := v.Position()
	require.Equal(t, decad.Approximate, pos.Exactness)
	require.Positive(t, pos.Bound.Base())
	res := vertexResidual(pos.Value, truth)
	require.LessOrEqualf(t, res, pos.Bound.Base(),
		"residual %g must sit inside the published bound %g", res, pos.Bound.Base())
	return res
}

// TestVertexPlacedPrismBoundEnclosesDisplacement is T96: a placed prism's
// rim vertices report Approximate with a positive bound, and that bound
// encloses the true displacement between the held (float) vertex and the
// exact-rational point the record, frame and placement denote (computed over
// math/big.Rat, never a second float answer). Shown-to-fail: deleting
// prism_build.go's frameLiftAllow term (buildLoopSidesAs) turns every vertex
// back to Exact with a zero bound, and the Approximate/Positive assertions
// below go red.
func TestVertexPlacedPrismBoundEnclosesDisplacement(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := boxBody(t, doc, 0, 0, 100, 60, 40)
	motion := generalMotion(t, 37, 500)
	placed, err := body.Placed(t.Context(), motion)
	require.NoError(t, err)

	frame := xyFrame(t)
	worst := 0.0
	for _, uv := range [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}} {
		for _, z := range []float64{0, 40} {
			truth := exactFramePoint(frame, motion, uv[0], uv[1], z)
			worst = math.Max(worst, requireEnclosedVertex(t, placed.Vertices(), truth))
		}
	}
	require.Positive(t, worst, "the fixture must genuinely displace at least one vertex")
}

// TestVertexTiltedPlaneUnplacedBoundEnclosesDisplacement is T97: the case
// decision 2 of the vertex-bound repair exists for — a body drawn on a
// tilted, non-axis-aligned sketch plane with NO placement at all. Lifting a
// plane-local coordinate through a non-axis-aligned frame rounds under the
// identity transform too, so this body's rim vertices must ALSO report
// Approximate with a positive, enclosing bound. Shown-to-fail: reverting
// bounds.go's frameAndPlacementRoundAllow to gate on `xform != r3.Identity()`
// alone (the loft/stitch/patch/unstitch pattern, which never has to consider
// its OWN frame lift) turns this case's Approximate/Positive assertions red
// while T100 stays green — this is the row that catches it.
func TestVertexTiltedPlaneUnplacedBoundEnclosesDisplacement(t *testing.T) {
	t.Parallel()
	s, p, frame := tiltedPlaneSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	require.NoError(t, err)

	identity := r3.Identity()
	worst := 0.0
	for _, uv := range [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}} {
		for _, z := range []float64{0, 40} {
			truth := exactFramePoint(frame, identity, uv[0], uv[1], z)
			worst = math.Max(worst, requireEnclosedVertex(t, body.Vertices(), truth))
		}
	}
	require.Positive(t, worst, "the fixture must genuinely displace at least one vertex")
}

// TestVertexPlacedRevolveBoundEnclosesDisplacement is T98: a placed
// revolve's cap vertices report Approximate with a positive, enclosing
// bound. The fixture's quarter turn starts at phi0 = 0 (Along), where
// sin/cos evaluate exactly (1, 0) with no libm rounding of their own, which
// isolates the frame-lift/placement charge this row tests from the
// angular-denotation charge revolve_bounds_test.go already covers.
// Shown-to-fail: deleting revolve_build.go's revolveVertexFrameLiftAllow
// term turns the Approximate/Positive assertions red.
func TestVertexPlacedRevolveBoundEnclosesDisplacement(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	motion := generalMotion(t, 37, 0)
	placed, err := body.Placed(t.Context(), motion)
	require.NoError(t, err)

	frame := xyFrame(t)
	worst := 0.0
	// annularSketch's rectangle is u in [0,10], v in [5,15]; a phi0 = 0
	// vertex's world position before placement is exactly (u, v, 0).
	for _, uv := range [][2]float64{{0, 5}, {0, 15}, {10, 5}, {10, 15}} {
		truth := exactFramePoint(frame, motion, uv[0], uv[1], 0)
		worst = math.Max(worst, requireEnclosedVertex(t, placed.Vertices(), truth))
	}
	require.Positive(t, worst, "the fixture must genuinely displace at least one vertex")
}

// TestVertexPlacedCapBlendBoundEnclosesDisplacement is T99: a placed
// cap-blend (chamfer) body's cap-level vertices report Approximate with a
// positive, enclosing bound. A 5 mm chamfer on a rectangular cap loop offsets
// each corner inward by exactly 5 mm along both adjacent edges, so the
// cap-level corner at plane-local (5, 5) is exact by construction, and the
// truth needs no offset-solve reproduction. Shown-to-fail: deleting
// capblend_geom.go's frameLiftAllow term (buildCapBand) turns the
// Approximate/Positive assertions red.
func TestVertexPlacedCapBlendBoundEnclosesDisplacement(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 100, 60, 20)
	loop := decad.Edges(decad.CreatedBy(decad.CapEnd(box)))
	body, err := box.Chamfer(t.Context(), loop, units.Millimeters(5))
	require.NoError(t, err)
	motion := generalMotion(t, 37, 500)
	placed, err := body.Placed(t.Context(), motion)
	require.NoError(t, err)

	frame := xyFrame(t)
	worst := 0.0
	for _, uv := range [][2]float64{{5, 5}, {95, 5}, {95, 55}, {5, 55}} {
		truth := exactFramePoint(frame, motion, uv[0], uv[1], 20)
		worst = math.Max(worst, requireEnclosedVertex(t, placed.Vertices(), truth))
	}
	require.Positive(t, worst, "the fixture must genuinely displace at least one vertex")
}

// TestVertexAxisAlignedUnplacedBoundStaysZero is T100: the charge the four
// rows above pin must not silently widen the common, exact-arithmetic case.
// An axis-aligned, unplaced prism, revolve and cap-blend body must each keep
// every vertex Exact with a zero bound, exactly as before this repair.
// Shown-to-fail: dropping frameAndPlacementRoundAllow's own trivial-frame
// gate (charging unconditionally whenever maxInputAbs is nonzero) turns
// every one of these assertions red.
func TestVertexAxisAlignedUnplacedBoundStaysZero(t *testing.T) {
	t.Parallel()

	requireAllZero := func(t *testing.T, vs []*decad.Vertex) {
		t.Helper()
		for _, v := range vs {
			pos := v.Position()
			require.Equal(t, decad.Exact, pos.Exactness)
			require.Equal(t, units.Millimeters(0), pos.Bound)
		}
	}

	t.Run("prism", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		body := boxBody(t, doc, 0, 0, 100, 60, 40)
		requireAllZero(t, body.Vertices())
	})

	t.Run("revolve", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		requireAllZero(t, body.Vertices())
	})

	t.Run("capBlend", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		box := boxBody(t, doc, 0, 0, 100, 60, 20)
		loop := decad.Edges(decad.CreatedBy(decad.CapEnd(box)))
		body, err := box.Chamfer(t.Context(), loop, units.Millimeters(5))
		require.NoError(t, err)
		requireAllZero(t, body.Vertices())
	})
}
