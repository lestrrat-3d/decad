package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/surface-design.md §6.4's per-surface flux integral: the
// closed-form volume and first-moment reading that lets Stitch close a
// boundary holding a curved face, for the Plane and Cylinder variants. Table
// R row R8 is a per-face dispatch: a face this evaluator has no closed-form
// flux integral for refuses R8, and a Plane or Cylinder face with a zero
// normalBound admits.
//
// THE FORMULA. Volume is V = (1/3) Σ_F ∫_F (p−A)·n dA over one global anchor
// A (verts[0], the same anchor loft_moments.go's tetrahedron sum uses, so the
// tetrahedron and flux paths are anchored alike). Per face, with A_F a
// per-variant anchor and S_F = ∫_F n dA the face's own vector area:
//
//	flux_F = K_F + (A_F − A)·S_F
//
// Plane: A_F = Frame.Origin(), K_F = 0 — (p−A_F)·n is identically zero on the
// plane. Cylinder: A_F = Origin (any point on the axis — the formula is
// invariant to which one, since a shift along the axis cancels between the
// two terms below), K_F = σ·Radius·f.area, σ = −1 when f.reversed else +1 —
// (p−O)·n is identically σ·Radius at every point of a cylinder wall, so the
// K_F term reuses the face's own already-proven area/areaBound rather than
// integrating anything fresh.
//
// THE SCOPE RESTRICTION THIS INCREMENT ADDS, not in the original design
// sketch. A general trimmed Plane or Cylinder face needs S_F from the
// boundary's own ½∮p×dr contour sum, which has an arm for a straight (Line3)
// edge and one for a circular (Arc3/Circle3) edge alike. Every fixture this
// increment actually proves — an annular revolve sheet, a solid-of-revolution
// cylinder built the same way — bounds every Plane face with FULL circles
// only (never a partial arc) and every Cylinder face with exactly two full
// circular rims at two axial levels (never a partial revolution). Both
// admitted arms below are scoped to exactly that shape and refuse
// (ErrUnsupported, R8) anything wider: a Plane loop that is not a single
// Circle3 edge, or a Cylinder face that is not exactly two single-Circle3
// loops. This is deliberate, not an oversight: an untested closed form is not
// a proof, and CLAUDE.md's own rule is that a narrower answer always beats a
// wider guess. Widening either arm to a general Line3/Arc3 boundary is a
// later increment's own work, once a fixture exists to prove it against.
//
// One consequence of the restriction: because both admitted surfaces are
// FULL circles/full-circumference cylinders, each face's own vector area S_F
// collapses to a form simpler than the general contour sum would give —
// Plane's is exactly n·Area (a constant normal over a flat region has no
// other vector area to compute), and a full-circumference Cylinder's is
// exactly the zero vector (the radial normal's own trig integrates to zero
// over a full turn — proven in cylinderFaceFluxAndMoment's own doc comment).
// So this file never builds the general ½∮p×dr contour-sum helper the wider
// design sketch anticipated; it is not needed for either arm actually landed,
// and adding it unexercised would be untested code with no fixture behind
// it. A later increment's Cone/Sphere/Torus arms may still need it, and can
// add it then, against their own fixtures.
//
// THE FIRST MOMENT. Every reading needs the volume's own first-moment
// sibling too — Body.Centroid has no ErrUnsupported arm for a solid, so a
// curved solid's centroid is not optional (docs/surface-design.md §8) — via
// the identical divergence-theorem shape with F_i = ((x_i−a_i)²/2)·ê_i,
// div F_i = x_i−a_i: ∫_Ω(x_i−a_i)dV = Σ_F ∫_F ((p_i−a_i)²/2)·n_i dA. The
// Plane arm reduces this to the standard disk/annulus second-moment
// identity (a full circle of radius R centered at (cu,cv) in the face's own
// local frame contributes cu·Area, cv·Area, cu²·Area+πR⁴/4, cv²·Area+πR⁴/4
// and cu·cv·Area to Iu, Iv, Iuu, Ivv, Iuv respectively — elementary polar
// integration over a disk, summed with each loop's own outer/hole sign). The
// Cylinder arm's own closed form is derived and verified in
// cylinderFaceFluxAndMoment's doc comment.
//
// BOUNDS. Every operation between a bound's origin and its use is charged
// through boundedAdd/boundedSub/boundedMul/boundedQuotient — never a bare
// float composed by hand — so a mistake here shows up as a bound the
// shown-to-fail tests can watch go red, not as a silent understatement.
// Every term that carries π (every area- and fourth-moment-of-a-disk term,
// every Cylinder moment term) reads it from piScalar(): Go's math.Pi is the
// correctly rounded float64 nearest true π, so its own representation error
// is one ulp, and every subsequent multiply charges its own rounding
// through boundedMul on top of that — a tight bound, not
// conservativeValueError's structural "assume no cancellation at all"
// fallback, which is sized for a quantity this evaluator has no other way
// to bound and would overstate π's own error by many orders of magnitude —
// wide enough, on a thin annulus, to starve stitchCurvedMass's own
// boundedQuotient calls of clearance and refuse a perfectly sound fixture.
// Nothing here ever claims Exact: every admitting arm's own K_F or moment
// carries π, so exactnessOf's zero-bound test can never fire for a curved
// stitched solid.
//
// RULE S — the construction-proof gate. Before this file's dispatch ever
// runs, evalStitchContext requires every operand face to descend from ONE
// source body whose payload passes payloadProvesSimple (verify.go) —
// stitchRuleSAdmits below. The reused crossing audit (docs/loft-design.md
// §6) consumes triangles, and a curved face has none to chord into it
// without admitting a self-intersection question on an approximation
// (verify.go's own reject-only rule for exactly this). So a curved closed
// set's only available proof of non-self-intersection is the one its SOURCE
// feature already carries — a full-turn revolve's own axis argument, a
// surface-result prism or loft's own — and Rule S is what makes sure this
// file never publishes Volume/Centroid without it. A stitch of curved sheets
// from two different features, or a stitch downstream of Body.Patch (whose
// own payload carries no such proof — docs/surface-design.md §5.2), stays
// refused on this gate alone.

// stitchRuleSAdmits is Rule S: every operand face must descend from exactly
// one source body, and that body's own evaluator payload must prove its
// boundary simple by construction (payloadProvesSimple). srcFaces is the
// PRE-rebuild operand face set (stitch.go's own srcFaces, read before the
// new body's own topology and Face.body pointers exist) — the same set
// stitchBounds already reads operand bodies from.
func stitchRuleSAdmits(ctx context.Context, srcFaces []*Face) bool {
	bodies := stitchOperandBodies(srcFaces)
	if len(bodies) != 1 || bodies[0] == nil || bodies[0].payload == nil {
		return false
	}
	return payloadProvesSimple(ctx, bodies[0].payload)
}

// stitchAllTetrahedronEligible reports whether every face qualifies for
// loft_moments.go's exact-rational tetrahedron sum: a Plane face
// (docs/surface-design.md's "planar and straight-edged", never "planar")
// bounded entirely by Line3 edges, with a zero normalBound. Checking only
// the surface tag (Kind() == KindPlane) is not enough on its own:
// triangulateStitchFaces builds its polygon from coedge START vertices
// only and drops an arc's own bulge, so a Plane face with a circular
// boundary would silently mis-triangulate. No construction reaching this
// function puts a curved boundary on a Plane face without ALSO putting a
// genuinely curved SURFACE elsewhere in the same closed set (which routes
// through stitch_flux.go's own flux path instead), so the edge-kind check
// closes a shape that was latent rather than live — closed here regardless,
// since the split this design states is "Plane bounded entirely by Line3"
// versus everything else, never "planar versus curved".
func stitchAllTetrahedronEligible(faces []*Face) bool {
	for _, f := range faces {
		if !faceIsTetrahedronEligible(f) {
			return false
		}
	}
	return true
}

func faceIsTetrahedronEligible(f *Face) bool {
	if !f.isPlanar() || f.normalBound != 0 {
		return false
	}
	for _, l := range f.loops {
		for _, ce := range l.coedges {
			if _, ok := ce.edge.curve.(Line3); !ok {
				return false
			}
		}
	}
	return true
}

// auditVertexLinksForStitchFaces builds sweep_composite.go's own
// uses-by-edge map over a stitched face set and runs its hoisted
// auditVertexLinks: the manifold-with-boundary proof at every vertex, that
// the faces touching it form one connected fan (an interior vertex, a
// cycle) or one connected path (a rim vertex, on an open sheet). Stitch's
// own checkStitchClosure already proves every edge has one or two adjacent
// faces with correct forward/backward parity — a strictly weaker claim than
// this: it says nothing about whether, AT ONE VERTEX, the several faces and
// two-face edges meeting there stay in one piece, which is exactly the gap
// a curved profile that touches its own revolve axis at more than one
// isolated point could open (docs/surface-design.md's own record of the
// two-arc lens case). This is a reject-only gate: it can only narrow what
// the curved path admits, never bless anything checkStitchClosure did not
// already allow through.
func auditVertexLinksForStitchFaces(ctx context.Context, faces []*Face) error {
	budget := newWorkBudget(ctx)
	uses := map[*Edge][]compositeCoedgeUse{}
	for _, face := range faces {
		for _, loop := range face.loops {
			for _, ce := range loop.coedges {
				if err := budget.step(); err != nil {
					return err
				}
				uses[ce.edge] = append(uses[ce.edge], compositeCoedgeUse{face: face, forward: ce.forward})
			}
		}
	}
	return auditVertexLinks(budget, uses)
}

// piScalar returns math.Pi as a boundedScalar. Go's math.Pi is the correctly
// rounded float64 nearest true pi, so its own representation error is at
// most one ulp — tiny, and nothing like conservativeValueError's structural
// "assume no cancellation at all" fallback, which is sized for a quantity
// this evaluator cannot otherwise bound and would overstate a well-known
// constant's own error by many orders of magnitude: wide enough, on a thin
// annulus, to starve stitchCurvedMass's own boundedQuotient calls of the
// clearance they need and refuse a perfectly sound thin fixture. Every
// multiply that follows charges its OWN rounding through boundedMul, so
// this is the one place pi's own representation error is spent.
func piScalar() boundedScalar {
	ulp := math.Nextafter(math.Pi, math.Inf(1)) - math.Pi
	return measuredScalar(math.Pi, ulp)
}

// boundedDot returns a·b as a boundedScalar, charging every multiply and add
// this dot product's own arithmetic commits (boundedMul/boundedAdd) —
// never the operands' own uncertainty, which every caller here supplies as
// zero: every vector boundedDot is asked about (a placed face's Frame
// origin/axis/normal, a placed Circle3's own center) is treated the same way
// the rest of this evaluator treats a placed coordinate outside a delta > 0
// placement term — authoritative for this file's own arithmetic, with the
// placement's own rounding charged separately, once, through
// sweptVolumeAllow/sweptMomentAllow.
func boundedDot(a, b r3.Vec) boundedScalar {
	x := boundedMul(measuredScalar(a.X, 0), measuredScalar(b.X, 0))
	y := boundedMul(measuredScalar(a.Y, 0), measuredScalar(b.Y, 0))
	z := boundedMul(measuredScalar(a.Z, 0), measuredScalar(b.Z, 0))
	return boundedAdd(boundedAdd(x, y), z)
}

// stitchFaceFluxAndMoment dispatches on the face's own tagged surface,
// returning its flux_F (this face's own contribution to 3·Volume, before
// the sum is divided by three) and its three first-moment contributions
// mx, my, mz. Every admitting arm requires a zero normalBound
// (docs/surface-design.md's own rule: a nonzero normalBound means the
// published tag only approximates the ruled surface a cap-blend band patch
// actually built, and integrating a closed form over the tag would be
// unsound for such a face). The sealed switch's default is Table R row R8,
// unchanged for every surface this increment does not land an arm for.
func stitchFaceFluxAndMoment(f *Face, anchor r3.Vec) (flux, mx, my, mz boundedScalar, err error) {
	if f.normalBound != 0 {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch closes a boundary holding a face whose published surface only approximates the geometry actually built (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	sign := 1.0
	if f.reversed {
		sign = -1.0
	}
	switch s := f.surface.(type) {
	case Plane:
		return planeFaceFluxAndMoment(f, s, anchor, sign)
	case Cylinder:
		return cylinderFaceFluxAndMoment(f, s, anchor, sign)
	default:
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch closes a boundary holding a %T face this evaluator has no closed-form flux integral for (docs/surface-design.md Table R row R8)`,
			ErrUnsupported, s,
		)
	}
}

// planeFaceFluxAndMoment is the Plane arm: K_F = 0, so flux_F = (A_F −
// anchor)·S_F with A_F = Frame.Origin() and S_F = sign·n·Area — a constant
// normal over a flat region has no other vector area, so this needs no
// contour sum at all. Every loop must be a single full Circle3 edge (this
// file's own scope restriction, top-of-file doc comment); anything else is
// ErrUnsupported. The first moment sums each loop's own disk (or, for a
// hole, negative-disk) contribution to the region's Iu, Iv, Iuu, Ivv, Iuv in
// the face's own local (u, v) frame, then folds those into the world-axis
// second moments through q_i = c_i + u·U_i + v·V_i (c_i = O_i − anchor_i):
//
//	∫_F q_i² dA = c_i²·Area + 2·c_i·U_i·Iu + 2·c_i·V_i·Iv
//	              + U_i²·Iuu + 2·U_i·V_i·Iuv + V_i²·Ivv
//
// and mx/my/mz = (sign·n_i/2)·that, for i = x, y, z.
func planeFaceFluxAndMoment(f *Face, pl Plane, anchor r3.Vec, sign float64) (flux, mx, my, mz boundedScalar, err error) {
	origin := pl.Frame.Origin()
	u, v, n := pl.Frame.U(), pl.Frame.V(), pl.Frame.N()

	area := boundedScalar{}
	iu, iv := boundedScalar{}, boundedScalar{}
	iuu, ivv, iuv := boundedScalar{}, boundedScalar{}, boundedScalar{}

	for _, l := range f.loops {
		if len(l.coedges) != 1 {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Plane loop bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				ErrUnsupported,
			)
		}
		c3, ok := l.coedges[0].edge.curve.(Circle3)
		if !ok {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Plane loop bounded by a full circle, not %T (docs/surface-design.md Table R row R8)`,
				ErrUnsupported, l.coedges[0].edge.curve,
			)
		}
		r := c3.Radius.Base()
		if !(r > 0) {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a positive circle radius (docs/surface-design.md Table R row R8)`,
				ErrUnsupported,
			)
		}
		loopSign := 1.0
		if !l.outer {
			loopSign = -1.0
		}

		local := pl.Frame.ToLocal(c3.Center)
		projScale := absSumUpper(vecMaxAbs(c3.Center), vecMaxAbs(origin))
		projBound := analyticRoundBound(projScale)
		cu := measuredScalar(local.X, projBound)
		cv := measuredScalar(local.Y, projBound)

		rr := boundedMul(measuredScalar(r, 0), measuredScalar(r, 0))
		diskArea := boundedMul(piScalar(), rr)
		if loopSign < 0 {
			diskArea = boundedNeg(diskArea)
		}
		fourth := boundedMul(measuredScalar(0.25, 0), boundedMul(piScalar(), boundedMul(rr, rr)))
		if loopSign < 0 {
			fourth = boundedNeg(fourth)
		}

		area = boundedAdd(area, diskArea)
		iu = boundedAdd(iu, boundedMul(cu, diskArea))
		iv = boundedAdd(iv, boundedMul(cv, diskArea))
		iuu = boundedAdd(iuu, boundedAdd(boundedMul(boundedMul(cu, cu), diskArea), fourth))
		ivv = boundedAdd(ivv, boundedAdd(boundedMul(boundedMul(cv, cv), diskArea), fourth))
		iuv = boundedAdd(iuv, boundedMul(boundedMul(cu, cv), diskArea))
	}

	cx := boundedSub(measuredScalar(origin.X, 0), measuredScalar(anchor.X, 0))
	cy := boundedSub(measuredScalar(origin.Y, 0), measuredScalar(anchor.Y, 0))
	cz := boundedSub(measuredScalar(origin.Z, 0), measuredScalar(anchor.Z, 0))

	dot := boundedAdd(boundedAdd(
		boundedMul(cx, measuredScalar(n.X, 0)),
		boundedMul(cy, measuredScalar(n.Y, 0))),
		boundedMul(cz, measuredScalar(n.Z, 0)),
	)
	flux = boundedMul(measuredScalar(sign, 0), boundedMul(area, dot))

	half := measuredScalar(0.5, 0)
	moment := func(ci boundedScalar, ni, ui, vi float64) boundedScalar {
		uiB, viB := measuredScalar(ui, 0), measuredScalar(vi, 0)
		term := boundedMul(ci, ci)
		term = boundedMul(term, area)
		term = boundedAdd(term, boundedMul(boundedMul(measuredScalar(2, 0), boundedMul(ci, uiB)), iu))
		term = boundedAdd(term, boundedMul(boundedMul(measuredScalar(2, 0), boundedMul(ci, viB)), iv))
		term = boundedAdd(term, boundedMul(boundedMul(uiB, uiB), iuu))
		term = boundedAdd(term, boundedMul(boundedMul(measuredScalar(2, 0), boundedMul(uiB, viB)), iuv))
		term = boundedAdd(term, boundedMul(boundedMul(viB, viB), ivv))
		term = boundedMul(measuredScalar(sign, 0), term)
		term = boundedMul(half, term)
		return boundedMul(measuredScalar(ni, 0), term)
	}
	mx = moment(cx, n.X, u.X, v.X)
	my = moment(cy, n.Y, u.Y, v.Y)
	mz = moment(cz, n.Z, u.Z, v.Z)
	return flux, mx, my, mz, nil
}

// cylinderFaceFluxAndMoment is the Cylinder arm, scoped to a full-circumference
// tube segment: exactly two loops, each a single full Circle3 edge, at two
// distinct axial positions. K_F = σ·Radius·f.area reuses the face's own
// already-proven area/areaBound (docs/surface-design.md's own reuse rule);
// the vector area S_F is exactly the ZERO vector for a full-circumference
// wall, proved below, so the (A_F − anchor)·S_F cross term vanishes and
// flux_F is exactly K_F.
//
// S_F = 0, proof. Parametrize the wall p(θ, z) = Origin + z·Axis +
// R·(cosθ·e1 + sinθ·e2), θ ∈ [0, 2π), e1/e2 an orthonormal basis of the
// plane ⊥ Axis. n(θ) = cosθ·e1 + sinθ·e2 does not depend on z, and
// ∫₀^2π cosθ dθ = ∫₀^2π sinθ dθ = 0, so S_F = ∫∫ n(θ)·R dθ dz = 0.
//
// THE FIRST MOMENT, derived and checked two ways (a z-axis-aligned cylinder
// centered on the anchor, where every component but the axial one must
// vanish by the x → −x / y → −y symmetry; and an anchor offset along a
// non-axis direction, checked against a direct single-variable integral of
// q_x² n_x over θ). For component i ∈ {x, y, z}: q_i = A_i + z·B_i + R·cosθ·E_i
// + R·sinθ·F_i (A_i = (Origin−anchor)_i, B_i = Axis_i, E_i = e1_i, F_i =
// e2_i), n_i = cosθ·E_i + sinθ·F_i. Expanding q_i²·n_i and integrating over a
// FULL θ period kills every term whose cos/sin exponents are not BOTH even
// (the standard "either exponent odd integrates to zero over a period"
// fact) — every term of q_i²·n_i survives that test in at most cos² or sin²,
// reducing ∫₀^2π q_i²·n_i dθ = 2π·(A_i + B_i·z)·R·(E_i² + F_i²). Since
// {e1, e2, Axis} is an orthonormal BASIS of ℝ³, the world unit vector ê_i
// decomposes as E_i·e1 + F_i·e2 + B_i·Axis, and |ê_i|² = 1 gives
// E_i² + F_i² = 1 − B_i². Integrating that linear-in-z result over
// z ∈ [zLo, zHi] and folding in the (R/2) from dA = R dθ dz and N_i = ∫(q_i²/2)n_i dA
// gives the closed form cylinderMomentTerm implements:
//
//	N_i = σ·π·R²·Δz·(1 − Axis_i²)·(A_i + Axis_i·zMid)
//
// Δz = |zHi − zLo|, zMid = (zLo + zHi)/2, both measured as Axis·(rim center
// − Origin) — a quantity this formula is invariant to shifting Origin along
// the axis by, checked directly: shifting Origin by t·Axis moves A_i by
// +t·Axis_i and zMid by −t, and Axis_i·(A_i + Axis_i·zMid) is unchanged by
// that shift for any t, so which point of the axis Origin happens to be
// never matters — the same invariance the plane's own A_F choice needs.
func cylinderFaceFluxAndMoment(f *Face, cyl Cylinder, anchor r3.Vec, sign float64) (flux, mx, my, mz boundedScalar, err error) {
	if len(f.loops) != 2 {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cylinder face bounded by exactly two full circles (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	var centers [2]r3.Vec
	for i, l := range f.loops {
		if len(l.coedges) != 1 {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cylinder rim bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				ErrUnsupported,
			)
		}
		c3, ok := l.coedges[0].edge.curve.(Circle3)
		if !ok {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cylinder rim bounded by a full circle, not %T (docs/surface-design.md Table R row R8)`,
				ErrUnsupported, l.coedges[0].edge.curve,
			)
		}
		centers[i] = c3.Center
	}

	radius := cyl.Radius.Base()
	if !(radius > 0) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a positive cylinder radius (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}

	z0 := boundedDot(cyl.Axis, centers[0].Sub(cyl.Origin))
	z1 := boundedDot(cyl.Axis, centers[1].Sub(cyl.Origin))
	dz := boundedAbs(boundedSub(z1, z0))
	if !(dz.value > 0) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cylinder face whose two rims sit at different axial positions (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	zMid := boundedMul(measuredScalar(0.5, 0), boundedAdd(z0, z1))

	flux = boundedMul(measuredScalar(sign, 0), boundedMul(measuredScalar(radius, 0), measuredScalar(f.area, f.areaBound)))

	piR2 := boundedMul(piScalar(), boundedMul(measuredScalar(radius, 0), measuredScalar(radius, 0)))
	piR2Dz := boundedMul(piR2, dz)

	moment := func(oi, ai, axisI float64) boundedScalar {
		aTerm := boundedSub(measuredScalar(oi, 0), measuredScalar(ai, 0))
		axisZMid := boundedMul(measuredScalar(axisI, 0), zMid)
		sumTerm := boundedAdd(aTerm, axisZMid)
		oneMinusAxis2 := boundedSub(measuredScalar(1, 0), boundedMul(measuredScalar(axisI, 0), measuredScalar(axisI, 0)))
		term := boundedMul(piR2Dz, boundedMul(oneMinusAxis2, sumTerm))
		return boundedMul(measuredScalar(sign, 0), term)
	}
	mx = moment(cyl.Origin.X, anchor.X, cyl.Axis.X)
	my = moment(cyl.Origin.Y, anchor.Y, cyl.Axis.Y)
	mz = moment(cyl.Origin.Z, anchor.Z, cyl.Axis.Z)
	return flux, mx, my, mz, nil
}

// stitchCurvedMass sums every face's own flux and first moment
// (stitchFaceFluxAndMoment), decides the global orientation sign from its
// own total — reversing every face and recomputing once, the curved
// analogue of stitch.go's acc.vol6.Sign() < 0 step — divides by three, and
// charges the placement allowance (sweptVolumeAllow/sweptMomentAllow) on
// the same terms capBandVolume already does for a placed curved solid: a
// placed curved stitched solid is Approximate on both readings, per
// docs/surface-design.md §6.4's placed-body paragraph.
func stitchCurvedMass(ctx context.Context, faces []*Face, anchor r3.Vec, delta float64) (Measurement, VecMeasurement, error) {
	if err := ctx.Err(); err != nil {
		return Measurement{}, VecMeasurement{}, err
	}
	fluxSum, momX, momY, momZ, err := sumStitchFlux(faces, anchor)
	if err != nil {
		return Measurement{}, VecMeasurement{}, err
	}
	if fluxSum.value < 0 {
		for _, f := range faces {
			reverseFaceOrientation(f)
		}
		fluxSum, momX, momY, momZ, err = sumStitchFlux(faces, anchor)
		if err != nil {
			return Measurement{}, VecMeasurement{}, err
		}
	}

	vol := boundedQuotient(fluxSum.value, fluxSum.bound, 3, 0)

	areaUpper := 0.0
	coordUpper := 0.0
	for _, f := range faces {
		areaUpper = absSumUpper(areaUpper, f.area, f.areaBound)
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				coordUpper = math.Max(coordUpper, ce.Start().Position().Value.Sub(anchor).Len())
			}
		}
		if cyl, ok := f.surface.(Cylinder); ok {
			// A rim vertex's own distance from anchor is what the loop above
			// already folds in; this adds the cylinder's own radius as a
			// blanket safety margin so coordUpper never understates a wall
			// point that sits farther from anchor than either rim vertex
			// does.
			coordUpper = absSumUpper(coordUpper, cyl.Radius.Base())
		}
	}

	if delta > 0 {
		// The placement allowance is charged once here, on momX/momY/momZ and
		// vol directly, and every downstream reading (the boundedQuotient
		// calls below) composes it through the ordinary quotient-bound
		// formula rather than through a second, separately-derived term —
		// charging it twice would only widen an already-sound bound, but it
		// would also hide a real regression: a shown-to-fail test that
		// deletes this leg must see the SAME published bound go slack, not a
		// smaller one still covered by a leftover duplicate charge.
		epsV := sweptVolumeAllow(delta, areaUpper)
		epsM := sweptMomentAllow(delta, areaUpper, coordUpper+delta)
		vol.bound = absSumUpper(vol.bound, epsV)
		momX.bound = absSumUpper(momX.bound, epsM)
		momY.bound = absSumUpper(momY.bound, epsM)
		momZ.bound = absSumUpper(momZ.bound, epsM)
	}

	volMeasurement := Measurement{
		Value:     units.CubicMillimeters(vol.value),
		Exactness: exactnessOf(vol.bound),
		Bound:     units.CubicMillimeters(vol.bound),
	}
	if vol.value == 0 {
		return Measurement{}, VecMeasurement{}, fmt.Errorf(`%w: a stitched curved solid with zero net volume has no centroid`, ErrDegenerate)
	}

	cx := boundedQuotient(momX.value, momX.bound, vol.value, vol.bound)
	cy := boundedQuotient(momY.value, momY.bound, vol.value, vol.bound)
	cz := boundedQuotient(momZ.value, momZ.bound, vol.value, vol.bound)
	if isNonFinite(cx.bound) || isNonFinite(cy.bound) || isNonFinite(cz.bound) {
		return Measurement{}, VecMeasurement{}, fmt.Errorf(`%w: the placement's proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, ErrUnsupported)
	}
	fx, fy, fz := anchor.X+cx.value, anchor.Y+cy.value, anchor.Z+cz.value

	bound := radius3D(math.Max(cx.bound, math.Max(cy.bound, cz.bound)))
	centroidMeasurement := VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}
	return volMeasurement, centroidMeasurement, nil
}

// sumStitchFlux sums every face's own flux and first moment, refusing R8 as
// soon as any one face's own dispatch does.
func sumStitchFlux(faces []*Face, anchor r3.Vec) (flux, momX, momY, momZ boundedScalar, err error) {
	for _, f := range faces {
		fFlux, fmx, fmy, fmz, err := stitchFaceFluxAndMoment(f, anchor)
		if err != nil {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
		}
		flux = boundedAdd(flux, fFlux)
		momX = boundedAdd(momX, fmx)
		momY = boundedAdd(momY, fmy)
		momZ = boundedAdd(momZ, fmz)
	}
	return flux, momX, momY, momZ, nil
}
