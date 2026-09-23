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
// boundary holding a curved face, for the Plane, Cylinder, Cone, Sphere and
// Torus variants. Table R row R8 is a per-face dispatch: a face this
// evaluator has no closed-form flux integral for refuses R8, and a Plane,
// Cylinder, Cone, Sphere or Torus face with a zero normalBound admits.
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
// integrating anything fresh. Radius itself is never read bare off
// Cylinder.Radius/Circle3.Radius — boundedCircleRadius derives it, and its
// own doc comment states why: neither type carries a bound field, and this
// evaluator's own revolve build can hand back a Radius that is a rounded,
// axis-dependent re-expression of a recorded point, not always the exact
// value the bare units.Value suggests.
//
// THE SCOPE RESTRICTION THIS INCREMENT ADDS, not in the original design
// sketch. A general trimmed Plane, Cylinder or Cone face needs S_F from the
// boundary's own ½∮p×dr contour sum, which has an arm for a straight (Line3)
// edge and one for a circular (Arc3/Circle3) edge alike. Every fixture this
// increment actually proves — an annular revolve sheet, a solid-of-revolution
// cylinder or cone built the same way — bounds every Plane face with FULL
// circles only (never a partial arc), every Cylinder face with exactly two
// full circular rims at two axial levels, and every Cone face with exactly
// two full circular rims at two distinct positions along its own growth axis
// (never a partial revolution, never a generatrix edge, for any of the
// three). These three admitted arms are scoped to exactly that shape and
// refuse (ErrUnsupported, R8) anything wider: a Plane loop that is not a
// single Circle3 edge, or a Cylinder/Cone face that is not exactly two
// single-Circle3 loops. The Sphere arm's own reachable fixture is narrower
// still — a complete closed spherical shell with NO boundary loop at all
// (a diameter-revolved semicircle's own two junctions are poles, and a pole
// junction mints no latitude circle, sphereFaceFluxAndMoment's own doc
// comment) — so it refuses any Sphere face carrying a boundary loop, rather
// than guess at a spherical zone or cap this file has no fixture for. This
// is deliberate, not an oversight: an untested closed form is not a proof,
// and CLAUDE.md's own rule is that a narrower answer always beats a wider
// guess. Widening any arm to a general Line3/Arc3 boundary is a later
// increment's own work, once a fixture exists to prove it against.
//
// One consequence of the restriction: because every admitted surface is a
// FULL circle, a full-circumference cylinder or a full-circumference cone
// frustum, each face's own vector area S_F collapses to a form simpler than
// the general contour sum would give — Plane's is exactly n·Area (a constant
// normal over a flat region has no other vector area to compute), a
// full-circumference Cylinder's is exactly the zero vector (the radial
// normal's own trig integrates to zero over a full turn — proven in
// cylinderFaceFluxAndMoment's own doc comment), and a full-circumference
// Cone's reduces to the closed-form cone-shadow identity
// π(R_lo²−R_hi²)·Axis (coneFaceFluxAndMoment's own doc comment) — the SAME
// circular term the general contour sum's own per-loop formula would give a
// full circle (½[C×(v1−v0)+R·L·k], with v1=v0 for a closed loop), just
// summed over the cone's own two rims rather than assembled through a
// generic per-edge dispatch. So this file never builds the general ½∮p×dr
// contour-sum helper the wider design sketch anticipated, WITH ITS OWN
// Line3 ARM, for any of the four arms actually landed — none of their
// fixtures ever presents a Line3 or partial-arc boundary edge, and adding
// unexercised dispatch code for one would be untested code with no fixture
// behind it. The Sphere arm's own S_F is zero for a plainer reason still: a
// zero-loop face has no boundary contour to sum at all, so its vector area
// is the empty sum rather than a trig integral that happens to collapse
// (sphereFaceFluxAndMoment's own doc comment). The Torus arm needs no
// general contour-sum form either, for a different reason again: its own
// scope restriction admits only the two-rim shape whose rims share the
// SAME radius (Major, torusFaceFluxAndMoment's own doc comment), so its S_F
// cancels the identical way a full-circumference Cylinder's does — reused,
// never re-derived, since it is the same fact under a different name.
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
// cylinderFaceFluxAndMoment's doc comment; the Cone arm's is derived and
// verified in coneFaceFluxAndMoment's own doc comment. The Sphere arm takes
// a shortcut the other three cannot: because a zero-loop Sphere face IS the
// whole closed boundary of the ball it bounds, on its own, its first moment
// is the ball's own moment, and the general shift-of-origin identity
// (volume times the offset from anchor to centroid) gives it directly —
// sphereFaceFluxAndMoment's own doc comment derives it and checks it
// against the same divergence-theorem surface integral by hand. The Torus
// arm cannot take that shortcut — its own scope restriction always admits a
// TWO-loop face sharing its boundary with a sibling Cylinder face, never a
// zero-loop complete torus — so its own moment is the general per-face
// divergence-theorem integral, derived and verified in
// torusFaceFluxAndMoment's own doc comment.
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
// The radius itself carries a second, independent source of bound
// (boundedCircleRadius's own doc comment): a revolve about an axis that is
// not coordinate-aligned through the origin hands back a Radius that is
// itself a rounded re-expression of a recorded point, and every use of it
// here goes through boundedMul against that proven bound rather than
// assuming the bare units.Value is exact. Every PUBLIC fixture this PR
// tests (stitch_flux_test.go) revolves about a coordinate-aligned axis
// through the origin, where that bound is proven exactly zero, so the
// nonzero case is pinned directly instead, on a hand-built face carrying a
// synthetic rim lengthBound far above ulp noise
// (TestStitchCylinderMomentChargesRadiusBound,
// TestStitchPlaneMomentChargesRadiusBound, stitch_internal_test.go) — the
// same treatment TestStitchCylinderFluxReusesAreaBound already gives the
// area-bound composition, for the identical reason: a fixture where the
// true bound happens to be zero cannot show this leg failing if it were
// ever deleted.
//
// The Sphere arm's own radius carries a DIFFERENT source of bound, because
// it has no rim edge to read boundedCircleRadius's own circumference from
// at all (this arm's own scope restriction admits only a zero-loop face):
// boundedSphereRadius inverts the face's own already-proven area/areaBound
// instead (its own doc comment). The public T50 fixture's own area bound is
// the same tiny, ulp-scale magnitude every other term here carries, so
// TestStitchSphereFluxReusesAreaBound (stitch_internal_test.go) pins the
// leg on a hand-built face carrying a synthetic areaBound far above that
// noise floor, the same treatment TestStitchCylinderFluxReusesAreaBound
// gives the Cylinder arm's own area-bound reuse.
//
// The Cone arm's apex is Origin when Radius is exactly 0 — the ONLY case
// this evaluator admits. The general formula, Origin −
// Axis·(Radius/tan(HalfAngle)), needs tan(HalfAngle)'s own rounding charged
// before that division could be trusted, and this evaluator has no sound
// way to do that: Go gives Sin/Cos/Atan2/Hypot (and so Tan) no public ulp
// contract, and composing tan from boundedSin/boundedCos and running it
// through boundedQuotient — the natural-looking fix — fails
// boundedQuotient's own clearance check unconditionally, for every angle,
// not only the degenerate ones (coneApex's own doc comment walks through
// why). So a nonzero Radius refuses (ErrUnsupported, R8) rather than
// publish an apex this evaluator cannot bound — pinned directly on a
// hand-built face (TestStitchConeApexRefusesNonzeroRadius,
// stitch_internal_test.go), since every reachable construction site sets
// Cone.Radius literally to 0 (Origin already IS the apex) and so never
// reaches this gate. coneApex separately refuses a degenerate HalfAngle
// (0, or non-finite one layer up through units.Value.In's own
// ErrNotFinite) regardless of Radius, pinned the same way
// (TestStitchConeHalfAngleRefusesDegenerateTangent,
// stitch_internal_test.go) — no real wallCone ever carries one
// (revolve_axis.go's own analytic-walk requirement keeps a real cone's
// HalfAngle finite and strictly between 0 and π/2).
//
// The Torus arm's Major and Minor carry a THIRD, still different treatment,
// because neither of the other two routes exists for them: unlike Cylinder/
// Cone (a rim's own circumference) or Sphere (the face's own area), a
// torus's two rims and its own proven area do not determine Major and Minor
// independently even together (torusFaceFluxAndMoment's own doc comment
// proves the ambiguity by a hand-checked counter-example). So this arm
// reads Torus.Major/Torus.Minor from the tag directly, gated on
// torusAxisIsCoordinateAligned rather than derived — the one condition
// under which that read carries the identical zero bound the OTHER arms'
// own edge/area derivations already give for a coordinate-aligned axis.
// Every public fixture this PR tests revolves about a coordinate-aligned
// axis through the origin, so the gate's own refusal for an off-axis Torus
// is pinned on a hand-built face (TestStitchTorusRefusesOffAxisAlignedAxis,
// stitch_internal_test.go), the same treatment the other arms' own
// hand-built refusal fixtures already get.
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

// faceIsTetrahedronEligible checks the surface tag itself (Plane), never
// f.isPlanar(): isPlanar also admits a heldPlanar Faceted face — a
// boolean-built face that stands for flat source geometry but is not itself
// a Plane — and triangulateStitchFaces demands a Plane outright, erroring on
// anything else. Latent today because no Faceted face is ever a stitch
// operand, but the mesh restatement (tessellate_stitch.go) gives the
// mismatch a second caller, so the tighter check closes it here rather than
// leave two admission rules that can disagree.
func faceIsTetrahedronEligible(f *Face) bool {
	if _, ok := f.surface.(Plane); !ok {
		return false
	}
	if f.normalBound != 0 {
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

// boundedCircleRadius reads a full circle's own radius as a boundedScalar,
// derived from the edge's already-proven length/lengthBound (circumference
// = 2·pi·R) rather than from Circle3.Radius/Cylinder.Radius directly.
//
// Circle3 and Cylinder carry Radius as a bare units.Value with no bound
// field of its own, and this evaluator's own construction does not make
// that value exact in general: revolve_axis.go's axisFrame.walk re-expresses
// a recorded profile point in axis coordinates through toAxisRhoBound, which
// folds in the resolved axis's own anchor and direction bounds
// (aUBound/aVBound/dUBound/dVBound) — exactly zero only when the axis is
// coordinate-aligned and passes through the origin (every product is by 0 or
// 1 and nothing rounds), and genuinely positive for any other axis. A
// revolve wall's or junction circle's radius (j.rho in revolve_build.go) is
// exactly this re-expressed value, so a face this arm admits can carry a
// radius this evaluator has already proven imperfect — the zero-normalBound
// gate says the face's geometry IS its tag (Plane/Cylinder, not some
// unpublished ruled approximation), never that every field of that tag is
// itself exact. Deriving the bound from the edge's own length sidesteps the
// need to re-derive axisFrame's own bound composition here: Edge.Length()'s
// public contract already states a sound enclosure of the true circumference
// for ANY Circle3 edge, from whichever builder made it, and R = length/2·pi
// inverts that same closed form.
func boundedCircleRadius(e *Edge) (boundedScalar, error) {
	if e.lengthUnbounded {
		return boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a circle whose circumference this evaluator can bound (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	twoPi := boundedMul(measuredScalar(2, 0), piScalar())
	return boundedQuotient(e.length, e.lengthBound, twoPi.value, twoPi.bound), nil
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
	case Cone:
		return coneFaceFluxAndMoment(f, s, anchor, sign)
	case Sphere:
		return sphereFaceFluxAndMoment(f, s, anchor, sign)
	case Torus:
		return torusFaceFluxAndMoment(f, s, anchor, sign)
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
		rB, err := boundedCircleRadius(l.coedges[0].edge)
		if err != nil {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
		}
		if !(rB.value > 0) {
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

		rr := boundedMul(rB, rB)
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
	var rimEdges [2]*Edge
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
		rimEdges[i] = l.coedges[0].edge
	}

	// rB is derived from the FIRST rim's own proven circumference
	// (boundedCircleRadius's own doc comment), never read off Cylinder.Radius
	// directly: both rims denote the same cylinder by construction, so
	// either rim's own proof covers the whole face.
	rB, err := boundedCircleRadius(rimEdges[0])
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}
	if !(rB.value > 0) {
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

	flux = boundedMul(measuredScalar(sign, 0), boundedMul(rB, measuredScalar(f.area, f.areaBound)))

	piR2 := boundedMul(piScalar(), boundedMul(rB, rB))
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

// coneApex derives a Cone's apex as three boundedScalars, one per world
// coordinate. Radius and HalfAngle are read exactly as topology.go's own
// Cone doc states them: Radius is the cone's radius AT Origin (zero when
// Origin is already the apex), and the wall grows along Axis by HalfAngle
// from there. The general formula is Origin − Axis·(Radius/tan(HalfAngle)),
// but this function NEVER evaluates that division: it admits Radius == 0
// only, where the formula collapses to apex = Origin, Exact, independent of
// tan(HalfAngle)'s own value. Every construction site in this evaluator
// (revolve_build.go's wallCone, capblend_geom.go's coneSurface) sets Radius
// literally to 0 — Origin already IS the apex — so this restriction costs
// no reachable admitted face; the refusal it adds for a nonzero Radius is
// exercised at a hand-built internal fixture
// (TestStitchConeApexRefusesNonzeroRadius, stitch_internal_test.go), since
// no construction site here ever sets Radius otherwise.
//
// WHY THE GENERAL DIVISION IS NOT SOUNDLY BOUNDABLE HERE, so a future
// change does not silently reintroduce it. tan(HalfAngle) would need its
// own rounding charged before Radius/tan(HalfAngle) could be trusted, and
// this codebase has no tool for that: analyticRoundBound's own doc comment
// states "Go deliberately gives Sin, Cos, Atan2 and Hypot no public ulp
// contract, so a result computed through them never trusts this helper's
// roundoff budget on its own" — the same posture capblend_moments.go and
// survey2d.go state independently, and boundedSqrt honors even for
// math.Sqrt, which IEEE 754 DOES guarantee correctly rounded. The
// natural-looking fix — compose tan from boundedSin/boundedCos and run it
// through boundedQuotient — was tried and fails outright:
// conservativeValueError's own structural bound on a Sin/Cos result
// (|value|+1) is wider than either function's own value, so it fails
// boundedQuotient's clearance check for EVERY angle, not only the
// degenerate ones, refusing the whole arm rather than only the cases a
// tangent gate should catch. Charging only "a few ulps" of math.Tan's own
// result instead would be a NEW error model this codebase does not have
// and, per the citations above, would contradict its own repeated stance —
// not a reuse of an existing one. Refusing the one construction that would
// need it is the reject-only alternative CLAUDE.md's own rule favors over
// carrying an unproven bound.
//
// The tan(HalfAngle) call below still runs, but only to validate the angle
// is non-degenerate, never to compute anything: a HalfAngle of 0 gives an
// exactly-zero tangent, a degenerate needle no reachable wallCone ever
// carries (revolve_axis.go's own analytic-walk requirement keeps HalfAngle
// finite and strictly between 0 and π/2 — docs/surface-design.md's own
// record of that requirement). The `isNonFinite(tanValue)` half of this
// check is a defensive backstop, not independently exercised: a NaN or Inf
// HalfAngle is refused one layer up, by units.Value.In's own ErrNotFinite,
// before this function's own tan(HalfAngle) call ever runs, and a
// HalfAngle near π/2 does not make math.Tan return an actual ±Inf for any
// finite input (π/2 itself has no exact float64 representation) —
// TestStitchConeHalfAngleRefusesDegenerateTangent
// (stitch_internal_test.go) shows the exactly-zero case failing red and
// records why the non-finite half of the guard is not similarly shown.
func coneApex(c Cone) (apexX, apexY, apexZ boundedScalar, err error) {
	half, herr := c.HalfAngle.In(units.Radian)
	if herr != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(`decad: a cone's half angle is not an angle: %w`, herr)
	}
	// tan(HalfAngle) is read through a single math.Tan call purely to
	// validate the angle is non-degenerate — a HalfAngle of 0 gives an
	// exactly-zero tangent, no real wallCone geometry at all. Its VALUE is
	// never used to compute anything: this file has no sound way to bound
	// tan(HalfAngle)'s own rounding (this function's own doc comment states
	// why), so the apex formula below never divides by it. That is also why
	// this check alone cannot admit a nonzero Radius — see the gate below.
	tanValue := math.Tan(half)
	if !(tanValue > 0) || isNonFinite(tanValue) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone whose half-angle has a positive, finite tangent (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	radiusValue, rerr := c.Radius.In(units.Millimeter)
	if rerr != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(`decad: a cone's radius is not a length: %w`, rerr)
	}
	if radiusValue != 0 {
		// The general apex formula is Origin − Axis·(Radius/tan(HalfAngle)),
		// and CHARGING that division soundly needs a proven bound on
		// tan(HalfAngle)'s own rounding — this function's own doc comment
		// explains why no such bound exists in this codebase: Go gives
		// Sin/Cos/Atan2/Hypot (and so Tan, composed from them) no public
		// ulp contract, and every other place this evaluator reads one of
		// those functions falls back to conservativeValueError's
		// deliberately wide structural bound rather than assume tighter
		// accuracy — which, as boundedCircleRadius's own history in this
		// file already showed, fails boundedQuotient's clearance check
		// outright rather than merely widen the result. Composing tan from
		// boundedSin/boundedCos and running it through boundedQuotient hits
		// exactly that failure. So a nonzero Radius refuses rather than
		// publish an apex whose own bound this evaluator cannot prove:
		// Radius is EXACTLY zero at every construction site in this
		// evaluator (revolve_build.go's wallCone, capblend_geom.go's
		// coneSurface — Origin is already the apex), so refusing what
		// nothing builds costs no reachable fixture
		// (TestStitchConeApexRefusesNonzeroRadius, stitch_internal_test.go).
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone whose Origin is already its own apex (Radius exactly 0); this evaluator has no sound bound for tan(HalfAngle)'s own rounding, which a nonzero Radius would need to divide by (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	// Radius is EXACTLY zero (checked above), so the general formula's
	// division by tan(HalfAngle) is skipped entirely rather than computed
	// and discarded: Origin is already the apex, Exact, with no dependency
	// on tan(HalfAngle)'s own accuracy at all.
	apexX = measuredScalar(c.Origin.X, 0)
	apexY = measuredScalar(c.Origin.Y, 0)
	apexZ = measuredScalar(c.Origin.Z, 0)
	return apexX, apexY, apexZ, nil
}

// axisDistanceFromApex returns Axis·(point − apex) as a boundedScalar,
// where apex carries its own per-coordinate bound (coneApex's own doc
// comment) and point/Axis are treated as exact placed coordinates, the same
// convention boundedDot's own doc comment states for every other vector
// this file asks about.
func axisDistanceFromApex(apexX, apexY, apexZ boundedScalar, axis, point r3.Vec) boundedScalar {
	dx := boundedSub(measuredScalar(point.X, 0), apexX)
	dy := boundedSub(measuredScalar(point.Y, 0), apexY)
	dz := boundedSub(measuredScalar(point.Z, 0), apexZ)
	return boundedAdd(boundedAdd(
		boundedMul(measuredScalar(axis.X, 0), dx),
		boundedMul(measuredScalar(axis.Y, 0), dy)),
		boundedMul(measuredScalar(axis.Z, 0), dz))
}

// coneFaceFluxAndMoment is the Cone arm, scoped identically in shape to
// cylinderFaceFluxAndMoment: exactly two loops, each a single full Circle3
// edge, at two distinct positions along the cone's own growth Axis. K_F = 0
// — the vector from the apex to any surface point runs along a ruling, and
// NormalAt's own Cone formula (n = cosβ·radial − sinβ·Axis, topology.go)
// makes that ruling's own direction (cosβ·Axis-perpendicular component +
// sinβ·Axis, the same angle β off the radial as n is off Axis but
// complementary) orthogonal to n by construction — a ruling and the
// surface normal at any of its points are always perpendicular, the
// defining property of a ruled surface's normal. So flux_F reduces to the
// pure cross term (apex − anchor)·S_F.
//
// S_F, proof. Parametrize the wall p(θ, z) = apex + z·Axis +
// z·tanβ·(cosθ·e1 + sinθ·e2), z the distance from the apex along Axis,
// θ ∈ [0, 2π), {e1, e2, Axis} an orthonormal basis. The intrinsic normal is
// n(θ) = cosβ·(cosθ·e1 + sinθ·e2) − sinβ·Axis (NormalAt's own formula,
// θ-independent apart from the radial term), and dA = z·tanβ·secβ dθ dz.
// Over a FULL θ period ∫cosθ dθ = ∫sinθ dθ = 0, so only n's constant −sinβ·
// Axis term survives the θ integral, at weight ∫₀^2π dθ = 2π:
//
//	S_F = ∫_zLo^zHi (−2π·sinβ·Axis)·z·tanβ·secβ dz
//	    = −2π·tan²β·Axis · ∫_zLo^zHi z dz = −π·tan²β·(zHi²−zLo²)·Axis
//
// Since R = z·tanβ at any point on the wall, tan²β·z² = R², so
// tan²β·(zHi²−zLo²) = R_hi²−R_lo² (R_lo/R_hi the radii AT zLo/zHi), giving
// the closed form this function implements:
//
//	S_F = π·(R_lo² − R_hi²)·Axis
//
// — the standard cone-shadow identity, needing no trig at the call site:
// R_lo/R_hi come from boundedCircleRadius, never a fresh tan(β) evaluation.
// Verified independently against numeric double-quadrature over several
// random half-angles, anchors and apex placements before landing (this
// PR's own scratch verification; not carried into the tree).
//
// THE FIRST MOMENT, derived the same way (expand q_i²·n_i, integrate over a
// full θ period — every ODD power of cosθ/sinθ vanishes, so only the terms
// surviving in cos²θ+sin²θ = 1 remain — then integrate the z-weighted
// result over [zLo, zHi]). With A_i = apex_i − anchor_i, B_i = Axis_i,
// K = 1−B_i² (the {e1,e2} share of that world axis, from the orthonormal
// basis identity E_i²+F_i² = 1−B_i², cylinderFaceFluxAndMoment's own proof
// of the identical fact):
//
//	M_i = (π·tan²β/2) · [ −2·A_i²·B_i·I1 + 2·A_i·(1−3·B_i²)·I2
//	                      + (2·B_i·(1−2·B_i²) − B_i·K·tan²β)·I3 ]
//
// I1 = (zHi²−zLo²)/2, I2 = (zHi³−zLo³)/3, I3 = (zHi⁴−zLo⁴)/4. tan²β is read
// from the rims' OWN slope ((R_hi−R_lo)/(zHi−zLo))² rather than a second
// math.Tan evaluation of HalfAngle, so this arm calls coneApex's own
// math.Tan exactly once per face, for the apex alone. Verified
// independently against numeric double-quadrature the same way S_F was,
// across several random half-angles, anchors, axes and apex placements.
func coneFaceFluxAndMoment(f *Face, cone Cone, anchor r3.Vec, sign float64) (flux, mx, my, mz boundedScalar, err error) {
	if len(f.loops) != 2 {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face bounded by exactly two full circles (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	var centers [2]r3.Vec
	var rimEdges [2]*Edge
	for i, l := range f.loops {
		if len(l.coedges) != 1 {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cone rim bounded by a single full circle (docs/surface-design.md Table R row R8)`,
				ErrUnsupported,
			)
		}
		c3, ok := l.coedges[0].edge.curve.(Circle3)
		if !ok {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Cone rim bounded by a full circle, not %T (docs/surface-design.md Table R row R8)`,
				ErrUnsupported, l.coedges[0].edge.curve,
			)
		}
		centers[i] = c3.Center
		rimEdges[i] = l.coedges[0].edge
	}

	apexX, apexY, apexZ, err := coneApex(cone)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}

	var radii [2]boundedScalar
	for i := range rimEdges {
		rB, err := boundedCircleRadius(rimEdges[i])
		if err != nil {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
		}
		if !(rB.value > 0) {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a positive cone rim radius (docs/surface-design.md Table R row R8)`,
				ErrUnsupported,
			)
		}
		radii[i] = rB
	}

	var zs [2]boundedScalar
	for i := range centers {
		zs[i] = axisDistanceFromApex(apexX, apexY, apexZ, cone.Axis, centers[i])
	}
	loIdx, hiIdx := 0, 1
	if zs[0].value > zs[1].value {
		loIdx, hiIdx = 1, 0
	}
	zLo, zHi := zs[loIdx], zs[hiIdx]
	rLo, rHi := radii[loIdx], radii[hiIdx]
	if !(zLo.value > 0) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face whose rims sit beyond the apex along its own axis (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	dz := boundedSub(zHi, zLo)
	if !(dz.value > 0) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Cone face whose two rims sit at different positions along its own axis (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}

	// S_F = pi*(rLo^2 - rHi^2)*Axis; the cross term is (apex-anchor).S_F,
	// which since S_F is a scalar multiple of the unit Axis reduces to
	// sMag * Axis.(apex-anchor).
	sMag := boundedMul(piScalar(), boundedSub(boundedMul(rLo, rLo), boundedMul(rHi, rHi)))
	axisDotApexMinusAnchor := boundedNeg(axisDistanceFromApex(apexX, apexY, apexZ, cone.Axis, anchor))
	flux = boundedMul(measuredScalar(sign, 0), boundedMul(sMag, axisDotApexMinusAnchor))

	// tan^2(beta) from the rims' own slope, never a second HalfAngle trig
	// evaluation (this function's own doc comment).
	slopeNum := boundedSub(rHi, rLo)
	slope := boundedQuotient(slopeNum.value, slopeNum.bound, dz.value, dz.bound)
	tan2 := boundedMul(slope, slope)

	zHiSq, zLoSq := boundedMul(zHi, zHi), boundedMul(zLo, zLo)
	i1 := boundedMul(measuredScalar(0.5, 0), boundedSub(zHiSq, zLoSq))
	zHiCu := boundedMul(zHiSq, zHi)
	zLoCu := boundedMul(zLoSq, zLo)
	i2 := boundedQuotient(boundedSub(zHiCu, zLoCu).value, boundedSub(zHiCu, zLoCu).bound, 3, 0)
	zHi4 := boundedMul(zHiSq, zHiSq)
	zLo4 := boundedMul(zLoSq, zLoSq)
	i3 := boundedMul(measuredScalar(0.25, 0), boundedSub(zHi4, zLo4))

	one := measuredScalar(1, 0)
	two := measuredScalar(2, 0)
	three := measuredScalar(3, 0)
	half := measuredScalar(0.5, 0)
	piTan2 := boundedMul(piScalar(), tan2)

	moment := func(apexI boundedScalar, anchorI, axisI float64) boundedScalar {
		ai := boundedSub(apexI, measuredScalar(anchorI, 0))
		bi := measuredScalar(axisI, 0)
		biSq := boundedMul(bi, bi)

		term1 := boundedNeg(boundedMul(boundedMul(two, boundedMul(ai, ai)), boundedMul(bi, i1)))

		oneMinus3BiSq := boundedSub(one, boundedMul(three, biSq))
		term2 := boundedMul(boundedMul(two, boundedMul(ai, oneMinus3BiSq)), i2)

		oneMinus2BiSq := boundedSub(one, boundedMul(two, biSq))
		partA := boundedMul(two, boundedMul(bi, oneMinus2BiSq))
		oneMinusBiSq := boundedSub(one, biSq)
		partB := boundedMul(bi, boundedMul(oneMinusBiSq, tan2))
		coef3 := boundedSub(partA, partB)
		term3 := boundedMul(coef3, i3)

		sum := boundedAdd(boundedAdd(term1, term2), term3)
		m := boundedMul(piTan2, boundedMul(half, sum))
		return boundedMul(measuredScalar(sign, 0), m)
	}
	mx = moment(apexX, anchor.X, cone.Axis.X)
	my = moment(apexY, anchor.Y, cone.Axis.Y)
	mz = moment(apexZ, anchor.Z, cone.Axis.Z)
	return flux, mx, my, mz, nil
}

// boundedSphereRadius derives a closed, boundary-less Sphere face's own
// radius from its already-proven area/areaBound (Area = 4πR², inverted)
// rather than trusting Sphere.Radius directly. boundedCircleRadius (the
// Cylinder/Cone arms' own route) reads a RIM EDGE's own proven
// circumference, but sphereFaceFluxAndMoment's own scope restriction admits
// a Sphere face only when it carries NO boundary loop at all (this file's
// only reachable Sphere fixture — docs/surface-design.md's own T50 — mints
// no latitude circle for either of its two on-axis pole junctions,
// revolve_build.go's fullRevLoops), so there is no edge here to read a
// circumference from at all, not merely one this file declines to trust.
// The face's own area, by contrast, is ALREADY a proven reading regardless
// of loop count: it is set at construction time from the profile's own
// closed-form Pappus integral (revolve_build.go's faceArea, walkAxisMoment),
// the identical value docs/surface-design.md §6.4's Area-sum block already
// trusts unconditionally for every face kind. Inverting Area = 4πR² through
// boundedQuotient and boundedSqrt is therefore a reuse of an
// already-published, already-tested reading, never a fresh unproven one —
// boundedSqrt's own rational bracket (ratSqrtDown/ratSqrtUp) is what makes
// the inversion itself sound, since Go's math.Sqrt carries no accuracy
// contract this file would otherwise have to lean on either.
func boundedSphereRadius(f *Face) (boundedScalar, error) {
	fourPi := boundedMul(measuredScalar(4, 0), piScalar())
	rSq := boundedQuotient(f.area, f.areaBound, fourPi.value, fourPi.bound)
	rB := boundedSqrt(rSq)
	if !(rB.value > 0) {
		return boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Sphere face with a positive radius (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	return rB, nil
}

// sphereFaceFluxAndMoment is the Sphere arm, scoped to a face with NO
// boundary loop at all: a complete, closed spherical shell — the only shape
// this evaluator's reachable fixtures ever present (a semicircle revolved a
// full turn about its own diameter, docs/surface-design.md's own T50:
// fullRevLoops mints a latitude circle only for a junction OFF the revolve
// axis, and both of a diameter-revolved semicircle's own junctions are
// poles ON it, so neither junction contributes an edge and the face carries
// zero loops, zero edges, and — since Body.Vertices() derives from
// Body.Edges() — the WHOLE stitched body carries zero vertices too).
// Anything wider (a spherical zone or cap bounded by one or two rim
// circles) is refused rather than guessed at: this file never builds the
// general ½∮p×dr contour-sum machinery (top-of-file doc comment), and no
// reachable fixture today exercises that shape, so admitting it would be
// untested code with no fixture behind it.
//
// K_F. Anchored at the sphere's own Center, p−Center is parallel to n at
// every surface point (n = σ·(p−Center)/Radius, σ = ±1 from f.reversed), so
// (p−Center)·n = σ·Radius identically — the same identity the Cylinder arm
// uses, with the sphere's own Center standing in for a point on the
// cylinder's axis. So flux_F = K_F + (Center−anchor)·S_F, K_F =
// σ·Radius·f.area — reusing the face's own already-proven area/areaBound
// exactly as the Cylinder arm's own doc comment states the reuse — with
// Radius itself read through boundedSphereRadius, never bare off
// Sphere.Radius (that function's own doc comment).
//
// S_F = 0, EXACTLY, rather than by a trig integral collapsing to zero: a
// face with no boundary loop has no contour to sum ∮p×dr over at all, so
// its vector area is the empty sum. (This also follows from an unrelated,
// more general fact — ∫_F n dA over ANY closed orientable surface is the
// zero vector, by the divergence theorem applied to a constant field — but
// the loop-free construction here makes it true by construction, needing no
// such argument.) So the (Center−anchor)·S_F cross term vanishes for EVERY
// anchor, not merely the sphere's own center, and flux_F is exactly K_F
// regardless of which anchor stitchCurvedMass passes in — the property that
// makes this arm safe to reach with the r3.Vec{} anchor stitch.go
// substitutes when the shared vertex table is empty (stitch.go's own doc
// comment at evalStitchContext, written against exactly this fixture before
// this arm landed).
//
// THE FIRST MOMENT. Because the face IS a complete closed boundary all on
// its own — never one of several faces sharing a boundary with others, by
// this arm's own zero-loop scope — its own moment is the WHOLE ball's own
// first moment, not a partial surface-integral term that only sums to
// something meaningful alongside sibling faces the way Plane/Cylinder/
// Cone's per-face terms do. The general shift-of-origin identity for a
// region Ω of volume V centered at Ĉ: ∫_Ω(x_i−a_i)dV = V·(Ĉ_i−a_i), applies
// directly with Ω the ball, Ĉ = Center, and V this face's own signed
// sub-volume flux_F/3 (flux_F is exactly 3·(signed volume) by construction,
// the same normalization stitchCurvedMass's own vol := fluxSum/3 uses for
// the total). So M_i = (flux_F/3)·(Center_i−anchor_i), checked directly
// against the divergence-theorem surface integral
// ∫_S((p_i−a_i)²/2)n_i dA by hand for a sphere: expanding
// (p_i−a_i)² = r_i²+2r_id_i+d_i² (r = p−Center, d = Center−anchor) and
// integrating each term over the sphere (∫r_i dA = ∫r_i³ dA = 0 by odd
// symmetry, ∫r_i² dA = (4/3)πR⁴ from ∫r_i²dA = (1/3)∫|r|²dA = (1/3)R²·Area)
// gives M_i = σ/(2R)·[2d_i·(4/3)πR⁴] = σ·d_i·(4/3)πR³ — the identical
// closed form, since (4/3)πR³ = R·Area/3 = K_F/(3σ) and σ²=1.
func sphereFaceFluxAndMoment(f *Face, sph Sphere, anchor r3.Vec, sign float64) (flux, mx, my, mz boundedScalar, err error) {
	if len(f.loops) != 0 {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path needs a Sphere face with no boundary at all (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	rB, err := boundedSphereRadius(f)
	if err != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, err
	}

	flux = boundedMul(measuredScalar(sign, 0), boundedMul(rB, measuredScalar(f.area, f.areaBound)))

	third := boundedQuotient(flux.value, flux.bound, 3, 0)
	moment := func(centerI, anchorI float64) boundedScalar {
		d := boundedSub(measuredScalar(centerI, 0), measuredScalar(anchorI, 0))
		return boundedMul(third, d)
	}
	mx = moment(sph.Center.X, anchor.X)
	my = moment(sph.Center.Y, anchor.Y)
	mz = moment(sph.Center.Z, anchor.Z)
	return flux, mx, my, mz, nil
}

// torusAxisIsCoordinateAligned reports whether t's own axis line is exactly
// a signed coordinate axis through the world origin: Axis is bit-identical
// to one of the six signed unit basis vectors, and Center's own two
// components perpendicular to it are bit-identical to zero. This is the
// SAME condition boundedCircleRadius's own doc comment names as making a
// revolve's axis-dependent re-expression bound exactly zero ("every
// product is by 0 or 1 and nothing rounds") — the reason the Cylinder/Cone/
// Sphere arms still derive their own radius from a rim or from area,
// regardless of axis alignment, is that their derivation has to work
// uniformly whether the axis is aligned or not, and doing so gives a bound
// that is automatically zero in the aligned case without a separate gate.
//
// A Torus has no such uniform, axis-independent derivation for Major and
// Minor (torusFaceFluxAndMoment's own doc comment proves why: the two rims
// of a symmetric tube zone do not determine them, even together with the
// face's own proven area — the system is genuinely under-determined, not
// merely hard). So this file cannot avoid reading Torus.Major/Torus.Minor
// from the tag directly the way it avoids reading Cylinder.Radius/
// Sphere.Radius directly; instead it GATES on the one condition under which
// that read carries the identical zero bound the other arms' own
// derivations already prove for a coordinate-aligned axis, and refuses
// (ErrUnsupported, R8) every other axis placement rather than publish a
// Major or Minor this file cannot bound.
func torusAxisIsCoordinateAligned(t Torus) bool {
	switch t.Axis {
	case r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0):
		return t.Center.Y == 0 && t.Center.Z == 0
	case r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0):
		return t.Center.X == 0 && t.Center.Z == 0
	case r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1):
		return t.Center.X == 0 && t.Center.Y == 0
	default:
		return false
	}
}

// torusFaceFluxAndMoment is the Torus arm, scoped to the one shape this
// evaluator's own reachable fixture builds: a symmetric tube zone spanning
// EXACTLY the tube's own outer quarter-to-quarter window — φ ∈ [−π/2, π/2],
// measuring φ from the plane through Center perpendicular to the axis
// (φ = 0 there: the tube's own outermost point, ρ = Major+Minor, at
// Center's own axial position) — bounded by exactly two full Circle3
// rims, one Minor above Center's own axial position and one Minor below
// it. offAxisSemicircleSketch (docs/surface-design.md's own T53) is exactly
// this shape: a straight wall at ρ = Major (the chord, closing the tube's
// own equatorial diameter, revolved into the face's own Cylinder sibling)
// and a semicircular arc bulging OUTWARD from ρ = Major to ρ = Major+Minor
// and back (revolved into this Torus face).
//
// WHY THIS FILE CANNOT ADMIT A WIDER WINDOW, and why that is a genuine
// mathematical limit rather than a missing derivation. Parametrizing the
// torus surface by (θ, φ) — θ the revolve angle, φ the tube angle, with
// ρ(φ) = Major + Minor·cosφ and axial offset z(φ) = Minor·sinφ from
// Center's own axial position — the outward normal is
// n(φ) = cosφ·radial + sinφ·Axis, and (p−Center)·n = Major·cosφ + Minor
// pointwise (the identity this file's own top-of-file doc table states).
// Integrating that over a φ window [φlo, φhi] and a full θ turn gives
// K_F = 2π·Minor·∫ (Major·cosφ+Minor)(Major+Minor·cosφ) dφ, whose
// antiderivative carries a term LINEAR IN THE RAW ANGLE φ itself (from
// ∫cos²φ dφ = φ/2 + sin2φ/4), not reducible to sinφ/cosφ alone. Recovering
// φlo/φhi as raw angles from the two rims' own proven radius and axial
// position needs an inverse trig function of computed floating data — this
// evaluator has no sound bound for one (this file's own coneApex doc
// comment states why at length; the same limit applies here). Nor can
// Major/Minor/φlo/φhi be recovered algebraically without one: two rims of
// the SAME radius at axial offsets ±e from Center satisfy
// (ρ−Major)²+e² = Minor² for infinitely many (Major, Minor) pairs — e.g.
// Major=10, Minor=5 (φ=±90°, e=5) and Major=8, Minor=√29≈5.385 (φ≈±68.2°,
// e=5) both put a full circle of radius 10 at axial offset ±5 from the
// SAME Center — a genuine, checked-by-hand ambiguity, not a derivation this
// file merely has not found. So the ONLY φ window this file can integrate
// without an unbounded trig call is one whose endpoints are KNOWN constants
// rather than recovered ones — ±π/2 is the sole such window this
// evaluator's own construction ever reaches (a complete, zero-loop torus
// would be the OTHER such window, Δφ=2π, the same shape sphereFaceFluxAndMoment's
// own zero-loop scope takes for a sphere, but no reachable fixture in this
// tree ever builds one) — and detecting THAT specific window from held data
// is what the two gates below establish, structurally, never by measuring
// an angle.
//
// GATE 1 — torusAxisIsCoordinateAligned. Trusting Major and Minor bare off
// the tag at all needs the identical zero-bound condition boundedCircleRadius's
// own doc names for the OTHER radius fields' axis-dependent rounding.
//
// GATE 2 — the window check. Given Gate 1, Major and Minor carry a proven
// zero bound, so e_lo = Axis·(rimCenter_lo − Center) and e_hi similarly are
// EXACT (boundedDot's own zero-bound result for an axis-aligned Axis — every
// product is by 0 or 1). Checking {e_lo, e_hi} == {−Minor, +Minor} EXACTLY
// (bit-for-bit, never a tolerance) is what proves sinφ_lo = −1, sinφ_hi = +1
// EXACTLY — a pure algebraic consequence, not a measurement — and therefore
// cosφ_lo = cosφ_hi = 0 and φ_lo, φ_hi = ∓π/2 EXACTLY, since sin = ±1 pins φ
// uniquely on the tube's own principal branch. This is reject-only: a face
// that fails the check is refused, never admitted on a near-match — a small
// residual here would prove nothing (CLAUDE.md's own rule), so none is
// accepted; only exact equality of already-exact quantities is.
//
// THE CLOSED FORMS, verified against independent numeric double integration
// (θ, φ grid, ~6×10⁵ samples, this PR's own scratch verification — not
// carried into the tree) for both a coordinate-aligned and a deliberately
// oblique Torus (arbitrary Center, Axis and anchor), agreeing to 4–5
// significant digits at that grid's own resolution:
//
//	K_F     = 3·π²·Major·Minor² + 4·π·Minor·(Major² + Minor²)
//	M_i     = π·Minor·(Center_i − anchor_i)·
//	            (2·Major²·(1 − Axis_i²) + π·Major·Minor + (4/3)·Minor²)
//
// K_F needs no cross term: the two rims share the SAME radius (Major, by
// Gate 2), so — exactly as cylinderFaceFluxAndMoment's own full-circumference
// argument shows for two equal-radius circles — the face's own vector area
// S_F is the zero vector, and flux_F is exactly K_F for every anchor. M_i's
// own derivation integrates (p_i−anchor_i)²·n_i over a full θ period first
// (every odd power of cosθ/sinθ vanishing, the same fact
// cylinderFaceFluxAndMoment's and coneFaceFluxAndMoment's own moment proofs
// use), then over φ ∈ [−π/2, π/2] — where ∫cosφ dφ = 2, ∫sinφ dφ = 0,
// ∫cos²φ dφ = ∫sin²φ dφ = π/2, ∫cos³φ dφ = 4/3, ∫sin²φ·cosφ dφ = 2/3, and
// every ODD-in-φ integral (sinφ alone, sinφ·cos²φ, sin³φ, sinφ·cosφ) is
// zero over the symmetric interval — collapsing every term but two, whose
// (1 − Axis_i²)-weighted and constant parts combine to the M_i form above
// (this PR's own scratch derivation, hand-expanded and cross-checked
// against the numeric integral).
func torusFaceFluxAndMoment(f *Face, t Torus, anchor r3.Vec, sign float64) (flux, mx, my, mz boundedScalar, err error) {
	refuse := func(msg string) (boundedScalar, boundedScalar, boundedScalar, boundedScalar, error) {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
			`%w: %s (docs/surface-design.md Table R row R8)`, ErrUnsupported, msg,
		)
	}
	if len(f.loops) != 2 {
		return refuse(`Stitch's flux path needs a Torus face bounded by exactly two full circles`)
	}
	if !torusAxisIsCoordinateAligned(t) {
		return refuse(`Stitch's flux path needs a Torus revolved about a coordinate-aligned axis through the origin; this evaluator has no sound bound for Major/Minor's own rounding otherwise`)
	}
	majorValue, merr := t.Major.In(units.Millimeter)
	if merr != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(`decad: a torus's major radius is not a length: %w`, merr)
	}
	minorValue, nerr := t.Minor.In(units.Millimeter)
	if nerr != nil {
		return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(`decad: a torus's minor radius is not a length: %w`, nerr)
	}
	if !(majorValue > 0) || !(minorValue > 0) {
		return refuse(`Stitch's flux path needs a Torus with a positive Major and Minor radius`)
	}

	var centers [2]r3.Vec
	for i, l := range f.loops {
		if len(l.coedges) != 1 {
			return refuse(`Stitch's flux path needs a Torus rim bounded by a single full circle`)
		}
		c3, ok := l.coedges[0].edge.curve.(Circle3)
		if !ok {
			return boundedScalar{}, boundedScalar{}, boundedScalar{}, boundedScalar{}, fmt.Errorf(
				`%w: Stitch's flux path needs a Torus rim bounded by a full circle, not %T (docs/surface-design.md Table R row R8)`,
				ErrUnsupported, l.coedges[0].edge.curve,
			)
		}
		centers[i] = c3.Center
	}

	e0 := boundedDot(t.Axis, centers[0].Sub(t.Center))
	e1 := boundedDot(t.Axis, centers[1].Sub(t.Center))
	halfWindow := (e0.value == -minorValue && e1.value == minorValue) ||
		(e0.value == minorValue && e1.value == -minorValue)
	if !halfWindow {
		return refuse(`Stitch's flux path needs a Torus face bounded by the tube's own two equatorial rims (its ±π/2 window); this evaluator has no sound way to recover a narrower angular window without an unbounded trig computation`)
	}

	major := measuredScalar(majorValue, 0)
	minor := measuredScalar(minorValue, 0)
	majorSq := boundedMul(major, major)
	minorSq := boundedMul(minor, minor)
	pi := piScalar()
	piSq := boundedMul(pi, pi)

	term1 := boundedMul(measuredScalar(3, 0), boundedMul(piSq, boundedMul(major, minorSq)))
	term2 := boundedMul(measuredScalar(4, 0), boundedMul(pi, boundedMul(minor, boundedAdd(majorSq, minorSq))))
	kf := boundedAdd(term1, term2)
	flux = boundedMul(measuredScalar(sign, 0), kf)

	majMinorPi := boundedMul(pi, boundedMul(major, minor))
	fourThirdsMinorSq := boundedMul(boundedQuotient(4, 0, 3, 0), minorSq)
	moment := func(centerI, anchorI, axisI float64) boundedScalar {
		axisISq := boundedMul(measuredScalar(axisI, 0), measuredScalar(axisI, 0))
		oneMinusAxisISq := boundedSub(measuredScalar(1, 0), axisISq)
		bracket := boundedAdd(
			boundedAdd(boundedMul(measuredScalar(2, 0), boundedMul(majorSq, oneMinusAxisISq)), majMinorPi),
			fourThirdsMinorSq,
		)
		d := boundedSub(measuredScalar(centerI, 0), measuredScalar(anchorI, 0))
		m := boundedMul(pi, boundedMul(minor, boundedMul(d, bracket)))
		return boundedMul(measuredScalar(sign, 0), m)
	}
	mx = moment(t.Center.X, anchor.X, t.Axis.X)
	my = moment(t.Center.Y, anchor.Y, t.Axis.Y)
	mz = moment(t.Center.Z, anchor.Z, t.Axis.Z)
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
		switch surf := f.surface.(type) {
		case Cylinder:
			if len(f.loops) > 0 && len(f.loops[0].coedges) > 0 {
				// A rim vertex's own distance from anchor is what the loop
				// above already folds in; this adds the cylinder's own
				// radius — its PROVEN value plus bound, boundedCircleRadius's
				// own doc comment, never Cylinder.Radius.Base() read bare —
				// as a blanket safety margin so coordUpper never understates
				// a wall point that sits farther from anchor than either rim
				// vertex does, however far the tagged Radius itself sits
				// from the true one. Both rims share one radius, so reading
				// either suffices.
				if rB, err := boundedCircleRadius(f.loops[0].coedges[0].edge); err == nil {
					coordUpper = absSumUpper(coordUpper, rB.value, rB.bound)
				}
			}
		case Cone:
			// A cone wall is ruled (straight rulings from the apex), so the
			// farthest wall point from any anchor lies on one of the two rim
			// CIRCLES, not necessarily at either rim's own seam vertex — the
			// identical gap the Cylinder margin above closes, but the two
			// rims here carry DIFFERENT radii, so both need their own
			// margin rather than just one.
			for _, l := range f.loops {
				if len(l.coedges) == 0 {
					continue
				}
				if rB, err := boundedCircleRadius(l.coedges[0].edge); err == nil {
					coordUpper = absSumUpper(coordUpper, rB.value, rB.bound)
				}
			}
		case Sphere:
			// A zero-loop Sphere face contributes NOTHING to the loop-based
			// scan above — it has no loop at all — so without this arm
			// coordUpper would silently ignore the sphere's own surface
			// entirely, however far it sits from anchor: exactly the "an
			// intervening term happened to be zero" trap CLAUDE.md warns
			// against, here because the fixture has no VERTEX to hide behind
			// rather than a zero coordinate. Every point of the sphere sits
			// within Center's own distance from anchor plus Radius, so that
			// sum is the margin, with Radius read through
			// boundedSphereRadius — never Sphere.Radius bare — exactly as
			// the Cylinder/Cone margins above read theirs through
			// boundedCircleRadius.
			if rB, err := boundedSphereRadius(f); err == nil {
				coordUpper = absSumUpper(coordUpper, surf.Center.Sub(anchor).Len(), rB.value, rB.bound)
			}
		case Torus:
			// Every point of a Torus face this file admits lies within
			// Major+Minor of Center's own radial position and within Minor
			// of its own axial position, regardless of which φ window the
			// face spans, so Center's own distance from anchor plus
			// Major+Minor is a safe margin — the same shape the Sphere
			// case above takes. Reaching this loop at all means
			// sumStitchFlux already ran torusFaceFluxAndMoment
			// successfully for this face, which is what proves
			// torusAxisIsCoordinateAligned and a positive Major/Minor
			// here; this reads them bare only because that proof already
			// ran, never in place of it.
			if majorValue, merr := surf.Major.In(units.Millimeter); merr == nil {
				if minorValue, nerr := surf.Minor.In(units.Millimeter); nerr == nil {
					coordUpper = absSumUpper(coordUpper, surf.Center.Sub(anchor).Len(), majorValue+minorValue)
				}
			}
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
