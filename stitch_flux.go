package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/surface-design.md §6.4's per-surface flux integral: the
// closed-form volume and first-moment reading that lets Stitch close a
// boundary holding a curved face, for the Plane, Cylinder, Cone, Sphere and
// Torus variants. Table R row R8 is a per-face dispatch: a face this
// evaluator has no closed-form flux integral for refuses R8, and a Plane,
// Cylinder, Cone, Sphere or Torus face with a zero normalBound admits — a
// revolve face only where its tag is exactly the surface its record denotes
// (stitchFluxTagsDenoted, at the bottom of this file).
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
// Cylinder.Radius/Circle3.Radius — stitchflux.CircleRadius derives it, and its
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
// Plane arm uses the standard disk/annulus second-moment identity when
// the active world-axis moments have in-plane components (a full circle of
// radius R centered at (cu,cv) in the face's own
// local frame contributes cu·Area, cv·Area, cu²·Area+πR⁴/4, cv²·Area+πR⁴/4
// and cu·cv·Area to Iu, Iv, Iuu, Ivv, Iuv respectively — elementary polar
// integration over a disk, summed with each loop's own outer/hole sign).
// When the normal is exactly world-axis aligned, only disk area enters the
// nonzero moment component. The
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
// through proofbound.BoundedAdd/proofbound.BoundedSub/proofbound.BoundedMul/proofbound.BoundedQuotient — never a bare
// float composed by hand — so a mistake here shows up as a bound the
// shown-to-fail tests can watch go red, not as a silent understatement.
// Every term that carries π (every area- and fourth-moment-of-a-disk term,
// every Cylinder moment term) reads it from stitchflux.PiScalar(): Go's math.Pi is the
// correctly rounded float64 nearest true π, so its own representation error
// is one ulp, and every subsequent multiply charges its own rounding
// through proofbound.BoundedMul on top of that — a tight bound, not
// proofbound.ConservativeValueError's structural "assume no cancellation at all"
// fallback, which is sized for a quantity this evaluator has no other way
// to bound and would overstate π's own error by many orders of magnitude —
// wide enough, on a thin annulus, to starve stitchCurvedMass's own
// proofbound.BoundedQuotient calls of clearance and refuse a perfectly sound fixture.
// Nothing here ever claims Exact: every admitting arm's own K_F or moment
// carries π, so exactnessOf's zero-bound test can never fire for a curved
// stitched solid.
//
// The radius itself carries a second, independent source of bound
// (stitchflux.CircleRadius's own doc comment): a revolve about an axis that is
// not coordinate-aligned through the origin hands back a Radius that is
// itself a rounded re-expression of a recorded point, and every use of it
// here goes through proofbound.BoundedMul against that proven bound rather than
// assuming the bare units.Value is exact. Every PUBLIC fixture this PR
// tests (apitest/stitch_flux_test.go) revolves about a coordinate-aligned axis
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
// it has no rim edge to read stitchflux.CircleRadius's own circumference from
// at all (this arm's own scope restriction admits only a zero-loop face):
// stitchflux.SphereRadius inverts the face's own already-proven area/areaBound
// instead (its own doc comment). The public T50 fixture's own area bound is
// the same tiny, ulp-scale magnitude every other term here carries, so
// TestStitchSphereFluxReusesAreaBound (stitch_internal_test.go) pins the
// leg on a hand-built face carrying a synthetic areaBound far above that
// noise floor, the same treatment TestStitchCylinderFluxReusesAreaBound
// gives the Cylinder arm's own area-bound reuse.
//
// The Cone arm's apex is Origin when Radius is exactly 0 — the ONLY case
// this evaluator admits. A revolve cone's Origin is its walk's float apex,
// and the arm widens it by how far it sits from the record's
// (stitchflux.ConeApexDeparture). The general formula, Origin −
// Axis·(Radius/tan(HalfAngle)), needs tan(HalfAngle)'s own rounding charged
// before that division could be trusted, and this evaluator has no sound
// way to do that: Go gives Sin/Cos/Atan2/Hypot (and so Tan) no public ulp
// contract, and composing tan from boundedSin/proofbound.BoundedCos and running it
// through proofbound.BoundedQuotient — the natural-looking fix — fails
// proofbound.BoundedQuotient's own clearance check unconditionally, for every angle,
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
// stitchflux.AxisIsCoordinateAligned rather than derived — the one condition
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
// isolated point could open, or two otherwise-unconnected stitched bodies
// sharing exactly one welded vertex-table entry with no edge joining them
// (docs/surface-design.md's own record of both shapes). This is a
// reject-only gate, called unconditionally on every build arm
// (docs/surface-design.md §6.4): it can only narrow what evalStitchContext
// admits, never bless anything checkStitchClosure did not already allow
// through.
//
// auditVertexLinks itself is worded for its OTHER caller, the composite
// sweep (sweep_composite.go), and reports [ErrUnsupported] there. Translated
// here rather than reworded in place, so the composite sweep's own wording
// and sentinel stay exactly as that caller states them: a pinch this
// evaluator proves is disproven input for Stitch, never unsupported reach
// (docs/surface-design.md Table R row R7), so this wrapper re-reports it as
// [ErrDegenerate] with Stitch's own wording. A bare context cancellation —
// auditVertexLinks' own budget-polling return, never wrapped in
// [ErrUnsupported] — passes through unchanged.
func auditVertexLinksForStitchFaces(ctx context.Context, faces []*Face) error {
	budget := proofbound.NewWorkBudget(ctx)
	uses := map[*Edge][]compositeCoedgeUse{}
	for _, face := range faces {
		for _, loop := range face.loops {
			for _, ce := range loop.coedges {
				if err := budget.Step(); err != nil {
					return err
				}
				uses[ce.edge] = append(uses[ce.edge], compositeCoedgeUse{face: face, forward: ce.forward})
			}
		}
	}
	if err := auditVertexLinks(budget, uses); err != nil {
		if errors.Is(err, ErrUnsupported) {
			return fmt.Errorf(`%w: Stitch's welded set has a vertex whose meeting faces are not one connected fan or path (docs/surface-design.md Table R row R7): %s`, ErrDegenerate, err)
		}
		return err
	}
	return nil
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
func stitchFaceFluxAndMoment(f *Face, anchor r3.Vec) (flux, mx, my, mz proofbound.BoundedScalar, err error) {
	if f.normalBound != 0 {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
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
		return stitchflux.PlaneFaceFluxAndMoment(stitchFluxFaceInput(f), s.Frame, anchor, sign)
	case Cylinder:
		return stitchflux.CylinderFaceFluxAndMoment(stitchFluxFaceInput(f), s.Origin, s.Axis, anchor, sign)
	case Cone:
		return coneFaceFluxAndMoment(f, s, anchor, sign)
	case Sphere:
		return stitchflux.SphereFaceFluxAndMoment(stitchFluxFaceInput(f), s.Center, anchor, sign)
	case Torus:
		input := stitchflux.TorusInput{Center: s.Center, Axis: s.Axis, Major: s.Major, Minor: s.Minor}
		return stitchflux.TorusFaceFluxAndMoment(stitchFluxFaceInput(f), input, anchor, sign)
	default:
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch closes a boundary holding a %T face this evaluator has no closed-form flux integral for (docs/surface-design.md Table R row R8)`,
			ErrUnsupported, s,
		)
	}
}

func stitchFluxFaceInput(f *Face) stitchflux.FaceInput {
	input := stitchflux.FaceInput{
		Loops:     make([]stitchflux.LoopSource, len(f.loops)),
		Area:      f.area,
		AreaBound: f.areaBound,
	}
	for i, l := range f.loops {
		input.Loops[i] = stitchLoopSource{loop: l}
	}
	return input
}

type stitchLoopSource struct{ loop *Loop }

func (s stitchLoopSource) EdgeCount() int { return len(s.loop.coedges) }

func (s stitchLoopSource) IsOuter() bool { return s.loop.outer }

func (s stitchLoopSource) Circle() (stitchflux.CircleRim, bool, string) {
	edge := s.loop.coedges[0].edge
	circle, ok := edge.curve.(Circle3)
	if !ok {
		return stitchflux.CircleRim{}, false, fmt.Sprintf("%T", edge.curve)
	}
	return stitchflux.CircleRim{
		Center:          circle.Center,
		Length:          edge.length,
		LengthBound:     edge.lengthBound,
		LengthUnbounded: edge.lengthUnbounded,
	}, true, ""
}

func coneFaceFluxAndMoment(f *Face, cone Cone, anchor r3.Vec, sign float64) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error) {
	apexBound, ok := stitchflux.ConeApexDeparture(f.denoted, surfacegeom.Cone(cone))
	if !ok {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, fmt.Errorf(
			`%w: Stitch's flux path cannot state how far this cone's apex sits from the apex its record denotes (docs/surface-design.md Table R row R8)`,
			ErrUnsupported,
		)
	}
	input := stitchflux.ConeInput{Origin: cone.Origin, Axis: cone.Axis, Radius: cone.Radius, HalfAngle: cone.HalfAngle, ApexBound: apexBound}
	return stitchflux.ConeFaceFluxAndMoment(stitchFluxFaceInput(f), input, anchor, sign)
}

// stitchCurvedMass sums every face's own flux and first moment
// (stitchFaceFluxAndMoment), decides the global orientation sign from its
// own total — reversing every face and recomputing once, the curved
// analogue of stitch.go's acc.vol6.Sign() < 0 step — divides by three, and
// charges the placement allowance (proofbound.SweptVolumeAllow/proofbound.SweptMomentAllow) on
// the same terms capband.BandVolume does for a placed curved solid: a
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
	if fluxSum.Value < 0 {
		for _, f := range faces {
			reverseFaceOrientation(f)
		}
		fluxSum, momX, momY, momZ, err = sumStitchFlux(faces, anchor)
		if err != nil {
			return Measurement{}, VecMeasurement{}, err
		}
	}

	envelopes := make([]stitchflux.MassEnvelopeFace, 0, len(faces))
	for _, f := range faces {
		envelope := stitchflux.MassEnvelopeFace{Surface: internalSurface(f.surface), Area: f.area, AreaBound: f.areaBound}
		for i, loop := range f.loops {
			for _, ce := range loop.coedges {
				envelope.Vertices = append(envelope.Vertices, ce.Start().Position().Value)
			}
			if len(loop.coedges) > 0 {
				edge := loop.coedges[0].edge
				rim := stitchflux.CircleRim{
					Length: edge.length, LengthBound: edge.lengthBound, LengthUnbounded: edge.lengthUnbounded,
				}
				envelope.Rims = append(envelope.Rims, rim)
				if i == 0 {
					envelope.FirstRim = &rim
				}
			}
		}
		envelopes = append(envelopes, envelope)
	}
	areaUpper, coordUpper := stitchflux.MassEnvelope(envelopes, anchor)

	vol, centre, state := stitchflux.MassFromFlux(fluxSum,
		[3]proofbound.BoundedScalar{momX, momY, momZ}, areaUpper, coordUpper, delta)
	if state == stitchflux.MassZeroVolume {
		return Measurement{}, VecMeasurement{}, fmt.Errorf(`%w: a stitched curved solid with zero net volume has no centroid`, ErrDegenerate)
	}
	if state == stitchflux.MassUnboundedCentroid {
		return Measurement{}, VecMeasurement{}, fmt.Errorf(`%w: the placement's proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, ErrUnsupported)
	}
	volMeasurement := Measurement{
		Value:     units.CubicMillimeters(vol.Value),
		Exactness: exactnessOf(vol.Bound),
		Bound:     units.CubicMillimeters(vol.Bound),
	}
	fx, fy, fz := anchor.X+centre[0].Value, anchor.Y+centre[1].Value, anchor.Z+centre[2].Value

	bound := proofbound.Radius3D(math.Max(centre[0].Bound, math.Max(centre[1].Bound, centre[2].Bound)))
	centroidMeasurement := VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}
	return volMeasurement, centroidMeasurement, nil
}

// sumStitchFlux sums every face's own flux and first moment, refusing R8 as
// soon as any one face's own dispatch does.
func sumStitchFlux(faces []*Face, anchor r3.Vec) (flux, momX, momY, momZ proofbound.BoundedScalar, err error) {
	for _, f := range faces {
		fFlux, fmx, fmy, fmz, err := stitchFaceFluxAndMoment(f, anchor)
		if err != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
		}
		flux = proofbound.BoundedAdd(flux, fFlux)
		momX = proofbound.BoundedAdd(momX, fmx)
		momY = proofbound.BoundedAdd(momY, fmy)
		momZ = proofbound.BoundedAdd(momZ, fmz)
	}
	return flux, momX, momY, momZ, nil
}

// stitchFluxTagsDenoted is the gate docs/surface-design.md §6.4 puts in front
// of the flux arms for a revolve face. Every arm integrates the face's TAG —
// a Cylinder's rims and area, a Plane's origin and normal, a Cone's apex and
// axis, a Sphere's or Torus's centre — while a revolve face's tag is only a
// float re-expression of the surface its record denotes (Face.denoted): a
// segment within the classifier's slope tolerance of parallel or
// perpendicular is tagged a Cylinder or a Plane, a centre within the contact
// tolerance of the axis is snapped onto it, and every axial coordinate is
// rounded. No arm charges that departure, and none could by a per-face term:
// the arms' formulas read different numbers off neighbouring faces' tags, so
// a departed tag leaves the modelled boundary open by an amount whose volume
// depends on the anchor. So a face carrying a denoted surface is admitted only
// where its tag IS that surface, decided exactly (stitchflux.TagIsDenoted); every
// other face's tag is its own record. It reads the unplaced operand faces:
// a stitch's own placement rounding is its delta.
func stitchFluxTagsDenoted(faces []*Face) bool {
	for _, f := range faces {
		if f.denoted != nil && !stitchflux.TagIsDenoted(stitchTaggedFace(f)) {
			return false
		}
	}
	return true
}

// stitchTaggedFace passes a face's surface and edge curves to the exact
// denotation gate without giving the internal package topology ownership.
func stitchTaggedFace(f *Face) stitchflux.TaggedFace {
	tagged := stitchflux.TaggedFace{Denoted: f.denoted, Surface: internalSurface(f.surface)}
	for _, loop := range f.loops {
		for _, ce := range loop.coedges {
			tagged.Curves = append(tagged.Curves, internalCurve(ce.edge.curve))
		}
	}
	return tagged
}
