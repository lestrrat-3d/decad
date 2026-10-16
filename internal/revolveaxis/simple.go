package revolveaxis

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ProvesSimple decides revolveplan.Payload's leg-4 admission:
// a revolve's own build runs no crossing audit over a triangle set the
// way a loft's does, so its non-self-intersection proof has to be a
// construction argument over the profile and the resolved axis instead.
//
// THE EMBEDDEDNESS ARGUMENT, repaired. `sketch` already proved the recorded
// profile a simple closed planar region. Every boundary stretch lying ON the
// resolved axis is classified wallAxis (internal/revolveaxis/wall_kind.go's Classify)
// and buildRevolveLoop skips it when assembling faces — it sweeps a
// zero-area set and emits no face at all — so the finished face set is the
// revolution of the boundary's OFF-axis part alone. Two distinct off-axis
// boundary points can only map to the same 3D point by sharing the same
// radius and axial position at the same swept angle, and a simple curve
// never repeats a (radius, axial) pair off the axis, so that face set is
// injective off the axis. full == true is what makes the angular fibre
// traverse the sweep exactly once — any other multiple of a turn would cover
// some off-axis point more than once — so both full and the radial condition
// below are load-bearing, and neither alone is enough. This is deliberately
// NOT the claim that the revolution "meets the axis in at most a point":
// semicircleSketch's diameter lies wholly ON the axis, and its sphere sheet
// is a direct counterexample to that stronger claim. The argument above never
// needs it — an on-axis stretch contributes no face to intersect anything
// with in the first place.
//
// THE RADIAL CONDITION IS STRICTER THAN THE BUILD GATE, on purpose.
// revolveplan.ResolveAxisSide already refused, at build, any profile whose radial extreme
// does not clear the axis to within tol = 1e-9 * max(1, scale) — it proves
// only that the radial minimum is not below −tol, never that it is
// non-negative. An admission gate resting on that tolerance is exactly what
// CLAUDE.md's reject-only rule forbids outright, so this leg re-decides the
// question at zero instead of inheriting the build's tolerance: it admits
// only when the radial minimum is PROVEN non-negative (rlo − rBound >= 0),
// not merely un-disproven.
//
// NO FREE-FORM GATE IS ADDED HERE. A revolve build calls
// revolveplan.ResolveAxisSide, whose AuditProfileAxisContact
// (recorded_walks.go) refuses a free-form generator when Frame.RadialLower is
// not positive. This leg adds no free-form check of its own.
//
// A direct Revolve can carry the strict positive-side proof produced by
// revolveplan.ResolveAxisSide's scan of this same profile and axis. Its scan adds a
// dot-product charge that makes the proof at least as strict as this scan.
// Placement keeps the profile and axis, so it keeps that proof; a changed
// profile or axis clears it. Other payloads take the scan below.
//
// The radial extreme is otherwise recomputed against ax, the AXIS THE
// BUILD ALREADY RESOLVED, rather than re-deriving a side or a tolerance: ax's
// own doc comment (axisFrame) states ρ = cross(d, p−a) as its radial
// coordinate with the region already oriented onto its non-negative side, so
// evaluating that same functional's extreme over the recorded profile reads
// the identical quantity revolveplan.ResolveAxisSide decided from, at the strict zero
// bound this leg requires instead of the build's ±tol band. A failure to
// measure it (never observed against a payload this evaluator built, since
// the identical scan already succeeded once at build time over the same
// profile) is read as un-proven rather than as an error: this leg only ever
// admits, so withholding admission is always the safe outcome.
//
// prismextent.ProfileBoundaryExtremes reads the raw functional nU*u+nV*v over the
// profile boundary and charges every rounding IT commits into rBound. It does
// NOT charge the axis anchor's own offset, roff = nU*aU+nV*aV, which this leg
// still has to subtract to reach the axis-relative ρ axisFrame's own doc
// comment defines: uAxis (this arm's own fixtures) happens to anchor at the
// origin, where aU and aV are both exactly zero and the shift is a no-op, but
// an axis anchored elsewhere makes roff genuinely nonzero, and the multiplies,
// the add and the subtraction that compute it each commit their OWN
// round-to-nearest error on top of whatever aU/aV/dU/dV's own proven bounds
// already are. A bare `rlo -= roff` would leave that error unaccounted, so
// the strict-zero comparison below could read a NEGATIVE true radial minimum
// as non-negative by exactly the rounding it dropped — the identical fault
// this leg exists to close in revolveplan.ResolveAxisSide's build-time tolerance, only
// smaller. proofbound.BoundedMul/proofbound.BoundedAdd/proofbound.BoundedSub charge both the operands' own
// proven bounds and this arithmetic's own commissioned rounding (the same
// vocabulary Frame.ToAxisRhoBound already uses for the identical ρ
// formula at a single point), so the bound handed to proofbound.AdmitBelow below
// provably covers every operation between the extremes call and the
// comparison — never a number the accompanying bound does not cover.
//
// scan is the root's profile extreme reader (prismextent.ProfileBoundaryExtremes),
// injected as ExtentAlong's is.
func ProvesSimple(ctx context.Context, profile momentinput.Profile, ax Frame, full, radialProof bool,
	sectionDelta float64, scan ExtentScan) bool {
	if !full || sectionDelta != 0 {
		// A displaced meridian (docs/surface-intersection-design.md §7.2) is
		// read the way a displaced prism section is: the radial extreme below
		// is of the recorded meridian, not the denoted one, so the leg is
		// undecided.
		return false
	}
	if ctx.Err() != nil {
		return false
	}
	if radialProof {
		return true
	}
	nU, nV := -ax.DV, ax.DU
	rawLo, _, rawBound, err := scan(ctx, profile, nU, nV, freeform.NewFreeformWork(), nil)
	if err != nil {
		return false
	}
	offset := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(nU, ax.DVBound), proofbound.MeasuredScalar(ax.AU, ax.AUBound)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(nV, ax.DUBound), proofbound.MeasuredScalar(ax.AV, ax.AVBound)),
	)
	rlo := proofbound.BoundedSub(proofbound.MeasuredScalar(rawLo, rawBound), offset)
	return proofbound.AdmitBelow(rlo, 0) == proofbound.SurvReject
}
