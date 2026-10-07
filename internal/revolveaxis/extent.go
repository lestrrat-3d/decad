package revolveaxis

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// SectionCoordUpper widens the RECORDED meridian's own coordinate envelope
// into one that covers the meridian the record DENOTES
// (docs/surface-intersection-design.md §7.1). coordUpper is an L1 magnitude
// (internal/boundarywalk/walk.go's ratL1Upper), so a point whose two components each move by
// at most sectionDelta adds at most twice it. Every term the axis frame and the
// sweep extreme charge at an envelope has to be charged at THIS one, or those
// two terms would be proven against the recorded meridian while the extreme
// they bound sits on the denoted one.
//
// A payload no construction displaced answers its own argument back, untouched:
// proofbound.AbsSumUpper up-rounds, so folding a zero would widen an ordinary revolve's
// every published box by an ulp.
func SectionCoordUpper(coordUpper, sectionDelta float64) float64 {
	if sectionDelta <= 0 {
		return coordUpper
	}
	return proofbound.AbsSumUpper(coordUpper, proofbound.ProductUpper(2, sectionDelta))
}

// SectionExtentAllow is docs/surface-intersection-design.md §7.1's FIFTH
// mechanism in extentBoundedAlong's own enumeration: how far the extreme of the
// linear functional wg·z + m·ρ moves when the meridian it is taken over is the
// recorded one rather than the denoted one.
//
// A recorded endpoint displaced by at most sectionDelta in each plane-local
// component moves its two AXIS coordinates by at most axisCharge's own two
// figures, each bounded by sectionDelta·(|dU| + |dV|) — the same fold §7.1
// derives for the walk, read here for the envelope rather than per endpoint.
// The functional's own coefficients satisfy wg² + c0² + c1² = 1 for a unit g
// over the orthonormal basis, so |wg| ≤ 1 and |m| ≤ 1 and the extreme moves by
// at most δz + δρ. The held basis departs from orthonormal only by the rounding
// frameRoundAllow already charges, at an envelope sectionCoordUpper widens, so
// that departure is accounted for there rather than doubled here.
//
// It displaces both ends the same way, so it composes OUTWARD with the per-end
// maximum exactly as frameRoundAllow does, never folded into one end's own sum.
func SectionExtentAllow(ax Frame, sectionDelta float64) float64 {
	if sectionDelta <= 0 {
		return 0
	}
	per := proofbound.ProductUpper(sectionDelta, proofbound.AbsSumUpper(ax.DU, ax.DV))
	return proofbound.ProductUpper(2, per)
}

// FrameRoundAllow bounds how far base/wg/c0/c1 — the four scalar coefficients
// extentBoundedAlong lifts the boundary and sweep extremes through — can sit
// from the values the axis's TRUE (unrounded) direction/anchor and the
// placement's own EXACT arithmetic would give. Two independent mechanisms
// compose outward:
//
//   - axisAllow: the axis frame's own direction/anchor uncertainty
//     (dUBound/dVBound/aUBound/aVBound, axisInPlane). Every material point of
//     the swept solid is xform.Apply(a3 + w·z + ρ·(e0·cos φ + e1·sin φ)) for
//     some (z, ρ, φ) the recorded boundary and sweep interval admit, with
//     |z|, |ρ| both bounded by envUpper = ax.radialUpper(coordUpper) — the
//     SAME envelope axisFrame.radialUpper already states for ρ, since
//     z = (p−a)·d and ρ = |cross(d, p−a)| are both bounded by |p−a| for a
//     unit d. Perturbing the anchor by its own proven bound moves a3 by at
//     most anchorAllow (through the frame's own unit U/V); perturbing the
//     direction by its own proven bound moves w and e0 by at most dirAllow
//     each (the same construction), and e1 = w×e0 by at most e1Allow (the
//     cross product's own two-term expansion, worst-cased at unit |w|, |e0|).
//     g is always a unit world axis, so a bound on the pre-transform point's
//     own displacement bounds its g-projection too (Cauchy-Schwarz) — the
//     isometry carries a magnitude bound through unchanged, in exact
//     arithmetic.
//   - placeAllow: the placement's own rounding, through
//     proofbound.ExactIsometryDotRound's exact rational check (internal/proofbound/bounds.go) on each of
//     base/wg/c0/c1 against the SAME frame+placement chain applied to the
//     ALREADY-HELD a3/w/e0/e1 — zero exactly where the placement's own float
//     arithmetic is exact for this input (an identity placement) regardless
//     of how tilted the axis frame itself is. wg's displacement moves the
//     extreme at the rate of |z| ≤ envUpper; c0's and c1's at the rate of
//     |ρ| ≤ envUpper (the swept radial coefficient multiplies ρ); base's
//     displaces the extreme directly, at both ends alike.
func FrameRoundAllow(
	ax Frame, sectionDelta float64, xform r3.Transform, g r3.Vec, b revolvemesh.RevolveBasis,
	base, wg, c0, c1, coordUpper float64,
) float64 {
	envUpper := ax.RadialUpper(SectionCoordUpper(coordUpper, sectionDelta))
	dirAllow := proofbound.AbsSumUpper(ax.DUBound, ax.DVBound)
	e1Allow := proofbound.AbsSumUpper(proofbound.ProductUpper(2, dirAllow), proofbound.ProductUpper(dirAllow, dirAllow))
	anchorAllow := proofbound.AbsSumUpper(ax.AUBound, ax.AVBound)
	axisAllow := proofbound.AbsSumUpper(
		anchorAllow,
		proofbound.ProductUpper(dirAllow, envUpper),
		proofbound.ProductUpper(envUpper, proofbound.AbsSumUpper(dirAllow, e1Allow)),
	)

	baseRound := proofbound.ExactIsometryDotRound(xform, b.A3, g, true, base)
	wgRound := proofbound.ExactIsometryDotRound(xform, b.W, g, false, wg)
	c0Round := proofbound.ExactIsometryDotRound(xform, b.E0, g, false, c0)
	c1Round := proofbound.ExactIsometryDotRound(xform, b.E1, g, false, c1)
	placeAllow := proofbound.AbsSumUpper(
		baseRound,
		proofbound.ProductUpper(wgRound, envUpper),
		proofbound.ProductUpper(envUpper, proofbound.AbsSumUpper(c0Round, c1Round)),
	)

	return proofbound.AbsSumUpper(axisAllow, placeAllow)
}

// ExtentBoundarySweepBound is the per-end composition: the outward sum of the two terms,
// through the same proofbound.AbsSumUpper every other composed bound in this package
// takes. A non-finite term answers +Inf rather than folding into a small
// bound (internal/proofbound/bounds.go's own rule), since proofbound.AbsSumUpper is an arithmetic on
// magnitudes and states nothing about an absent one.
func ExtentBoundarySweepBound(loBound, hiBound, sweepLo, sweepHi float64) float64 {
	outward := func(boundary, sweep float64) float64 {
		if proofbound.IsNonFinite(boundary) || proofbound.IsNonFinite(sweep) {
			return math.Inf(1)
		}
		return proofbound.AbsSumUpper(boundary, sweep)
	}
	return math.Max(outward(loBound, sweepLo), outward(hiBound, sweepHi))
}

// FinishExtent charges the final summation base + lo and base + hi against
// those terms through proofbound.ExactSumRound
// (internal/proofbound/bounds.go). frameRoundAllow proves base/wg/c0/c1 each right and says
// nothing about adding them: a pure translation of an axis-aligned revolve
// leaves all four exactly right and still rounds here. It is charged per END
// — the two ends are summed from different terms — and composed outward with
// the per-end maximum above, through the same triangle inequality every
// other composition in this reading takes.
//
// The meridian this reading scanned is the RECORDED
// one, and a trimmed section's own cut coordinates sit within its
// sectionDelta of the meridian the record denotes
// (docs/surface-intersection-design.md §7.1). It is zero for every payload
// no construction displaced, which is what leaves an ordinary revolve's box
// on the path it takes today.
func FinishExtent(ax Frame, sectionDelta, base, lo, hi, bound, frameAllow float64) (float64, float64, float64) {
	loEnd, hiEnd := base+lo, base+hi
	sumAllow := math.Max(
		proofbound.ExactSumRound(loEnd, base, lo),
		proofbound.ExactSumRound(hiEnd, base, hi),
	)
	bound = proofbound.AbsSumUpper(bound, frameAllow, sumAllow)
	if sectionAllow := SectionExtentAllow(ax, sectionDelta); sectionAllow > 0 {
		bound = proofbound.AbsSumUpper(bound, sectionAllow)
	}
	return loEnd, hiEnd, bound
}

// SweepBoundAlong is the swept radial coefficient's own contribution to the
// extent's half-width along one direction: revolveangle.ExtremeBounds proves how far
// the held sweep extreme (mlo, mhi) can sit from the true one, and that
// direction error turns into a position error through the same
// directional-perturbation Lipschitz bound (internal/proofbound/bounds.go) every directional
// extreme charges.
//
// It returns the LOW end's term and the HIGH end's term separately, never their
// larger. Each end's extreme is evaluated at its own held coefficient — the low
// end at mlo, the high end at mhi — so each carries only its own coefficient's
// displacement, and the caller composes it with that same end's boundary-scan
// bound. Folding the two ends together here would hand the caller one number it
// could no longer attribute to an end, and the composition it owes is per end.
//
// The envelope that Lipschitz step charges is the RADIAL one: the extreme's
// own functional is wg·z + m·ρ, so a perturbation of the swept radial
// coefficient m multiplies ρ, the distance from the RESOLVED AXIS, and not the
// profile's coordinates about the frame origin. axisFrame.radialUpper owns that
// envelope and folds in the axis anchor, which is the whole term an offset axis
// adds; an extent whose radial envelope cannot be proven finite is refused
// rather than published against a bound that omits it.
func SweepBoundAlong(
	ax Frame, sectionDelta, c0, c1, phi0, phi1 float64,
	den revolveangle.Sweep, mlo, mhi float64, full bool, coordUpper float64,
) (float64, float64, error) {
	rhoUpper := ax.RadialUpper(SectionCoordUpper(coordUpper, sectionDelta))
	if proofbound.IsNonFinite(rhoUpper) {
		return 0, 0, fmt.Errorf(`%w: the revolved region's radial distance from its own axis has no finite proven bound, so no sweep-extreme bound can be composed`, decaderr.ErrNotFinite)
	}
	loBound, hiBound := revolveangle.ExtremeBounds(c0, c1, phi0, phi1, den, mlo, mhi, full)
	return proofbound.DirectionalPerturbationAllow(loBound, rhoUpper),
		proofbound.DirectionalPerturbationAllow(hiBound, rhoUpper),
		nil
}

// ExtremeFromScan is one extreme of the linear functional wg·z + k·ρ over the
// recorded boundary, evaluated in axis coordinates through the plane-local
// boundary extremes, beside that extreme's own proven bound.
//
// Three mechanisms compose into that bound. The scan's own is nonzero exactly
// where a candidate's own POSITION is not exactly representable — a circular
// candidate's radius or apex (extrude.go's circularExtremeInterval), a computed
// walk endpoint, a free-form span's enclosure — and it is proven over the
// SECTION's coordinates.
//
// The scan's own ARITHMETIC is the second, and it is independent of the first:
// the scan holds each candidate as the float gu·u + gv·v, and that
// multiply-and-sum rounds at the section's coordinate magnitude even where the
// candidate's position is a value the record states verbatim and the first term
// is therefore zero. proofbound.PlaneDotDecompositionRoundAllow (internal/proofbound/bounds.go) charges it at
// the section's own coordinate envelope, which is the only magnitude in this
// reading it scales with — the prism reading's prismDecompositionRoundAllow is
// the same mechanism one sweep coordinate wider, and neither reading ever reads
// the other's term.
//
// The anchor shift below is the third: this reading's own arithmetic — two
// products, their sum, and the subtraction that carries the scan's extreme into
// axis coordinates — and every one of those rounds at the ANCHOR's magnitude
// rather than the section's, so an axis far from the frame origin rounds here
// while the scan reports zero. proofbound.ExactPlaneDotRound and proofbound.ExactSumRound (internal/proofbound/bounds.go)
// charge exactly what that arithmetic committed, and the anchor's own proven
// uncertainty (axisInPlane's aUBound/aVBound) rides in beside them through the
// direction it is read against.
func ExtremeFromScan(
	ax Frame, gu, gv, lo, hi, bound, coordUpper float64, wantMax bool,
) (float64, float64) {
	scanAllow := proofbound.PlaneDotDecompositionRoundAllow(gu, gv, coordUpper)
	off := gu*ax.AU + gv*ax.AV
	shiftAllow := proofbound.AbsSumUpper(
		proofbound.ExactPlaneDotRound(gu, gv, ax.AU, ax.AV, off),
		proofbound.ProductUpper(math.Abs(gu), ax.AUBound),
		proofbound.ProductUpper(math.Abs(gv), ax.AVBound),
	)
	scan := hi
	if !wantMax {
		scan = lo
	}
	extreme := scan - off
	return extreme, proofbound.AbsSumUpper(bound, scanAllow, shiftAllow, proofbound.ExactSumRound(extreme, scan, -off))
}
