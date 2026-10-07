package revolveaxis

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SideExtremes are the bounded radial and axial profile scans before the
// resolved axis anchor and direction charges are applied.
type SideExtremes struct {
	RLo, RHi, RBound float64
	ZLo, ZHi, ZBound float64
	CoordUpper       float64
}

// SideResult holds the oriented frame and the proof charges its side gate earns.
type SideResult struct {
	Frame            Frame
	RadialAdmitAllow float64
	RadialProof      bool
	AxialExtentUpper float64
	Side             float64
}

// ResolveSide orients the axis so the recorded region lies on its
// non-negative-ρ side, enforcing the §6 half-plane and contact rules: a
// region with boundary on both sides of the axis is rejected, as is a curve
// tangent to the axis at an interior point (a circle kissing the axis would
// sweep a self-touching horn torus). It returns the oriented frame and the
// side sign the sweep interval must be remapped by.
//
// The two radial extremes are read through the boundary scan's bounded form,
// so a section whose extreme rides a computed arc radius arrives with a proven
// interval rather than a held float claiming to be exact. Each of the three
// decisions below is taken on that interval and must be DECIDED by it: the
// side sign is a structural fact about the region, not a measurement, so a
// radial extreme whose own interval straddles the ±tol band names no side and
// is refused rather than guessed. The band is 1e-9 of the section's own scale
// and the bound is the scan's own last-ulp figure, so an ordinary section is
// decided with eight orders of magnitude to spare; reaching the refusal means
// the region genuinely sits on the axis to within its own arithmetic.
//
// roff/zoff are the axis anchor's own offset along each functional: the scan
// above reads the profile about the FRAME origin, and the axis's anchor is
// what shifts that reading onto the axis. A bare `rlo -= roff` would leave
// the two products, their sum, and the subtraction's OWN round-to-nearest
// error unaccounted for, on top of whatever aU/aV/dU/dV's own proven bounds
// already are — the identical gap verify.go's revolvePayloadProvesSimple
// exists to close one level up, only here it feeds the gate that decides
// which side the region is admitted on, not a review AFTER the fact. Reading
// the offset through proofbound.BoundedMul/proofbound.BoundedAdd/proofbound.BoundedSub (the same vocabulary
// axisFrame.toAxisRhoBound already uses for the identical ρ formula at one
// point) charges every one of those roundings into rloB/rhiB, so the strict
// admission gate below reads a number the accompanying bound provably covers.
//
// THE ADMISSION GATE ITSELF is the two-part repair CLAUDE.md's reject-only
// rule requires: proofbound.AdmitBelow(near, 0) reads whether the CHOSEN side's
// near-axis extreme is proven negative, proven non-negative, or neither, off
// the extreme's own proven interval — never off a tolerance.
//
//   - Proven negative (proofbound.SurvAdmit) refuses outright, however the interval got
//     that wide: a region that truly dips across the axis is a real defect,
//     not an artifact of the bound charging it wide.
//   - Proven non-negative (proofbound.SurvReject) needs no allowance, and that holds
//     even where the bound is nonzero — an interval whose own lower end
//     already clears zero commits nothing further. This is what keeps
//     radialAdmitAllow at exactly zero for every axis-aligned fixture: their
//     arithmetic is exact (a zero bound), so proofbound.AdmitBelow can only answer
//     proofbound.SurvReject or proofbound.SurvAdmit, never straddle, and a zero-bound interval that
//     is not proven negative IS proven non-negative.
//   - Neither (proofbound.SurvStraddle) is the genuine case a nonzero bound creates — a
//     tilted or offset axis whose own direction/anchor rounding leaves the
//     true radial minimum undecided. Admitting it is sound only because the
//     interval's own worst case is bounded (by the coarse ±tol classification
//     above, which already proved the chosen side's near extreme sits no
//     lower than −tol), and radialAdmitAllow carries exactly that worst case
//     forward to revolve_build.go, which charges it into every published
//     measurement the snap/unsnap mismatch can touch.
func ResolveSide(line Line2, readings SideExtremes) (SideResult, error) {
	nU, nV := -line.DV, line.DU
	rlo, rhi, rBound := readings.RLo, readings.RHi, readings.RBound
	zlo, zhi, zBound := readings.ZLo, readings.ZHi, readings.ZBound
	coordUpper := readings.CoordUpper
	// boundaryExtremesBoundedContext's own returned bound charges each
	// candidate's POSITIONAL uncertainty (a walked endpoint's own proven
	// displacement, a circular candidate's own enclosure) but NOT the
	// multiply-and-sum arithmetic of evaluating gu·u + gv·v itself for a
	// direction that is not exactly 0, 1 or −1 — the identical gap
	// axisExtremeContext closes for its own, structurally identical scan
	// through proofbound.PlaneDotDecompositionRoundAllow (internal/proofbound/bounds.go). A profile vertex
	// the record states verbatim therefore still commits real rounding
	// forming its dot with a tilted axis's own (nU, nV)/(dU, dV), and that
	// rounding is architecture-sensitive (FMA differs amd64/arm64):
	// left uncharged, the SAME recorded profile can compute as provably
	// negative on one platform and merely uncertain on another, for a region
	// whose true radial minimum is exactly zero. Folding it in here — once,
	// for THIS scan, never composed with axisExtremeContext's own copy of the
	// identical charge on a different reading — is what makes the admission
	// decision agree across platforms: the charge is zero for an axis-aligned
	// direction (proofbound.PlaneDotDecompositionRoundAllow's own trivial-coefficient
	// case), so it costs nothing for the tree's axis-aligned fixtures, and it
	// dominates a tilted axis's few-ulp discrepancy by orders of magnitude,
	// which turns a coin-flip sign into a proven straddle everywhere.
	rBound = proofbound.AbsSumUpper(rBound, proofbound.PlaneDotDecompositionRoundAllow(nU, nV, coordUpper))
	zBound = proofbound.AbsSumUpper(zBound, proofbound.PlaneDotDecompositionRoundAllow(line.DU, line.DV, coordUpper))

	roffB := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(nU, line.DVBound), proofbound.MeasuredScalar(line.AU, line.AUBound)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(nV, line.DUBound), proofbound.MeasuredScalar(line.AV, line.AVBound)),
	)
	zoffB := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(line.DU, line.DUBound), proofbound.MeasuredScalar(line.AU, line.AUBound)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(line.DV, line.DVBound), proofbound.MeasuredScalar(line.AV, line.AVBound)),
	)
	rloB := proofbound.BoundedSub(proofbound.MeasuredScalar(rlo, rBound), roffB)
	rhiB := proofbound.BoundedSub(proofbound.MeasuredScalar(rhi, rBound), roffB)
	zloB := proofbound.BoundedSub(proofbound.MeasuredScalar(zlo, zBound), zoffB)
	zhiB := proofbound.BoundedSub(proofbound.MeasuredScalar(zhi, zBound), zoffB)

	scale := math.Max(math.Max(math.Abs(rloB.Value), math.Abs(rhiB.Value)), math.Max(math.Abs(zloB.Value), math.Abs(zhiB.Value)))
	tol := 1e-9 * math.Max(1, scale)
	// Each side test is decided on the extreme's own proven interval: above
	// names "clear of the axis on the + side", below "clear on the − side",
	// and an interval spanning the threshold decides neither.
	hiAbove := proofbound.AdmitAbove(rhiB, tol)
	loBelow := proofbound.AdmitBelow(rloB, -tol)
	if hiAbove == proofbound.SurvStraddle || loBelow == proofbound.SurvStraddle {
		return SideResult{}, fmt.Errorf(`%w: the recorded region's radial extreme about this axis is known only to ±%v mm, which does not decide which side of the axis the region lies on`, decaderr.ErrDegenerate, math.Max(rloB.Bound, rhiB.Bound))
	}
	switch {
	case hiAbove == proofbound.SurvAdmit && loBelow == proofbound.SurvAdmit:
		return SideResult{}, fmt.Errorf(`%w: the revolve axis passes through the region`, decaderr.ErrDegenerate)
	case hiAbove == proofbound.SurvReject && loBelow == proofbound.SurvReject:
		return SideResult{}, fmt.Errorf(`%w: the region collapses onto the revolve axis`, decaderr.ErrDegenerate)
	}
	side := 1.0
	near := rloB
	if hiAbove == proofbound.SurvReject {
		side = -1
		near = proofbound.BoundedNeg(rhiB)
	}

	var radialAdmitAllow float64
	var radialProof bool
	switch proofbound.AdmitBelow(near, 0) {
	case proofbound.SurvAdmit:
		return SideResult{}, fmt.Errorf(`%w: the recorded region's radial minimum about this axis is proven negative, so the axis cuts through material`, decaderr.ErrDegenerate)
	case proofbound.SurvStraddle:
		radialAdmitAllow = math.Max(0, near.Bound-near.Value)
	case proofbound.SurvReject:
		// The positive-side reading is the same extreme and offset that
		// revolvePayloadProvesSimple would scan again. The build adds an
		// extra dot-product charge, so this proof is at least as strict.
		// A flipped axis keeps the old scan because its coefficient signs
		// and rounding path differ.
		radialProof = side > 0
	}

	axialExtent := proofbound.BoundedSub(zhiB, zloB)
	axialExtentUpper := proofbound.AbsSumUpper(axialExtent.Value, axialExtent.Bound)

	return SideResult{
		Frame: Frame{
			AU: line.AU, AV: line.AV,
			AUBound: line.AUBound, AVBound: line.AVBound,
			DU: side * line.DU, DV: side * line.DV,
			DUBound: line.DUBound, DVBound: line.DVBound,
			SnapTol: tol,
		},
		RadialAdmitAllow: radialAdmitAllow,
		RadialProof:      radialProof,
		AxialExtentUpper: axialExtentUpper,
		Side:             side,
	}, nil
}
