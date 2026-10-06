package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/cappatch"
	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/modify-reach-design.md §8.4's closed-form first moments
// for the cap-loop chamfer payload — the fix for A8: evalCapBlendContext used
// to publish a centroid that was an area-weighted average of each face's own
// first-loop-start VERTEX (capBlendCentroidEstimate, gone), bounded only by
// the body's own bounding box. That was not a centroid; it was a stand-in
// with no formula behind it at all, wrong by 10 mm on a cylinder 21.5 mm
// across. This file computes the real thing: M = ∫ p dV in the payload's own
// plane-local (u, v, z) coordinates — the same frame capBandVolume already
// works in, valid because a rigid map preserves volume and transforms the
// moment linearly — decomposed exactly the way the volume already is,
// M = Σ_loops sign_li · [M_slab(li) + M_band(li, start) + M_band(li, end)],
// with sign_li the SAME per-loop sign evalCapBlendContext already applies to
// slabVolume/bandVolume. capBlendCentroidGeometryBound (this file, the second
// half of the old estimate) remains the geometric safety net every analytic
// centroid falls back to, now a CEILING on the formula answer via math.Min,
// never the whole bound.
//
// The slab term is shell_cup.go's loopEnclosedMomentsContext (a signed first
// moment sibling of loopEnclosedAreaContext) times the straight height, with
// the z component the elementary A·h·(zLo+zHi)/2. The band term is the
// divergence theorem with F = (u²/2, 0, 0), (0, v²/2, 0), (0, 0, z²/2) over
// the SAME closed sub-solid capBandVolume already integrates (the two flat
// disks plus the patches): patchFirstMomentFlux is patchRawFlux's per-axis
// sibling, exact rationally for a Plane patch (exactPlanePatchMoment) and a
// closed-form Fourier sum for a Cone/apex/whole-turn one
// (coneMomentTermsX/Y/Z), derived by computer algebra and verified against a
// fine numerical double integral of the raw flux integrand and against this
// package's own shipped volume formula (both reproduced independently, never
// merely asserted) — see capBandMoment's own doc for the disk/patch sign
// composition, identical to capBandVolume's.

func patchFirstMomentFlux(g capPatchGeom) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar) {
	return cappatch.FirstMomentFlux(g.patch())
}

type phaseTerm = cappatch.PhaseTerm

func coneMomentTermsX(R0, R1, H, cU, dS, dC *big.Rat) []phaseTerm {
	return cappatch.ConeMomentTermsX(R0, R1, H, cU, dS, dC)
}

func coneMomentTermsY(R0, R1, H, cV, dS, dC *big.Rat) []phaseTerm {
	return cappatch.ConeMomentTermsY(R0, R1, H, cV, dS, dC)
}

func coneMomentTermsZ(R0, R1, H, z0, dS, dC *big.Rat) []phaseTerm {
	return cappatch.ConeMomentTermsZ(R0, R1, H, z0, dS, dC)
}

// loopCoordinateUpper is one loop's own coordinate envelope
// (profileCoordinateUpper, extrude.go, wrapped as a single-outer-loop
// ProfileRecord) — proofbound.SweptMomentAllow's coordUpper input, one dimension's worth
// of the SAME envelope prismCentroidGeometryBound already forms for a whole
// profile.
func loopCoordinateUpper(loop LoopRecord, work *freeform.FreeformWork) (float64, error) {
	return profileCoordinateUpper(ProfileRecord{Outer: loop}, work, nil)
}

// capBandMoment is one loop's chamfer-band first-moment contribution — Mx,
// My, Mz, the divergence-theorem flux of F = (u²/2, 0, 0), (0, v²/2, 0),
// (0, 0, z²/2) — over the SAME closed sub-solid capBandVolume integrates: the
// two flat disks (the loop's own enclosed area at capZ and at sideZ) plus the
// patches (buildCapBand's geom).
//
// The two disks are FLAT (constant z), so their outward normal is ±ẑ:
// n_u = n_v = 0 everywhere on them, and Green's theorem gives them NO Mu/Mv
// contribution at all — only Mz, at (level²/2)·(±area), the divergence
// theorem's own reduction of capBandVolume's (level)·(±area) volume term one
// power higher (docs/modify-reach-design.md §8.4). Every patch, in contrast,
// is built from the loop's OWN walk, so the SAME -matSign·orient sign
// correction capBandVolume applies to its own patchRawFlux sum applies here
// (§8.4's "Signs"): -matSign covers the AXIAL half, orient — the loop's own
// signed-area sign — the IN-PLANE half.
//
// The cap contour's own displacement (delta) is composed ONCE, after the
// sum, via internal/proofbound/bounds.go's proofbound.SweptMomentAllow — never inside a disk term or inside
// patchFirstMomentFlux itself, both of which already read the SAME displaced
// cap-level coordinates and would double the charge (capBandVolume's
// identical rule for proofbound.SweptVolumeAllow). areaUpper is the same surface the
// contour's displacement acted on (this band's patches plus the cap disk
// they close on); coordUpper is the band's own coordinate envelope — the
// ORIGINAL loop's (loopCoordinateUpper) AND the built cap boundary's
// (capLoopBoundary, widened by delta since that boundary is itself only
// known to within delta of the one it denotes), plus the two axial levels.
// The band's material lies between the two loops, so a bound taken from the
// original loop alone can fall short wherever the offset moves a coordinate
// OUTWARD — capArea's own boundary is exactly that displaced coordinate set,
// and proofbound.SweptMomentAllow's own contract (internal/proofbound/bounds.go) requires coordUpper to
// bound every point the difference volume can hold.
func capBandMoment(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign, delta float64, work *freeform.FreeformWork) (mu, mv, mz proofbound.BoundedScalar, err error) {
	capZB := cbp.capBandLevel(capZ, matSign)
	sideZB := proofbound.BoundedAdd(capZB, proofbound.MeasuredScalar(matSign*cbp.d, cbp.dDelta))
	sideZ := sideZB.Value

	signedArea, err := loopSignedAreaBudget(proofbound.NewWorkBudget(ctx), loop)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}

	sideArea, err := loopEnclosedAreaContext(ctx, loop)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	capBoundary, err := capLoopBoundary(ctx, loop, cbp.d)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	capArea, err := loopEnclosedAreaContext(ctx, capBoundary)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}

	half := proofbound.ExactScalar(0.5)
	capZTerm := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.BoundedMul(capZB, capZB), half), proofbound.BoundedMul(proofbound.ExactScalar(-matSign), capArea))
	sideZTerm := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.BoundedMul(sideZB, sideZB), half), proofbound.BoundedMul(proofbound.ExactScalar(matSign), sideArea))
	muTotal := proofbound.BoundedScalar{}
	mvTotal := proofbound.BoundedScalar{}
	mzTotal := proofbound.BoundedAdd(capZTerm, sideZTerm)

	patchAreaTotal := proofbound.BoundedScalar{}
	for _, g := range geom {
		pmu, pmv, pmz := patchFirstMomentFlux(g)
		sign := -matSign * orient
		muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*pmu.Value, pmu.Bound))
		mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*pmv.Value, pmv.Bound))
		mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*pmz.Value, pmz.Bound))
		pa, pb := patchAreaOf(g)
		patchAreaTotal = proofbound.BoundedAdd(patchAreaTotal, proofbound.MeasuredScalar(pa, pb))
	}

	if delta > 0 {
		areaUpper := proofbound.AbsSumUpper(patchAreaTotal.Value, patchAreaTotal.Bound, capArea.Value, capArea.Bound)
		coordUpper, cerr := loopCoordinateUpper(loop, work)
		if cerr != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, cerr
		}
		capCoordUpper, cerr := loopCoordinateUpper(capBoundary, work)
		if cerr != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, cerr
		}
		coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(capCoordUpper, delta))
		coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(sideZ), sideZB.Bound))
		coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(capZ), capZB.Bound))
		allow := proofbound.SweptMomentAllow(delta, areaUpper, coordUpper)
		muTotal.Bound = proofbound.AbsSumUpper(muTotal.Bound, allow)
		mvTotal.Bound = proofbound.AbsSumUpper(mvTotal.Bound, allow)
		mzTotal.Bound = proofbound.AbsSumUpper(mzTotal.Bound, allow)
	}
	return muTotal, mvTotal, mzTotal, nil
}

// capBlendCentroidGeometryBound is the geometric safety net every analytic
// centroid falls back to: the true centroid of a bounded solid lies within
// its own bounding box, so |estimate-true| is bounded by the box's own reach
// from the estimate — sound whatever the estimate's own accuracy. It is now a
// CEILING on the closed-form first-moment answer (evalCapBlendContext's
// math.Min), never published on its own: the old capBlendCentroidEstimate's
// face-average estimate is gone (docs/modify-reach-design.md §8.4's
// closed-form first moments replace it), and faceRepresentativePoint had no
// other caller and is gone with it.
//
// The reach is maximized over all EIGHT corners of the box, and that is the
// whole of the proof rather than a thoroughness flourish. p -> |p - estimate|
// is convex, so its maximum over the box — a convex hull of its eight
// corners — is attained AT a corner; taking the max over all eight therefore
// bounds the distance to every point the box holds, the true centroid among
// them, wherever the estimate itself sits. Reading only Min and Max leaves
// six corners unexamined, and a box whose extent along one axis is far larger
// than along another puts its farthest corner among exactly those six: the
// reported bound would then be smaller than the estimate's own error and
// enclose nothing.
//
// The box's own Bound is added on top for the same reason: the safety net is
// "the true centroid lies within the box", and where a face of the box is
// itself known only to a displacement, the box that provably contains the
// body is the reported one widened by it.
func capBlendCentroidGeometryBound(estimate r3.Vec, bounds Box) float64 {
	xs := [2]float64{bounds.Min.X, bounds.Max.X}
	ys := [2]float64{bounds.Min.Y, bounds.Max.Y}
	zs := [2]float64{bounds.Min.Z, bounds.Max.Z}
	reach := 0.0
	for _, x := range xs {
		for _, y := range ys {
			for _, z := range zs {
				dd := r3.NewVec(x, y, z).Sub(estimate).Len()
				if dd > reach {
					reach = dd
				}
			}
		}
	}
	return proofbound.AbsSumUpper(reach, bounds.Bound.Mag())
}
