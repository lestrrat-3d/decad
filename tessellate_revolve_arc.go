package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
)

// This file is docs/tessellation-design.md §13's increment T3
// (docs/tessellation-reach-design.md §6, R4): the CIRCULAR meridian generator —
// a sphere or a torus wall — as an extension of the straight-generator cells
// tessellate_revolve.go assembles and tessellate_revolve_proof.go proves.
//
// It owns three things a straight generator never needs:
//
//   - The meridian STATIONS. A circular walk is chorded along the meridian, so
//     the mesh carries samples between its two recorded junctions. Each one is
//     enclosed at the recorded parameter it denotes and STORED as the float
//     nearest that enclosure's midpoint — never a math.Cos/math.Sin evaluation,
//     which Go states no ulp contract for — so the station's construction gap
//     is a fact about this build rather than an assumption about a library.
//   - The cell's own §10.2 Ecell. A straight generator's true area density is
//     affine in the meridian parameter, which is what let R3 decompose the sign
//     in closed form (tess §15). A circular generator's is not: with the arc
//     matched to its chord by the shared parameter, ρ(t) = cV + r·sin(θ0 + t·Δθ)
//     and the difference against the flat facet's constant density has no
//     rational root. So this file takes tess §15's OTHER admissible path,
//     CERTIFIED INTERVAL SUBDIVISION under a fixed budget: the unit interval is
//     cut into a fixed number of pieces, every node encloses the integrand
//     through a certified radian sine enclosure, and each piece contributes an
//     outward-rounded ABSOLUTE integral bound built from its own two nodes plus
//     a proven second-order allowance. The absolute value is inside the sum, so
//     nothing cancels between pieces and a later boolean retaining one sign
//     lobe is still bounded by the whole.
//   - The proven circular-segment area a partial cap's curved trim omits.
//
// Nothing here samples anything to decide anything, and nothing here calls a
// library trig function to publish a bound.

// revolveArcStation is one interior meridian sample of a circular walk: the
// axis coordinates the RECORD denotes at station k of n, enclosed exactly, and
// the float pair this mesh stores for it.
//
// The station's recorded parameter is TStart + (k/n)·(TEnd − TStart), taken as
// an EXACT rational — rounding it to a float first would enclose the recorded
// curve at a neighbouring parameter and prove a bound about a point this
// chording never named (chordStationBound's own rule). The plane-local
// enclosure is circularEndpointInterval's, and revolvemesh.AxisCoordInterval carries it
// into (z, ρ) through the payload's own axis frame with no rounding.
//
// The stored pair is the float NEAREST the enclosure's midpoint. That is what
// makes the station's construction gap a fact about this build: it is at most
// the enclosure's own width plus half an ulp, both of which the tolerance split
// reserved a-priori (revolveStationGapPrior) and the caller measures and holds
// to account. Nothing here calls math.Cos or math.Sin.
//
// A station the record cannot enclose refuses. A station the arithmetic puts
// ON or BEYOND the axis is ErrDegenerate rather than a pole: a generator that
// meets the axis at an interior point sweeps no manifold solid, and §12 forbids
// rounding a near-axis ring onto the axis to make one.
func revolveArcStation(ax axisFrame, seg CurveSegment, k, n int) (revolvemesh.RevMeridian, float64, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return revolvemesh.RevMeridian{}, 0, err
	}
	start, span, ok := circularSegmentRange(seg)
	if !ok || n <= 0 || k <= 0 || k >= n {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	frac := new(big.Rat).SetFrac64(int64(k), int64(n))
	rt := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
	uIv, vIv, ok := circularEndpointInterval(seg, rt)
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	zIv, rhoIv, ok := revolvemesh.AxisCoordInterval(ax.aU, ax.aV, ax.dU, ax.dV, uIv, vIv)
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	z, _ := intervalMid(zIv).Float64()
	rho, _ := intervalMid(rhoIv).Float64()
	gap := math.Max(proofbound.IntervalFloatError(zIv, z), proofbound.IntervalFloatError(rhoIv, rho))
	if proofbound.IsNonFinite(z) || proofbound.IsNonFinite(rho) || proofbound.IsNonFinite(gap) {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	if rho <= 0 {
		return revolvemesh.RevMeridian{}, 0, fmt.Errorf(`%w: a revolve meridian chord station lands on the axis or across it, so the recorded generator sweeps no manifold solid there`, ErrDegenerate)
	}
	return revolvemesh.RevMeridian{Z: z, Rho: rho, ZIv: zIv, RhoIv: rhoIv}, gap, nil
}
