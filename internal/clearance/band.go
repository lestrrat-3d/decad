package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// TiltSine bounds the sine of the angle between the directions two float unit
// vectors stand for: |a × b| widened by AnalyticRoundBound(1), capped at 1.
// It is the tilt of every band reading (agent-docs/clearance-band-design.md
// §3).
func TiltSine(a, b r3.Vec) float64 {
	return math.Min(1, proofbound.AbsSumUpper(a.Cross(b).Len(), proofbound.AnalyticRoundBound(1)))
}

// TiltCosine is a lower bound on √(1 − s²), rounded down, for a tilt sine s
// from TiltSine.
func TiltCosine(s float64) float64 {
	sq := proofbound.ProductUpper(s, s)
	rest := freeform.DownRound(1 - sq)
	if !(rest > 0) {
		return 0
	}
	return freeform.DownRound(math.Sqrt(rest))
}

// AnnulusRegionMiss reports whether the closed annulus about (cx, cy) between
// radii rIn and rOut lies wholly outside the region by more than margin: the
// annulus form of CircleRegionHits' exclusion. The annulus is connected, so it
// lies outside the region when it meets no boundary element and one of its
// points classifies outside. The probe is the middle circle; every element of
// the region must lie farther from it than the annulus's half width plus
// margin, and one point of it must classify outside with that margin. margin
// carries the caller's tolerance and the float charge of the annulus's own
// centre and radii. A false answer proves nothing.
func AnnulusRegionMiss(r Region2, cx, cy, rIn, rOut, margin float64) bool {
	if proofbound.IsNonFinite(cx) || proofbound.IsNonFinite(cy) || proofbound.IsNonFinite(rIn) ||
		proofbound.IsNonFinite(rOut) || proofbound.IsNonFinite(margin) || !(rIn > 0) || rOut < rIn {
		return false
	}
	mid := (rIn + rOut) / 2
	half := proofbound.AbsSumUpper(rOut-mid, mid-rIn)
	probe := survey2d.SurveyElem{Kind: survey2d.SurveyArc, Qx: cx, Qy: cy, Rr: mid, Th0: 0, Th1: 2 * math.Pi, Closed: true}
	need := proofbound.AbsSumUpper(half, margin)
	for _, e := range r.Elems {
		if !(ElemElemDistLB(e, probe) > need) {
			return false
		}
	}
	return r.Classify(cx+mid, cy, margin) == -1
}
