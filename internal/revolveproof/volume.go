package revolveproof

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// MeridianMoment is docs/tessellation-design.md §11's Mmeridian: the
// volume between the analytic body B0 and the body BM whose every circular
// meridian subarc has been replaced by its chord.
//
//	Mmeridian = sweepAngle · Σ_c |∫_{S_c} ρ dA|
//
// The two bodies differ, in the (z, ρ) half plane, by exactly the circular
// segments S_c between each arc and its chords, and revolving a region of that
// half plane through an angle Φ occupies Φ·∫ ρ dA of volume — Pappus, valid
// because ρ ≥ 0 keeps the region off the far side of the axis. The absolute
// value is taken per sliver, so a hole that GAINS material is charged the same
// sign as an outline that loses it and nothing cancels across loops.
//
// Each walk's slivers are bounded by their own total area times an upper bound
// on ρ over them: revolvemesh.ChordSegmentArea already proves the first (with no trig call
// and no library ulp assumption), and the second is the largest ρ the walk's own
// endpoints and enclosed cardinal points reach — a sliver lies between its arc
// and the chord joining two points of that arc, so it reaches no farther from
// the axis than the arc does.
func MeridianMoment(walks WalkLoops, counts [][]int, sweep float64, full bool) float64 {
	total := 0.0
	for li := range walks.Len() {
		r := walks.Walks(li)
		for k, w := range r {
			if !w.IsCircular() {
				continue
			}
			area := revolvemesh.ChordSegmentArea(w.Radius, math.Abs(w.Th1-w.Th0), counts[li][k])
			rho := 0.0
			for _, pt := range WalkExtremes(w.SegmentWalk) {
				rho = math.Max(rho, pt[1])
			}
			total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(area, rho))
		}
	}
	if total == 0 {
		return 0
	}
	return proofbound.ProductUpper(SweepUpper(sweep, full), total)
}

// SweepUpper is an upward-rounded bound on the swept angle Mmeridian
// multiplies by. A full turn reads the in-tree π bracket rather than
// math.Pi — the constant is the nearest float to π and may sit below it, which
// is the wrong side for a bound — and a partial sweep rounds its own float
// subtraction up by one ulp, which covers that subtraction's whole rounding.
func SweepUpper(sweep float64, full bool) float64 {
	if full {
		return proofbound.RatFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper))
	}
	return proofbound.UpRound(sweep)
}

// SymDiff composes docs/tessellation-design.md §11's four stages into the
// mesh's volSymDiff.
//
//	volSymDiff = proofbound.UpRound(Mmeridian + Σ_cells Icell + Mconstruct + Mround)
//
// angular is the already-summed Σ_cells Icell, in exact rationals, so the one
// float rounding it takes is the conversion here. The two coordinate stages are
// swept-volume allowances in the shape §11 names: a boundary point moving at
// speed at most delta can displace volume no faster than the area it sweeps
// allows, and proofbound.PerturbedAreaUpper covers every surface on the stage's path, not
// only its two ends. Both stages read the composed displacement as their area
// argument because BH sits within deltaC + deltaR of the returned mesh and BC
// within deltaR of it, so one area bound at the composed figure covers every
// intermediate surface of both paths.
//
// Every leg is an absolute occupied-volume charge; none of them may cancel
// another, which is why they compose through proofbound.AbsSumUpper rather than a signed
// sum.
func SymDiff(vertices []r3.Vec, triangles [][3]int, walks WalkLoops, counts [][]int,
	sweep float64, full bool, freeformMeridian float64, angular *big.Rat, deltaC, deltaR float64) (float64, error) {
	if angular == nil || angular.Sign() < 0 {
		return 0, revolvemesh.ErrRevolveAngularHomotopy
	}
	coord := proofbound.AbsSumUpper(deltaC, deltaR)
	area := proofbound.PerturbedAreaUpper(vertices, triangles, coord)
	sym := proofbound.AbsSumUpper(
		MeridianMoment(walks, counts, sweep, full), freeformMeridian,
		proofbound.RatFloatUp(angular),
		proofbound.SweptVolumeAllow(deltaC, area),
		proofbound.SweptVolumeAllow(deltaR, area),
	)
	if proofbound.IsNonFinite(sym) || sym < 0 {
		return 0, fmt.Errorf(`%w: this revolve mesh states no finite bound on the volume it and the body it stands for differ by`, decaderr.ErrUnsupported)
	}
	return proofbound.UpRound(sym), nil
}
