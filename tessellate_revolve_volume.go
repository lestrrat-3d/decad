package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is docs/tessellation-design.md §13's increment T4
// (docs/tessellation-reach-design.md §6, R5): the OCCUPIED-VOLUME proof a
// revolve mesh must carry before any boolean may consume it. §§8-10 prove the
// mesh itself — where its facets sit and how much area they hold — and this
// file proves the one further thing a boolean needs, which is how much VOLUME
// the mesh and the body it stands for can differ by.
//
// §11 forbids the obvious shortcut outright: Mesh.Bound × held area is NOT that
// proof. A two-sided Hausdorff bound does not bound occupied volume, because a
// torus's inner and outer walls move in OPPOSITE material senses and a
// doubly-curved cell can gain material where another loses it — the signed
// error cancels while the symmetric difference does not. So §11 instead walks
// the four explicit stages between the analytic body and the returned
// polyhedron and charges each of them separately:
//
//	B0 → BM   the meridian arcs replaced by their chords     Mmeridian
//	BM → BH   the angular direction chorded                  Σ_cells Icell
//	BH → BC   coordinate-construction rounding               Mconstruct
//	BC → BR   final placement rounding                       Mround
//
// The set triangle inequality composes them, and every term is an ABSOLUTE
// swept volume, so nothing cancels between stages, between loops, or between a
// hole and the outline it sits in.
//
// Two structural facts make the middle term cheap:
//
//   - BM's meridian is a POLYLINE. Every circular generator has already been
//     replaced by its chords at this stage, so §11's straight-generator
//     homotopy H answers for EVERY cell of a sphere or a torus too; the
//     curvature it dropped is charged in full by Mmeridian, one stage earlier.
//   - The angular factor of Icell does not depend on the cell. Rotating a cell
//     about the axis is an isometry, so the same reading answers for every
//     angular interval, and separating the meridian direction out of the triple
//     integral (below) leaves a factor that depends on dφ ALONE. It is
//     therefore proven once per mesh rather than once per cell.

// revolveMeridianMoment is docs/tessellation-design.md §11's Mmeridian: the
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
// on ρ over them: tessellation.ChordSegmentArea already proves the first (with no trig call
// and no library ulp assumption), and the second is the largest ρ the walk's own
// endpoints and enclosed cardinal points reach — a sliver lies between its arc
// and the chord joining two points of that arc, so it reaches no farther from
// the axis than the arc does.
func revolveMeridianMoment(p *revolvePlan) float64 {
	total := 0.0
	for li, r := range p.resolved {
		for k, w := range r.walks {
			if !w.IsCircular() {
				continue
			}
			area := tessellation.ChordSegmentArea(w.Radius, math.Abs(w.Th1-w.Th0), p.counts[li][k])
			rho := 0.0
			for _, pt := range revolveWalkExtremes(w.SegmentWalk) {
				rho = math.Max(rho, pt[1])
			}
			total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(area, rho))
		}
	}
	if total == 0 {
		return 0
	}
	return proofbound.ProductUpper(revolveSweepUpper(p), total)
}

// revolveSweepUpper is an upward-rounded bound on the swept angle Mmeridian
// multiplies by. A full turn reads the in-tree π bracket rather than
// math.Pi — the constant is the nearest float to π and may sit below it, which
// is the wrong side for a bound — and a partial sweep rounds its own float
// subtraction up by one ulp, which covers that subtraction's whole rounding.
func revolveSweepUpper(p *revolvePlan) float64 {
	if p.rp.full {
		return proofbound.RatFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper))
	}
	return proofbound.UpRound(p.sweep)
}

// revolveSymDiff composes docs/tessellation-design.md §11's four stages into the
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
func revolveSymDiff(m *Mesh, p *revolvePlan, angular *big.Rat, deltaC, deltaR float64) (float64, error) {
	if angular == nil || angular.Sign() < 0 {
		return 0, tessellation.ErrRevolveAngularHomotopy
	}
	coord := proofbound.AbsSumUpper(deltaC, deltaR)
	area := proofbound.PerturbedAreaUpper(m.vertices, m.triangles, coord)
	sym := proofbound.AbsSumUpper(
		revolveMeridianMoment(p),
		proofbound.RatFloatUp(angular),
		proofbound.SweptVolumeAllow(deltaC, area),
		proofbound.SweptVolumeAllow(deltaR, area),
	)
	if proofbound.IsNonFinite(sym) || sym < 0 {
		return 0, fmt.Errorf(`%w: this revolve mesh states no finite bound on the volume it and the body it stands for differ by`, ErrUnsupported)
	}
	return proofbound.UpRound(sym), nil
}
