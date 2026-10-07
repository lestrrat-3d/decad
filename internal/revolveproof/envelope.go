// Package revolveproof computes mesh envelopes, preflight budgets, cell
// area slack and occupied-volume bounds for revolved bodies.
package revolveproof

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// WalkLoops reads a revolve section's resolved walks in their original order.
type WalkLoops interface {
	Len() int
	Walks(i int) []survey2d.SideWalk
}

// Extents is docs/tessellation-design.md §8's ρ and |z| envelope over
// every loop, read from the WALKS rather than from a chording of them: a
// straight generator attains both at its endpoints, and a circular one at its
// endpoints plus every cardinal point its own parameter interval contains. A
// cardinal point needs no trig — the four of them are (cU ± r, cV) and
// (cU, cV ± r) exactly — so this envelope carries no library assumption.
//
// A section with no material off the axis is an invariant failure the builder's
// own area gate already refuses.
func Extents(loops WalkLoops) (float64, float64, error) {
	rhoMax, zAbsMax := 0.0, 0.0
	see := func(z, rho float64) error {
		if proofbound.IsNonFinite(rho) || proofbound.IsNonFinite(z) {
			return fmt.Errorf(`%w: a revolve meridian sample is not finite`, decaderr.ErrUnsupported)
		}
		rhoMax = math.Max(rhoMax, rho)
		zAbsMax = math.Max(zAbsMax, math.Abs(z))
		return nil
	}
	for i := range loops.Len() {
		r := loops.Walks(i)
		for _, w := range r {
			for _, p := range WalkExtremes(w.SegmentWalk) {
				if err := see(p[0], p[1]); err != nil {
					return 0, 0, err
				}
			}
		}
	}
	if rhoMax <= 0 {
		return 0, 0, fmt.Errorf(`%w: the recorded region lies entirely on the revolve axis, so it sweeps no solid`, decaderr.ErrDegenerate)
	}
	return rhoMax, zAbsMax, nil
}

// WalkExtremes lists the (z, ρ) points where one walk can attain either
// envelope: its two endpoints, plus, for a circular walk, each cardinal point
// its own angular interval contains.
func WalkExtremes(w survey2d.SegmentWalk) [][2]float64 {
	out := [][2]float64{{w.StartU, w.StartV}, {w.EndU, w.EndV}}
	if !w.IsCircular() {
		return out
	}
	span := math.Abs(w.Th1 - w.Th0)
	lo := math.Min(w.Th0, w.Th1)
	cardinals := [4][2]float64{
		{w.CU + w.Radius, w.CV},
		{w.CU, w.CV + w.Radius},
		{w.CU - w.Radius, w.CV},
		{w.CU, w.CV - w.Radius},
	}
	for q, p := range cardinals {
		// The cardinal's own angle is q·π/2; shift it into [lo, lo+2π) and keep
		// it when the walk's interval reaches that far.
		d := math.Mod(float64(q)*math.Pi/2-lo, 2*math.Pi)
		if d < 0 {
			d += 2 * math.Pi
		}
		if d <= span {
			out = append(out, p)
		}
	}
	return out
}

// LoopMaxSagitta reduces the per-chord sagittas to the per-loop figure the
// cross-loop clearance gate reads.
func LoopMaxSagitta(sag [][]float64) []float64 {
	out := make([]float64, len(sag))
	for i, loop := range sag {
		for _, s := range loop {
			out[i] = math.Max(out[i], s)
		}
	}
	return out
}
