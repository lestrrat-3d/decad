package capband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Closure bounds slivers between a cap band's integrated patches and its
// recorded side and cap faces.
type Closure struct {
	Rim, SideLevel, CapLevel, Reach float64
}

func (c Closure) Zero() bool { return c == Closure{} }

// ClosureOf measures the slivers from held walks, joins and corner rulings.
// Slants contain the held edge lengths; slant bounds cover their arithmetic.
func ClosureOf(walks []survey2d.SideWalk, joins []capcontour.Join,
	slantIn, slantOut, slantInHeld, slantOutHeld []float64) (Closure, bool) {
	n := len(walks)
	var out Closure
	sideGap := make([]float64, n)
	capSpread := make([]float64, n)
	for i, w := range walks {
		if !w.IsCircular() {
			bS, bE := proofbound.WalkEndBoundAllow(w.StartBound), proofbound.WalkEndBoundAllow(w.EndBound)
			if proofbound.IsNonFinite(bS) || proofbound.IsNonFinite(bE) {
				return Closure{}, false
			}
			if b := proofbound.AbsSumUpper(bS, bE); b > 0 {
				out.SideLevel = proofbound.AbsSumUpper(out.SideLevel,
					proofbound.ProductUpper(proofbound.AbsSumUpper(w.Length, w.LengthBound, b), b))
			}
			continue
		}
		var gap float64
		var ok bool
		if (w.StartBound == proofbound.WalkEndBound{}) && (w.EndBound == proofbound.WalkEndBound{}) {
			_, gap, ok = RadiusAllow(w.CU, w.CV, 0, Point{U: w.StartU, V: w.StartV}, Point{U: w.EndU, V: w.EndV})
		} else {
			gap, ok = capcontour.CircularWalkEndGap(w)
		}
		j0, j1 := joins[i], joins[(i+1)%n]
		start, end := j0.M, j1.M
		if j0.Arc {
			start = j0.PB
		}
		if j1.Arc {
			end = j1.PA
		}
		_, spread, okS := RadiusAllow(w.CU, w.CV, 0, start, end)
		if !ok || !okS {
			return Closure{}, false
		}
		sideGap[i], capSpread[i] = gap, spread
	}
	for i := range n {
		pi := (i + n - 1) % n
		prev, cur := walks[pi], walks[i]
		j := joins[i]
		du := new(big.Rat).Sub(proofarith.FloatRat(prev.EndU), proofarith.FloatRat(cur.StartU))
		dv := new(big.Rat).Sub(proofarith.FloatRat(prev.EndV), proofarith.FloatRat(cur.StartV))
		gap := proofbound.RatFloatUp(new(big.Rat).Add(du.Abs(du), dv.Abs(dv)))
		if prev.IsCircular() {
			gap = proofbound.AbsSumUpper(gap, proofbound.WalkEndBoundAllow(prev.EndBound), sideGap[pi], capSpread[pi])
		}
		if cur.IsCircular() {
			gap = proofbound.AbsSumUpper(gap, proofbound.WalkEndBoundAllow(cur.StartBound), sideGap[i], capSpread[i])
		}
		rulings := []float64{proofbound.AbsSumUpper(slantIn[i], slantInHeld[i])}
		if j.Arc {
			_, spread, ok := RadiusAllow(j.VU, j.VV, 0, j.PA, j.PB)
			if !ok {
				return Closure{}, false
			}
			gap = proofbound.AbsSumUpper(gap, spread)
			rulings = append(rulings, proofbound.AbsSumUpper(slantOut[i], slantOutHeld[i]))
		}
		if proofbound.IsNonFinite(gap) {
			return Closure{}, false
		}
		if gap == 0 {
			continue
		}
		for _, slant := range rulings {
			out.Rim = proofbound.AbsSumUpper(out.Rim,
				proofbound.ProductUpper(proofbound.AbsSumUpper(slant, gap), gap))
		}
		corner := proofbound.ProductUpper(gap, gap)
		out.SideLevel = proofbound.AbsSumUpper(out.SideLevel, corner)
		out.CapLevel = proofbound.AbsSumUpper(out.CapLevel, corner)
		out.Reach = math.Max(out.Reach, gap)
	}
	return out, true
}

// FluxAllow bounds a closure's contribution to three times the band's volume.
func (c Closure) FluxAllow(pointUpper, sideZUpper, capZUpper float64) float64 {
	return proofbound.AbsSumUpper(
		proofbound.ProductUpper(pointUpper, c.Rim),
		proofbound.ProductUpper(sideZUpper, c.SideLevel),
		proofbound.ProductUpper(capZUpper, c.CapLevel),
	)
}

// MomentAllow bounds each first-moment flux across the closure.
func (c Closure) MomentAllow(pointUpper, sideZUpper, capZUpper float64) (float64, float64) {
	half := func(x float64) float64 { return proofbound.ProductUpper(0.5, proofbound.ProductUpper(x, x)) }
	inPlane := proofbound.ProductUpper(half(pointUpper), c.Rim)
	axial := proofbound.AbsSumUpper(inPlane,
		proofbound.ProductUpper(half(sideZUpper), c.SideLevel),
		proofbound.ProductUpper(half(capZUpper), c.CapLevel))
	return inPlane, axial
}
