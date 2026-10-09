package capband

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ErrPatchSkewUnbounded refuses a circular patch whose directrices turn a
// quarter turn or more apart at a corner, beyond the proven area bound.
var ErrPatchSkewUnbounded = fmt.Errorf(`%w: a cap-loop chamfer's circular band patch turns its cap contour a quarter turn or more from its side wall at a corner, and this evaluator proves no area bound for that ruled patch`, decaderr.ErrUnsupported)

// SetPatchSkews records the corner skews of a circular wall patch on the
// branch its held windows name.
func SetPatchSkews(g *Patch, side0, side1, cap0, cap1 Point) error {
	c0, c1 := WindowOnBranch(g.CapTh0, g.CapTh1, g.Th0)
	s0, ok0 := CornerSkewUpper(g.CU, g.CV, side0, cap0, c0-g.Th0)
	s1, ok1 := CornerSkewUpper(g.CU, g.CV, side1, cap1, c1-g.Th1)
	if !ok0 || !ok1 {
		return ErrPatchSkewUnbounded
	}
	g.SkewStart, g.SkewEnd = s0, s1
	return nil
}

// SetPatchLocusSpans records circular wall i's two corner-foot loci. A
// rounded setback or an unbounded corner flux leaves the spans unset.
func SetPatchLocusSpans(budget *proofbound.WorkBudget, g *Patch, walks []survey2d.SideWalk,
	joins []offset2d.Join, i int, dc, dcDelta float64, swapped bool) error {
	if dcDelta != 0 || !(g.CornerFlux < math.Inf(1)) {
		return nil
	}
	n := len(walks)
	w := walks[i]
	spansAt := func(k int, sideU, sideV float64) ([]LocusSpan, error) {
		j := joins[k]
		if j.Arc || j.G1 {
			return StraightLocusSpans(dc), nil
		}
		spans, ok, err := CornerLocusSpans(budget, walks[(k+n-1)%n], walks[k],
			w.CU, w.CV, sideU, sideV, j.VertU, j.VertV, dc)
		if err != nil || !ok {
			return nil, err
		}
		return spans, nil
	}
	start, err := spansAt(i, w.StartU, w.StartV)
	if err != nil {
		return err
	}
	end, err := spansAt((i+1)%n, w.EndU, w.EndV)
	if err != nil {
		return err
	}
	if start == nil || end == nil {
		return nil
	}
	if swapped {
		start, end = end, start
	}
	g.Locus0, g.Locus1, g.LocusSetback = start, end, dc
	return nil
}

// PatchCornerFlux sums circular wall i's two corner-sliver fluxes. An
// unbounded sliver makes the returned bound positive infinity.
func PatchCornerFlux(budget *proofbound.WorkBudget, walks []survey2d.SideWalk,
	joins []offset2d.Join, i int, axial, dc, dcDelta float64) (float64, error) {
	n := len(walks)
	w := walks[i]
	total := 0.0
	for _, k := range [2]int{i, (i + 1) % n} {
		j := joins[k]
		if j.Arc || j.G1 {
			continue
		}
		prev, cur := walks[(k+n-1)%n], walks[k]
		flux, ok, err := MiterLocusSliverFlux(budget, prev, cur, w.CU, w.CV,
			j.VertU, j.VertV, axial, dc, dcDelta)
		if err != nil {
			return 0, err
		}
		if !ok {
			return math.Inf(1), nil
		}
		total = proofbound.AbsSumUpper(total, flux)
	}
	return total, nil
}

// BandLevelDelta bounds the side level's setback conversion and float sum.
func BandLevelDelta(capZ, matSign, ds, dsDelta float64) float64 {
	sideZ := capZ + matSign*ds
	return proofbound.AbsSumUpper(dsDelta, proofarith.AddRoundError(capZ, matSign*ds, sideZ))
}

// CornerGap bounds the held corner's distance from a walk end it denotes.
func CornerGap(vU, vV, u, v float64, bound proofbound.WalkEndBound) float64 {
	a, okA := proofarith.DyOf(vU)
	b, okB := proofarith.DyOf(vV)
	c, okC := proofarith.DyOf(u)
	d, okD := proofarith.DyOf(v)
	if !okA || !okB || !okC || !okD {
		return math.Inf(1)
	}
	x, y := proofarith.DySubScalar(a, c), proofarith.DySubScalar(b, d)
	gap := proofarith.DySqrtUp(proofarith.DyAdd(proofarith.DyMul(x, x), proofarith.DyMul(y, y)))
	return proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(bound), gap)
}
