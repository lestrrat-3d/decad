package rimextension

import (
	"context"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PolarSource gives the denoted circle and the two directed endpoint rays of a
// native convex minor arc. Heights are the exact world Y values of the prism.
type PolarSource struct {
	EndClosure                                   float64
	CenterX, CenterZ, RadiusSquared, Bottom, Top *big.Rat
	Rays                                         [2][2]*big.Rat
}

func ReadPolarSource(ctx context.Context, f BoundFace, m *proofbound.ReplayBudget) (PolarSource, bool, error) {
	var none PolarSource
	if m == nil || f.Final == nil || f.Final.Reversed || f.Translation != (r3.Vec{}) {
		return none, false, nil
	}
	p, ok, err := windowPrism(ctx, f, m)
	if err != nil || !ok {
		return none, false, err
	}
	for i, seg := range p.Profile.Outer.Segments {
		if !windowRole(f, "side(0,"+strconv.Itoa(i)+")") {
			continue
		}
		a, ok := seg.(momentinput.ArcSeg)
		if !ok || a.TStart != 0 || a.TEnd != 1 {
			return none, false, nil
		}
		x, z := scalar(a.Center.U), scalar(a.Center.V)
		sx, sz, ex, ez := scalar(a.Start.U), scalar(a.Start.V), scalar(a.End.U), scalar(a.End.V)
		if x == nil || z == nil || sx == nil || sz == nil || ex == nil || ez == nil {
			return none, false, nil
		}
		sx, sz, ex, ez = minus(sx, x), minus(sz, z), minus(ex, x), minus(ez, z)
		if minus(times(sx, ez), times(sz, ex)).Sign() <= 0 {
			return none, false, nil
		}
		next := p.Profile.Outer.Segments[(i+1)%len(p.Profile.Outer.Segments)]
		line, ok := next.(momentinput.LineSeg)
		if !ok || !(line.TStart == 0 && line.TEnd == 1 || line.TStart == 1 && line.TEnd == 0) {
			return none, false, nil
		}
		join := line.Start
		if line.TStart == 1 {
			join = line.End
		}
		jx, jz := scalar(join.U), scalar(join.V)
		if jx == nil || jz == nil {
			return none, false, nil
		}
		r2 := plus(times(sx, sx), times(sz, sz))
		closure, ok := arcEndClosure(r2, ex, ez, minus(jx, x), minus(jz, z))
		if !ok {
			return none, false, nil
		}
		first, last := p.DenotedZ0(), p.DenotedZ1()
		if first == nil || last == nil || first.Cmp(last) == 0 {
			return none, false, nil
		}
		o := p.Frame.Origin()
		y0, y1 := minus(scalar(o.Y), first), minus(scalar(o.Y), last)
		return PolarSource{closure, plus(scalar(o.X), x), plus(scalar(o.Z), z), r2, wmin(y0, y1), wmax(y0, y1), [2][2]*big.Rat{{sx, sz}, {ex, ez}}}, true, nil
	}
	return none, false, nil
}

// The last facet of the arc walk runs to the following walk's start. A
// radial closing segment stays on the endpoint ray of the finite chart.
// Every point of that segment is within its length of the circular endpoint.
func arcEndClosure(r2, x, z, jx, jz *big.Rat) (float64, bool) {
	if r2.Sign() <= 0 || minus(times(x, jz), times(z, jx)).Sign() != 0 || plus(times(x, jx), times(z, jz)).Sign() <= 0 {
		return 0, false
	}
	join := plus(times(jx, jx), times(jz, jz))
	if join.Cmp(r2) == 0 {
		return 0, true
	}
	hi, lo := proofbound.RatSqrtUp(r2), proofbound.RatSqrtDown(join)
	if join.Cmp(r2) > 0 {
		hi, lo = proofbound.RatSqrtUp(join), proofbound.RatSqrtDown(r2)
	}
	gap := proofbound.RatFloatUp(minus(scalar(hi), scalar(lo)))
	return gap, !proofbound.IsNonFinite(gap)
}
