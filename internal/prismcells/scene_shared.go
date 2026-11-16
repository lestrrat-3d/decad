package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// Shared is the coincident reading ChargeCrossings stored. It is empty before
// ChargeCrossings runs.
func (d SceneDelta) Shared() CoincidentReading { return d.shared }

// CircleRadiusBound is an outward-rounded upper bound on |r - rho| for every
// radius rho the circular record segment defines: a CircleSeg's recorded
// radius, or an ArcSeg's distance from Center to Start and to End. Each
// squared radius is an exact rational of the held floats, and the bound
// reads the outward square roots of the two squares, as circleGapUpper does.
// It is exactly 0 when every defined radius equals r exactly, and +Inf for a
// non-circular segment, a radius not in millimeters, or a non-finite value.
func CircleRadiusBound(seg sectionrecord.CurveSegment, r float64) float64 {
	rr := floatRat(r)
	if rr == nil {
		return math.Inf(1)
	}
	var squares []*big.Rat
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		if s.Radius.Unit() != units.Millimeter {
			return math.Inf(1)
		}
		c := floatRat(s.Radius.Mag())
		if c == nil {
			return math.Inf(1)
		}
		squares = []*big.Rat{new(big.Rat).Mul(c, c)}
	case sectionrecord.ArcSeg:
		c, ok := floatRats(s.Center.U, s.Center.V, s.Start.U, s.Start.V, s.End.U, s.End.V)
		if !ok {
			return math.Inf(1)
		}
		sq := func(x, y *big.Rat) *big.Rat {
			dx, dy := new(big.Rat).Sub(x, c[0]), new(big.Rat).Sub(y, c[1])
			return new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		}
		squares = []*big.Rat{sq(c[2], c[3]), sq(c[4], c[5])}
	default:
		return math.Inf(1)
	}
	want := new(big.Rat).Mul(rr, rr)
	bound := 0.0
	for _, p := range squares {
		hi, lo := p, want
		if hi.Cmp(lo) < 0 {
			hi, lo = lo, hi
		}
		if hi.Cmp(lo) == 0 {
			continue
		}
		bound = max(bound, math.Nextafter(proofbound.RatSqrtUp(hi)-proofbound.RatSqrtDown(lo), math.Inf(1)))
	}
	return bound
}

func floatRat(v float64) *big.Rat {
	out, ok := floatRats(v)
	if !ok {
		return nil
	}
	return out[0]
}
