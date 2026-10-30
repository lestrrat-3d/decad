package revolvebore

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// InsideRadius proves every complete section segment strictly inside the
// axis-centered circle. Circular segments use their complete carrier, so an
// arc whose carrier crosses the circle is conservatively refused.
func InsideRadius(loop sectionrecord.LoopRecord, radius float64) bool {
	r := proofarith.FloatRat(radius)
	if r == nil || r.Sign() <= 0 || len(loop.Segments) == 0 {
		return false
	}
	r2 := new(big.Rat).Mul(r, r)
	work := freeform.NewFreeformWork()
	for _, seg := range loop.Segments {
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			if !insidePoint(s.Start, r2) || !insidePoint(s.End, r2) {
				return false
			}
		case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil || !w.IsCircular() {
				return false
			}
			carrierRadius := proofbound.AbsSumUpper(w.Radius, w.RadiusBound,
				proofbound.WalkEndBoundAllow(w.StartBound), proofbound.WalkEndBoundAllow(w.EndBound))
			if !insideCarrier(sectionrecord.Point2{U: w.CU, V: w.CV}, carrierRadius, r2) {
				return false
			}
		default:
			return false
		}
	}
	return true
}

func pointRadiusSquared(p sectionrecord.Point2) *big.Rat {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	if u == nil || v == nil {
		return nil
	}
	return new(big.Rat).Add(new(big.Rat).Mul(u, u), new(big.Rat).Mul(v, v))
}

func insidePoint(p sectionrecord.Point2, radiusSquared *big.Rat) bool {
	p2 := pointRadiusSquared(p)
	return p2 != nil && p2.Cmp(radiusSquared) < 0
}

func insideCarrier(center sectionrecord.Point2, radius float64, outerSquared *big.Rat) bool {
	c2, r := pointRadiusSquared(center), proofarith.FloatRat(radius)
	if c2 == nil || r == nil || r.Sign() <= 0 {
		return false
	}
	r2 := new(big.Rat).Mul(r, r)
	remaining := new(big.Rat).Sub(new(big.Rat).Sub(outerSquared, c2), r2)
	if remaining.Sign() <= 0 {
		return false
	}
	left := new(big.Rat).Mul(remaining, remaining)
	right := new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(c2, r2))
	return left.Cmp(right) > 0
}
