package prismpattern

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ConnectorReach bounds how far each straight connector of a copied prism's
// trailing root arc runs inside the blank circle. A connector begins strictly
// inside the circle, ends strictly outside, and increases squared radius at its
// inner endpoint. Convexity of squared radius bounds its unique crossing
// parameter by (R²-r0²)/(2 near·step).
func ConnectorReach(p prismrecord.Payload, root sectionrecord.ArcSeg, radius float64,
	rootIndex int) (float64, error) {
	segments := p.Profile.Outer.Segments
	if rootIndex != len(segments)-1 || rootIndex < 2 {
		return 0, fmt.Errorf("%w: copied-prism connectors have no trailing root arc", decaderr.ErrUnsupported)
	}
	scale2 := Scale2(p.Xform)
	radiusSq := RadiusSq(radius)
	reach := 0.0
	for _, index := range []int{0, rootIndex - 1} {
		line, ok := segments[index].(sectionrecord.LineSeg)
		if !ok {
			return 0, fmt.Errorf("%w: copied-prism root connector is not a line", decaderr.ErrUnsupported)
		}
		near, far := line.Start, line.End
		if near != root.Start && near != root.End {
			near, far = far, near
		}
		if near != root.Start && near != root.End {
			return 0, fmt.Errorf("%w: copied-prism connector misses its root arc", decaderr.ErrUnsupported)
		}
		nu, nv := proofarith.FloatRat(near.U), proofarith.FloatRat(near.V)
		du := new(big.Rat).Sub(proofarith.FloatRat(far.U), nu)
		dv := new(big.Rat).Sub(proofarith.FloatRat(far.V), nv)
		derivative := new(big.Rat).Mul(big.NewRat(2, 1), scale2)
		derivative.Mul(derivative, new(big.Rat).Add(new(big.Rat).Mul(nu, du), new(big.Rat).Mul(nv, dv)))
		if derivative.Sign() <= 0 {
			return 0, fmt.Errorf("%w: copied-prism connector has no outward crossing", decaderr.ErrUnsupported)
		}
		nearSq := new(big.Rat).Mul(RadialSq(near), scale2)
		gap := new(big.Rat).Sub(radiusSq, nearSq)
		if gap.Sign() <= 0 {
			return 0, fmt.Errorf("%w: copied-prism connector starts outside the blank", decaderr.ErrUnsupported)
		}
		tUpper := new(big.Rat).Quo(gap, derivative)
		if tUpper.Cmp(big.NewRat(1, 1)) > 0 {
			tUpper.SetInt64(1)
		}
		stepSq := new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
		length := proofbound.ProductUpper(proofbound.RatSqrtUp(stepSq), proofbound.RatFloatUp(tUpper))
		reach = max(reach, length)
	}
	return reach, nil
}
