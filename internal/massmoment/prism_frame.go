package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// PrismMidLevel returns the exact rational midpoint of two held levels.
func PrismMidLevel(z0Held, z1Held float64) (*big.Rat, error) {
	z0, z1 := proofarith.FloatRat(z0Held), proofarith.FloatRat(z1Held)
	if z0 == nil || z1 == nil {
		return nil, fmt.Errorf("%w: prism levels are not finite", decaderr.ErrNotFinite)
	}
	return new(big.Rat).Quo(new(big.Rat).Add(z0, z1), big.NewRat(2, 1)), nil
}

// PrismRotation maps frame-local axes through the held placement basis.
func PrismRotation(frame r3.Frame, xform r3.Transform) ([3][3]*big.Rat, error) {
	basis := xform.Basis()
	placement := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	frameAxes := [3]r3.Vec{frame.U(), frame.V(), frame.N()}
	var out [3][3]*big.Rat
	for i := range out {
		for k := range out[i] {
			sum := new(big.Rat)
			for l := range placement {
				entry := proofarith.FloatRat(revolvemesh.VecComponent(placement[l], i))
				axis := proofarith.FloatRat(revolvemesh.VecComponent(frameAxes[k], l))
				if entry == nil || axis == nil {
					return out, fmt.Errorf("%w: prism orientation is not finite", decaderr.ErrNotFinite)
				}
				sum.Add(sum, entry.Mul(entry, axis))
			}
			out[i][k] = sum
		}
	}
	return out, nil
}
