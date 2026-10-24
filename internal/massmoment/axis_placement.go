package massmoment

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// DriftSinLimit is the largest proven sine of the angle between a copy's image
// of the axis and the axis that AxisRotation admits: 2⁻⁴⁶, which is 64 units
// in the last place of 1. It admits the drift that rounding leaves in
// r3.Rotation about a float-normalized axis, a few units in the last place,
// and declines a deliberate tilt (agent-docs/revolve-axis-design.md §4.5).
const DriftSinLimit = 0x1p-46

// halfPiUpper is a float no smaller than π/2. math.Pi/2 rounds below it.
var halfPiUpper = math.Nextafter(math.Pi/2, math.Inf(1))

// AxisRotation admits a placement as a rotation about axis and returns the map
// charge of its linear part and ε, the displacement charge per unit of radius
// that the placed copy owes against an exact rotation about the axis
// (agent-docs/revolve-axis-design.md §4.5, §5.5).
//
// A canonical axis is XAxisRotation, and ε is the Stretch of the map charge.
// Any other axis admits a valid, non-reflecting placement with zero
// translation whose linear part M, read as exact rationals, has a map charge
// below 1/2, satisfies (M·A)·A > 0 exactly, and moves A by a proven sine
// s_φ ≤ DriftSinLimit. Then ε = e + (π/2)(e + s_φ), rounded up, with e the
// Stretch. The result is false for anything else.
func AxisRotation(t r3.Transform, axis coneaxis.Axis) (MapCharge, float64, bool) {
	if axis.Canonical() {
		if !XAxisRotation(t) {
			return MapCharge{}, 0, false
		}
		rotation, err := PlacementRotation(t)
		if err != nil {
			return MapCharge{}, 0, false
		}
		charge, err := MapChargeOf(rotation)
		if err != nil {
			return MapCharge{}, 0, false
		}
		return charge, charge.Stretch, true
	}
	if !t.IsValid() || t.IsReflection() || t.Translation() != (r3.Vec{}) {
		return MapCharge{}, 0, false
	}
	rotation, err := PlacementRotation(t)
	if err != nil {
		return MapCharge{}, 0, false
	}
	charge, err := MapChargeOf(rotation)
	if err != nil {
		return MapCharge{}, 0, false
	}
	a := axis.Vec()
	exactA := [3]*big.Rat{proofarith.FloatRat(a.X), proofarith.FloatRat(a.Y), proofarith.FloatRat(a.Z)}
	var ma [3]*big.Rat
	for i := range ma {
		ma[i] = new(big.Rat)
		for k := range exactA {
			ma[i].Add(ma[i], new(big.Rat).Mul(rotation[i][k], exactA[k]))
		}
	}
	dot := new(big.Rat)
	maSq := new(big.Rat)
	for i := range ma {
		dot.Add(dot, new(big.Rat).Mul(ma[i], exactA[i]))
		maSq.Add(maSq, new(big.Rat).Mul(ma[i], ma[i]))
	}
	if dot.Sign() <= 0 {
		return MapCharge{}, 0, false
	}
	// |MA×A|² = |MA|²|A|² − (MA·A)², an exact identity.
	aSq := axis.NormSq().Rat()
	crossSq := new(big.Rat).Mul(maSq, aSq)
	crossSq.Sub(crossSq, new(big.Rat).Mul(dot, dot))
	maFloor, aFloor, crossCeil := proofbound.RatSqrtDown(maSq), proofbound.RatSqrtDown(aSq), proofbound.RatSqrtUp(crossSq)
	if maFloor <= 0 || aFloor <= 0 || proofbound.IsNonFinite(crossCeil) {
		return MapCharge{}, 0, false
	}
	// Rounding the numerator up and each factor of the denominator down can
	// only widen the quotient.
	denom := new(big.Rat).Mul(proofarith.FloatRat(maFloor), proofarith.FloatRat(aFloor))
	sin := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(crossCeil), denom))
	if math.IsNaN(sin) || sin > DriftSinLimit {
		return MapCharge{}, 0, false
	}
	e := proofarith.FloatRat(charge.Stretch)
	sum := new(big.Rat).Add(e, proofarith.FloatRat(sin))
	eps := new(big.Rat).Mul(proofarith.FloatRat(halfPiUpper), sum)
	eps.Add(eps, e)
	return charge, proofbound.RatFloatUp(eps), true
}

// XAxisRotation admits only a proper rigid rotation around the
// canonical +X axis used by the Revolve source class. Its held basis error
// is charged by MapChargeOf before the placed mesh is used.
func XAxisRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EX.X >= math.Nextafter(1, 0) && b.EX.X <= math.Nextafter(1, math.Inf(1)) &&
		b.EX.Y == 0 && b.EX.Z == 0 && b.EY.X == 0 && b.EZ.X == 0 &&
		b.EY.Y == b.EZ.Z && b.EY.Z == -b.EZ.Y
}

// XAxisPlacementCharge returns the map charge of a recorded sector placement:
// zero for the identity, the charge of its rotation for an X-axis rotation,
// and an unsupported error for any other transform.
func XAxisPlacementCharge(t r3.Transform) (MapCharge, error) {
	if t == r3.Identity() {
		return MapCharge{}, nil
	}
	if !XAxisRotation(t) {
		return MapCharge{}, fmt.Errorf("%w: sector placement is not an axis rotation", decaderr.ErrUnsupported)
	}
	rotation, err := PlacementRotation(t)
	if err != nil {
		return MapCharge{}, err
	}
	return MapChargeOf(rotation)
}

// PlacedBound charges a mesh bound for the placement map charge.
func PlacedBound(bound float64, charge MapCharge) float64 {
	return proofbound.AbsSumUpper(bound,
		proofbound.ProductUpper(charge.Stretch, bound))
}
