package massmoment

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

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
