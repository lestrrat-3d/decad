package compositesweep

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RimPost is one held vertex of a path rim: its position and coordinate bound.
type RimPost struct {
	Position r3.Vec
	Bound    float64
}

// CertifyRimPair checks that two chain rims meet at the join. Each rim is its
// start and end post; a nil post is a missing source post. Matching posts must
// lie within the sum of their coordinate bounds.
func CertifyRimPair(from, to [2]*RimPost) error {
	for i := range from {
		a, b := from[i], to[i]
		if a == nil || b == nil {
			return fmt.Errorf(`%w: a composite chain join has a missing source post`, decaderr.ErrUnsupported)
		}
		d2, ok := proofarith.DySquaredDistance3(
			a.Position.X, a.Position.Y, a.Position.Z,
			b.Position.X, b.Position.Y, b.Position.Z)
		if !ok || proofarith.DySqrtUp(d2) > proofbound.AbsSumUpper(a.Bound, b.Bound) {
			return fmt.Errorf(`%w: composite chain source rims disagree beyond their coordinate bounds`,
				decaderr.ErrUnsupported)
		}
	}
	return nil
}
