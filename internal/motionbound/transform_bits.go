package motionbound

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// TransformBits retains signed zeros so a cached payload is reused only
// when the composed transform has the same float operands in every component.
func TransformBits(t r3.Transform) [12]uint64 {
	b := t.Basis()
	vectors := [4]r3.Vec{b.EX, b.EY, b.EZ, t.Translation()}
	var key [12]uint64
	for i, v := range vectors {
		key[3*i] = math.Float64bits(v.X)
		key[3*i+1] = math.Float64bits(v.Y)
		key[3*i+2] = math.Float64bits(v.Z)
	}
	return key
}
