package tessellation

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// InsideSignOf is the exact ±1 rational offsetRadius's own sign convention
// reads off a circular wall's walked sense: +1 when the wall's material
// lies inside the circle (th1 >= th0), −1 when it lies outside.
func InsideSignOf(w survey2d.SideWalk) *big.Rat {
	if w.Th1 < w.Th0 {
		return big.NewRat(-1, 1)
	}
	return big.NewRat(1, 1)
}
