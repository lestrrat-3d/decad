package explodeblock

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Directions returns the candidate removal directions: each given axis in both
// senses, in the order given, then the six world axes. Axes are normalised;
// zero or non-finite axes are dropped and directions closer than tol (the
// largest component of the difference) to one already listed are not repeated.
func Directions(axes []r3.Vec, tol float64) []r3.Vec {
	var out []r3.Vec
	add := func(d r3.Vec) {
		n, ok := d.Normalize()
		if !ok {
			return
		}
		for _, o := range out {
			if math.Abs(o.X-n.X) <= tol && math.Abs(o.Y-n.Y) <= tol && math.Abs(o.Z-n.Z) <= tol {
				return
			}
		}
		out = append(out, n)
	}
	for _, a := range axes {
		add(a)
		add(a.Scale(-1))
	}
	for _, w := range []r3.Vec{
		r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1),
		r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0),
		r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0),
	} {
		add(w)
	}
	return out
}
