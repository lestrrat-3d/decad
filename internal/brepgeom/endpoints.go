package brepgeom

import "github.com/lestrrat-3d/r3"

// MatchEndpoints reports whether two endpoint pairs agree without regard to direction.
func MatchEndpoints(a, b, p, q r3.Vec, tol float64) bool {
	near := func(u, v r3.Vec) bool { return u.Sub(v).Len() <= tol }
	return (near(a, p) && near(b, q)) || (near(a, q) && near(b, p))
}
