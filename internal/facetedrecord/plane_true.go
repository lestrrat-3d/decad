package facetedrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// PlaneImageIsTag reports whether moved, the float image of the plane tag under delta, is as a point set exactly the
// image of tag under delta read as exact rationals: its normal U'×V' is parallel to (B·U)×(B·V) and its origin lies in
// the image plane through B·O + t, decided in exact rational arithmetic (agent-docs/coplanar-contact-design.md §17).
// The frames may still differ: only the plane is claimed.
func PlaneImageIsTag(tag, moved surfacegeom.Plane, delta r3.Transform) bool {
	linear, err := massmoment.PlacementRotation(delta)
	if err != nil {
		return false
	}
	t, ok := massmoment.ExactVec(delta.Translation())
	if !ok {
		return false
	}
	vec := func(v r3.Vec) ([3]*big.Rat, bool) { return massmoment.ExactVec(v) }
	o, ok1 := vec(tag.Frame.Origin())
	u, ok2 := vec(tag.Frame.U())
	v, ok3 := vec(tag.Frame.V())
	mo, ok4 := vec(moved.Frame.Origin())
	mu, ok5 := vec(moved.Frame.U())
	mv, ok6 := vec(moved.Frame.V())
	if !ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6 {
		return false
	}
	apply := func(x [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range 3 {
			s := new(big.Rat)
			for k := range 3 {
				s.Add(s, new(big.Rat).Mul(linear[i][k], x[k]))
			}
			out[i] = s
		}
		return out
	}
	n := ratCross(apply(u), apply(v))
	nm := ratCross(mu, mv)
	if ratZero(n) || ratZero(nm) || !ratZero(ratCross(n, nm)) {
		return false
	}
	img := apply(o)
	s := new(big.Rat)
	for i := range 3 {
		s.Add(s, new(big.Rat).Mul(n[i], new(big.Rat).Sub(mo[i], new(big.Rat).Add(img[i], t[i]))))
	}
	return s.Sign() == 0
}

func ratZero(a [3]*big.Rat) bool { return a[0].Sign() == 0 && a[1].Sign() == 0 && a[2].Sign() == 0 }

// placedPlane returns the claim a placement by delta leaves on a group whose tag moved to moved: moved itself when the
// group's tag before the placement was an exact plane and moved is exactly its image (PlaneImageIsTag), else nil. The
// tag before the placement is an exact plane when it is a retained, non-denoted Plane that states no departure and
// no displacement (the retained-plane reading), or one an earlier placement proved (PlacedPlane, still equal to it).
func placedPlane(g Group, moved surfacegeom.Surface, delta r3.Transform) *surfacegeom.Plane {
	tag, ok := g.Surface.(surfacegeom.Plane)
	next, okNext := moved.(surfacegeom.Plane)
	if !ok || !okNext || !g.Retained || g.Denoted != nil || g.AxialDelta != 0 || g.ExtentDelta != 0 {
		return nil
	}
	exact := g.NormalBound == 0 || g.PlacedPlane != nil && *g.PlacedPlane == tag
	if !exact || !PlaneImageIsTag(tag, next, delta) {
		return nil
	}
	return &next
}

// PlacedPlane proves the exact image of a retained plane under delta.
func PlacedPlane(g Group, moved surfacegeom.Surface, delta r3.Transform) *surfacegeom.Plane {
	return placedPlane(g, moved, delta)
}
