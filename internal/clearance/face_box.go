package clearance

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file builds the boxes the kernel reads a face's or an edge's extent
// from (docs/clearance-design.md §5). Every box encloses the feature exactly
// as the cells read it, from the carrier's own floats: each coordinate is
// computed over exact rationals and rounded outward, and every square root is
// rounded up. A box is never read off float sums or products of those
// floats, whose rounding scales with the terms rather than with the result:
// a lift that cancels, or an axis tilted by less than √u off a coordinate
// axis, whose 1 − a² rounds to zero.

// everywhere is the box that contains every point, returned for an input
// whose coordinates do not lift to exact rationals.
func everywhere() [2]r3.Vec {
	inf := math.Inf(1)
	return [2]r3.Vec{r3.NewVec(-inf, -inf, -inf), r3.NewVec(inf, inf, inf)}
}

// ratVec lifts a float vector to exact rationals. ok is false for a
// non-finite coordinate.
func ratVec(v r3.Vec) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for i, c := range [3]float64{v.X, v.Y, v.Z} {
		if proofbound.IsNonFinite(c) {
			return out, false
		}
		out[i] = new(big.Rat).SetFloat64(c)
	}
	return out, true
}

// outwardBox returns the box [c − ext, c + ext] over an exact centre c and
// exact half-widths ext, rounded outward.
func outwardBox(c, ext [3]*big.Rat) [2]r3.Vec {
	var lo, hi [3]float64
	for i := range 3 {
		lo[i] = proofbound.RatFloatDown(new(big.Rat).Sub(c[i], ext[i]))
		hi[i] = proofbound.RatFloatUp(new(big.Rat).Add(c[i], ext[i]))
	}
	return [2]r3.Vec{r3.NewVec(lo[0], lo[1], lo[2]), r3.NewVec(hi[0], hi[1], hi[2])}
}

// circleExtent returns, for each coordinate axis e_i, a proven upper bound on
// the half-width r·√((a_j² + a_k²)/|a|²) a circle of radius r in the plane
// normal to a spans along e_i: the exact extent for an axis of any nonzero
// length. ok is false for a zero or non-finite axis or radius.
func circleExtent(axis r3.Vec, r float64) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	a, ok := ratVec(axis)
	if !ok || proofbound.IsNonFinite(r) {
		return out, false
	}
	sq := [3]*big.Rat{}
	den := new(big.Rat)
	for i := range 3 {
		sq[i] = new(big.Rat).Mul(a[i], a[i])
		den.Add(den, sq[i])
	}
	if den.Sign() == 0 {
		return out, false
	}
	for i := range 3 {
		num := new(big.Rat).Add(sq[(i+1)%3], sq[(i+2)%3])
		ext := proofbound.ProductUpper(math.Abs(r), proofbound.RatSqrtUp(num.Quo(num, den)))
		if proofbound.IsNonFinite(ext) {
			return out, false
		}
		out[i] = new(big.Rat).SetFloat64(ext)
	}
	return out, true
}

// CircleBox is a proven box of the circle of radius r about c in the plane
// normal to axis, which may have any nonzero length.
func CircleBox(c, axis r3.Vec, r float64) [2]r3.Vec {
	return AxisCircleBox(c, axis, 0, r)
}

// AxisCircleBox is a proven box of the circle of radius r about the axis line
// through anchor along axis, at the axial reading z: the circle whose points
// p have (p − anchor)·axis = z, the reading AxisCoords takes. Its centre is
// anchor + axis·z/|axis|², held exactly.
func AxisCircleBox(anchor, axis r3.Vec, z, r float64) [2]r3.Vec {
	o, okO := ratVec(anchor)
	a, okA := ratVec(axis)
	ext, okE := circleExtent(axis, r)
	if !okO || !okA || !okE || proofbound.IsNonFinite(z) {
		return everywhere()
	}
	den := new(big.Rat)
	for i := range 3 {
		den.Add(den, new(big.Rat).Mul(a[i], a[i]))
	}
	t := new(big.Rat).Quo(new(big.Rat).SetFloat64(z), den)
	var c [3]*big.Rat
	for i := range 3 {
		c[i] = new(big.Rat).Add(o[i], new(big.Rat).Mul(a[i], t))
	}
	return outwardBox(c, ext)
}

// BallBox is a proven box of the ball of radius r about c.
func BallBox(c r3.Vec, r float64) [2]r3.Vec {
	o, ok := ratVec(c)
	if !ok || proofbound.IsNonFinite(r) {
		return everywhere()
	}
	e := new(big.Rat).SetFloat64(math.Abs(r))
	return outwardBox(o, [3]*big.Rat{e, e, e})
}

// PadBox grows a box by pad on every side, rounded outward.
func PadBox(b [2]r3.Vec, pad float64) [2]r3.Vec {
	pad = math.Abs(pad)
	down := func(x float64) float64 { return math.Nextafter(x-pad, math.Inf(-1)) }
	up := func(x float64) float64 { return math.Nextafter(x+pad, math.Inf(1)) }
	return [2]r3.Vec{
		r3.NewVec(down(b[0].X), down(b[0].Y), down(b[0].Z)),
		r3.NewVec(up(b[1].X), up(b[1].Y), up(b[1].Z)),
	}
}

// CapBox is a proven box of a planar face: every point o + u·x + v·y with
// (x, y) in the face's region, each arc taken as its full circle. A line
// element's two ends are held exactly, and an arc's image is the ellipse
// about its centre's image with half-width Rr·√(u_i² + v_i²) along e_i, the
// exact extent of x·u_i + y·v_i over x² + y² = Rr².
func CapBox(f *CFace) [2]r3.Vec {
	o, okO := ratVec(f.O)
	u, okU := ratVec(f.U)
	v, okV := ratVec(f.V)
	if !okO || !okU || !okV {
		return everywhere()
	}
	at := func(x, y float64) ([3]*big.Rat, bool) {
		var p [3]*big.Rat
		if proofbound.IsNonFinite(x) || proofbound.IsNonFinite(y) {
			return p, false
		}
		rx, ry := new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y)
		for i := range 3 {
			p[i] = new(big.Rat).Add(o[i], new(big.Rat).Add(new(big.Rat).Mul(u[i], rx), new(big.Rat).Mul(v[i], ry)))
		}
		return p, true
	}
	zero := new(big.Rat)
	box := [2]r3.Vec{
		r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)),
		r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1)),
	}
	for _, e := range f.Region.Elems {
		if e.Kind == survey2d.SurveyLine {
			for _, end := range [2][2]float64{{e.Ax, e.Ay}, {e.Bx, e.By}} {
				p, ok := at(end[0], end[1])
				if !ok {
					return everywhere()
				}
				box = BoxUnion(box, outwardBox(p, [3]*big.Rat{zero, zero, zero}))
			}
			continue
		}
		c, ok := at(e.Qx, e.Qy)
		if !ok || proofbound.IsNonFinite(e.Rr) {
			return everywhere()
		}
		var ext [3]*big.Rat
		for i := range 3 {
			sq := new(big.Rat).Add(new(big.Rat).Mul(u[i], u[i]), new(big.Rat).Mul(v[i], v[i]))
			w := proofbound.ProductUpper(math.Abs(e.Rr), proofbound.RatSqrtUp(sq))
			if proofbound.IsNonFinite(w) {
				return everywhere()
			}
			ext[i] = new(big.Rat).SetFloat64(w)
		}
		box = BoxUnion(box, outwardBox(c, ext))
	}
	return box
}

// coneBox is a proven box of a cone face: the union of its two rims, the
// circles at the axial readings ZWin.Lo and ZWin.Hi from the apex, of radius
// z·Rise/Run rounded up. The face is ruled between those rims, so it lies in
// the hull of the two.
func coneBox(f *CFace) [2]r3.Vec {
	if !(f.Run > 0) {
		return everywhere()
	}
	rim := func(z float64) [2]r3.Vec {
		return AxisCircleBox(f.Anchor, f.Axis, z, proofbound.DivUpper(proofbound.ProductUpper(math.Abs(z), f.Rise), f.Run))
	}
	return BoxUnion(rim(f.ZWin.Lo), rim(f.ZWin.Hi))
}
