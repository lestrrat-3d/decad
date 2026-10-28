package exactbool

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Plane is the plane N·x = H through three float points, with N = (b−a)×(c−a) and H = N·a, both exact. N is not
// normalised, so no division happens anywhere in the kernel's decisions.
type Plane struct {
	N proofarith.DyV3
	H proofarith.Dyadic
}

// PlaneThrough returns the plane through a, b and c. Its normal is zero when the points are collinear; Degenerate
// reports that.
func PlaneThrough(a, b, c r3.Vec) Plane {
	da := proofarith.DyVec(a)
	n := proofarith.DvCross(proofarith.DvSub(proofarith.DyVec(b), da), proofarith.DvSub(proofarith.DyVec(c), da))
	return Plane{N: n, H: proofarith.DvDot(n, da)}
}

// Degenerate reports whether the plane has a zero normal.
func (p Plane) Degenerate() bool { return proofarith.DvIsZero(p.N) }

// SideOf returns the sign of N·x − H at the float point x: +1 on the side the normal points to.
func (p Plane) SideOf(x r3.Vec) int {
	return proofarith.DySubScalar(proofarith.DvDot(p.N, proofarith.DyVec(x)), p.H).Sign()
}

// Point is the meeting of three planes, kept as the homogeneous numerators X and the determinant D, so the point is
// X/D and no coordinate is ever divided.
type Point struct {
	X proofarith.DyV3
	D proofarith.Dyadic
}

// Meet returns the point common to p, q and s, and false when the three normals are dependent.
func Meet(p, q, s Plane) (Point, bool) {
	qs := proofarith.DvCross(q.N, s.N)
	d := proofarith.DvDot(p.N, qs)
	if d.IsZero() {
		return Point{}, false
	}
	sp := proofarith.DvCross(s.N, p.N)
	pq := proofarith.DvCross(p.N, q.N)
	x := proofarith.DvAdd(proofarith.DvAdd(proofarith.DvScale(qs, p.H), proofarith.DvScale(sp, q.H)),
		proofarith.DvScale(pq, s.H))
	return Point{X: x, D: d}, true
}

// SideOfPoint returns the sign of N·x − H at the point: +1 on the side the plane's normal points to.
func (p Plane) SideOfPoint(x Point) int {
	v := proofarith.DySubScalar(proofarith.DvDot(p.N, x.X), proofarith.DyMul(p.H, x.D))
	return v.Sign() * x.D.Sign()
}

// Order returns −1, 0 or +1 as the axis coordinate of a is less than, equal to or greater than b's.
func Order(axis int, a, b Point) int {
	v := proofarith.DySubScalar(proofarith.DyMul(a.X[axis], b.D), proofarith.DyMul(b.X[axis], a.D))
	return v.Sign() * a.D.Sign() * b.D.Sign()
}

// Equal reports whether a and b are the same point.
func Equal(a, b Point) bool {
	return Order(0, a, b) == 0 && Order(1, a, b) == 0 && Order(2, a, b) == 0
}

// Float returns the point rounded to the nearest floats, and a bound on the distance from the rounded point to the
// exact one: half an ulp of each rounded coordinate, combined.
func (x Point) Float() (r3.Vec, float64) {
	d := x.D.Rat()
	var out [3]float64
	var sq float64
	for i := range out {
		q := new(big.Rat).Quo(x.X[i].Rat(), d)
		f, _ := q.Float64()
		out[i] = f
		e := math.Abs(math.Nextafter(f, math.Inf(1))-f) / 2
		if e2 := math.Abs(f-math.Nextafter(f, math.Inf(-1))) / 2; e2 > e {
			e = e2
		}
		sq += e * e
	}
	return r3.NewVec(out[0], out[1], out[2]), math.Sqrt(sq)
}
