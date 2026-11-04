// Package occupancy certifies how far a held mesh lies from the true solid a
// set of convex leaves denotes (agent-docs/body-refine-design.md §4–§5). A
// leaf is described by an exact outer piece, a convex polyhedron that contains
// the leaf's true solid. A held facet piece moved by a translation that misses
// every outer piece has every point within the translation's length of the
// true boundary, provided the held mesh lies inside the true solid up to its
// rounding (§5 (i)).
//
// Every admission is exact: float arithmetic only proposes a separating axis,
// and the rational comparison on that axis decides.
package occupancy

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// MinSides and MaxSides bound the circumscribed polygon of CirclePrism.
const (
	MinSides = 3
	MaxSides = 1024
)

// Affine is the map x ↦ M·x + T whose float entries are read as exact
// rationals. It need not be exactly orthogonal: an affine image keeps
// containment, so a piece built under it still contains the leaf's image.
type Affine struct {
	M [3][3]float64
	T [3]float64
}

// Identity is the identity map.
func Identity() Affine {
	return Affine{M: [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
}

// Chain applies its maps in order, each exactly. A composite of float maps
// generally has no float entries, so the maps stay separate. Which maps a
// leaf's record denotes (its section frame, then its placement) is the
// adapter's decision (agent-docs/body-refine-design.md §4, increment R2).
type Chain []Affine

// xvec is an exact point.
type xvec [3]*big.Rat

func ratOf(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

func xvecOf(v r3.Vec) xvec { return xvec{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

func (a Affine) applyExact(p xvec) xvec {
	var out xvec
	for i := range 3 {
		s := ratOf(a.T[i])
		for j := range 3 {
			s.Add(s, new(big.Rat).Mul(ratOf(a.M[i][j]), p[j]))
		}
		out[i] = s
	}
	return out
}

func (c Chain) applyExact(p xvec) xvec {
	for _, a := range c {
		p = a.applyExact(p)
	}
	return p
}

func (p xvec) float() r3.Vec {
	x, _ := p[0].Float64()
	y, _ := p[1].Float64()
	z, _ := p[2].Float64()
	return r3.Vec{X: x, Y: y, Z: z}
}

// ratVec is a vector of exact rationals.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) ratVec { return ratVec{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)} }

// dotExact is a·p, exactly.
func dotExact(a ratVec, p xvec) *big.Rat {
	var sum fracSum
	sum.start(new(big.Rat))
	for k := range 3 {
		sum.addProduct(a[k], p[k])
	}
	return sum.rat()
}

// fracSum accumulates a sum of rational products as one fraction, with no
// reduction between terms and one at the end. The value is the canonical
// rational a term-by-term big.Rat sum gives, for far fewer GCDs.
type fracSum struct {
	n, d, tn, td, scaled big.Int
}

// start sets the sum to r.
func (f *fracSum) start(r *big.Rat) {
	f.n.Set(r.Num())
	f.d.Set(r.Denom())
}

// addProduct adds a·b to the sum.
func (f *fracSum) addProduct(a, b *big.Rat) {
	f.tn.Mul(a.Num(), b.Num())
	f.td.Mul(a.Denom(), b.Denom())
	if f.d.Cmp(&f.td) == 0 {
		f.n.Add(&f.n, &f.tn)
		return
	}
	f.scaled.Mul(&f.tn, &f.d)
	f.n.Mul(&f.n, &f.td)
	f.n.Add(&f.n, &f.scaled)
	f.d.Mul(&f.d, &f.td)
}

// zero sets the sum to 0.
func (f *fracSum) zero() {
	f.n.SetInt64(0)
	f.d.SetInt64(1)
}

// cmpRat compares the sum with c by cross-multiplying, without reducing
// either: both denominators are positive, so the order is the rationals'.
func (f *fracSum) cmpRat(c *big.Rat) int {
	f.tn.Mul(&f.n, c.Denom())
	f.td.Mul(c.Num(), &f.d)
	return f.tn.Cmp(&f.td)
}

// rat returns the sum as a reduced rational.
func (f *fracSum) rat() *big.Rat { return new(big.Rat).SetFrac(&f.n, &f.d) }

// Piece is a convex polyhedron held exactly by its vertices, with float
// copies of the vertices, its face normals and its edges for the axis search.
type Piece struct {
	verts  []xvec
	fverts []r3.Vec
	faces  []face
	edges  [][2]int
	box    [2]r3.Vec
	// The piece's own half-spaces in local coordinates: side k is
	// ux·x + uy·y ≤ c and the levels bound z. inv takes a world point back to
	// local coordinates exactly and finv in floats, for ranking only.
	sides  []side
	z0, z1 *big.Rat
	fz0    float64
	fz1    float64
	inv    []exactAffine
	finv   []Affine
}

type side struct {
	ux, uy, c    *big.Rat
	fux, fuy, fc float64
}

// beyond reports whether ux·x[0] + uy·x[1] exceeds c, exactly. The sum is
// held as one unreduced fraction and compared by cross-multiplication, so no
// greatest common divisor is taken.
func (s side) beyond(x xvec) bool {
	var sum fracSum
	sum.zero()
	sum.addProduct(s.ux, x[0])
	sum.addProduct(s.uy, x[1])
	return sum.cmpRat(s.c) > 0
}

// face is one plane of the piece in floats, for ranking axes and escape
// directions only: unit outward normal n and offset off = n·(a face vertex).
type face struct {
	n   r3.Vec
	off float64
}

// Box is the piece's float bounding box, widened by one ulp on every side so
// that it contains the exact vertices.
func (p *Piece) Box() [2]r3.Vec { return p.box }

// Vertices returns float copies of the exact vertices.
func (p *Piece) Vertices() []r3.Vec { return append([]r3.Vec(nil), p.fverts...) }

// ErrInvalid is wrapped by every CirclePrism refusal of its arguments.
var ErrInvalid = errors.New("occupancy: invalid circle prism")

// CirclePrism returns an exact convex piece that contains the image, under
// place, of the solid cylinder x² + y² ≤ r², z0 ≤ z ≤ z1. The section is the
// circumscribed polygon of the half-planes x·u_k ≤ c_k, where u_k is the float
// direction at angle 2πk/sides and c_k a float proven to satisfy
// c_k² ≥ r²·(u_k·u_k), so the circle lies inside every half-plane.
func CirclePrism(place Chain, r, z0, z1 float64, sides int) (*Piece, error) {
	switch {
	case sides < MinSides || sides > MaxSides:
		return nil, fmt.Errorf("%w: %d sides outside [%d, %d]", ErrInvalid, sides, MinSides, MaxSides)
	case !(r > 0) || math.IsInf(r, 0):
		return nil, fmt.Errorf("%w: radius %v", ErrInvalid, r)
	case math.IsNaN(z0) || math.IsNaN(z1) || math.IsInf(z0, 0) || math.IsInf(z1, 0) || !(z0 < z1):
		return nil, fmt.Errorf("%w: levels %v, %v", ErrInvalid, z0, z1)
	}
	r2 := new(big.Rat).Mul(ratOf(r), ratOf(r))
	lines := make([]side, sides)
	for k := range lines {
		th := 2 * math.Pi * float64(k) / float64(sides)
		ux, uy := math.Cos(th), math.Sin(th)
		c := r * math.Hypot(ux, uy)
		rx, ry := ratOf(ux), ratOf(uy)
		need := new(big.Rat).Mul(rx, rx)
		need.Add(need, new(big.Rat).Mul(ry, ry))
		need.Mul(need, r2)
		for steps := 0; ; steps++ {
			rc := ratOf(c)
			if new(big.Rat).Mul(rc, rc).Cmp(need) >= 0 {
				break
			}
			if steps > 8 {
				return nil, fmt.Errorf("%w: offset for side %d did not converge", decaderr.ErrUnsupported, k)
			}
			c = math.Nextafter(c, math.Inf(1))
		}
		lines[k] = side{rx, ry, ratOf(c), ux, uy, c}
	}
	return linesPrism(place, lines, z0, z1, circleCornerPrefilter)
}

// cornerClearlyInside reports true only when the exact corner P, held as
// px = fl(P.x) and py = fl(P.y) (each the nearest float), satisfies the
// half-plane ux·x + uy·y < c, for finite floats ux, uy in [−1, 1] and c. It
// only lets CirclePrism skip an exact comparison that would pass; false
// decides nothing, and the exact comparison runs.
//
// Bound. Write u = 2^-53 and η = 2^-1075, the absolute rounding error of a
// result in the subnormal range, so every rounded operation errs by at most
// u times its result plus η. ux, uy and c are exact floats (the lines hold the
// very values their exact twins were made from), so the only errors are:
//
//   - px = P.x + dx with |dx| ≤ u|P.x| + η, and likewise py;
//   - q1 = fl(ux·px), so |q1 − ux·P.x| ≤ 2.01u|ux·P.x| + 3η, and likewise q2;
//   - t = fl(q1 + q2), whose sum adds at most u|q1 + q2| and no η, because a
//     sum of floats that lands in the subnormal range is exact.
//
// With A = |ux·P.x| + |uy·P.y| this gives |t − (ux·P.x + uy·P.y)| ≤
// 3.1u·A + 7η. The margin m = fl(fl(a·2^-50) + 2^-1000) uses a = fl(|q1| +
// |q2|) ≥ A(1 − 3.1u) − 7η, and 2^-50 = 8u, so m ≥ 7.9u·A − 8η +
// (1 − u)·2^-1000 is at least 2.5 times the bound, the η terms being dwarfed
// by 2^-1000. Rounding is monotone and m is a float, so fl(c − t) > m implies
// c − t > m exactly, whence c − (ux·P.x + uy·P.y) > m − 3.1u·A − 7η > 0.
//
// Overflow and NaN. A corner beyond the float range converts to ±Inf, and a
// product can overflow. Then q1, q2 or t is infinite or NaN. NaN fails every
// comparison. t = +Inf gives c − t = −Inf. t = −Inf gives c − t = +Inf, but
// then a = +Inf, so m = +Inf and +Inf > +Inf is false. If only a overflows,
// m = +Inf and the test is again false. So a non-finite reading never
// skips. The float64() conversions keep every product rounded on its own, so
// no platform fuses one into a sum or difference.
func cornerClearlyInside(ux, uy, c, px, py float64) bool {
	q1, q2 := float64(ux*px), float64(uy*py)
	t := q1 + q2
	a := math.Abs(q1) + math.Abs(q2)
	return c-t > float64(a*0x1p-50)+0x1p-1000
}

// containsMoved reports whether every corner of tri moved by shift satisfies
// every one of p's own half-spaces, decided on exact local coordinates.
func (p *Piece) containsMoved(tri [3]r3.Vec, shift r3.Vec) bool {
	sx := xvecOf(shift)
	for _, v := range tri {
		x := xvecOf(v)
		for k := range 3 {
			x[k].Add(x[k], sx[k])
		}
		f := v.Add(shift)
		for _, a := range p.finv {
			f = a.applyFloat(f)
		}
		// A float reading clearly outside a level or side refuses at once.
		if f.Z < p.fz0-1e-9 || f.Z > p.fz1+1e-9 {
			return false
		}
		for _, e := range p.inv {
			x = e.apply(x)
		}
		if x[2].Cmp(p.z0) < 0 || x[2].Cmp(p.z1) > 0 {
			return false
		}
		for _, sd := range p.sides {
			if sd.fux*f.X+sd.fuy*f.Y > sd.fc+1e-9 {
				return false
			}
			if sd.beyond(x) {
				return false
			}
		}
	}
	return true
}

// InscribedCirclePrism returns an exact convex piece inside the image, under
// place, of the solid cylinder x² + y² ≤ r², z0 ≤ z ≤ z1: the prism of a
// polygon whose corners are float points proven exactly inside the circle,
// between the two levels.
func InscribedCirclePrism(place Chain, r, z0, z1 float64, sides int) (*Piece, error) {
	switch {
	case sides < MinSides || sides > MaxSides:
		return nil, fmt.Errorf("%w: %d sides outside [%d, %d]", ErrInvalid, sides, MinSides, MaxSides)
	case !(r > 0) || math.IsInf(r, 0):
		return nil, fmt.Errorf("%w: radius %v", ErrInvalid, r)
	case math.IsNaN(z0) || math.IsNaN(z1) || math.IsInf(z0, 0) || math.IsInf(z1, 0) || !(z0 < z1):
		return nil, fmt.Errorf("%w: levels %v, %v", ErrInvalid, z0, z1)
	}
	rr := new(big.Rat).Mul(ratOf(r), ratOf(r))
	corners := make([][2]*big.Rat, sides)
	for k := range corners {
		th := 2 * math.Pi * float64(k) / float64(sides)
		x, y := r*math.Cos(th), r*math.Sin(th)
		for steps := 0; ; steps++ {
			rx, ry := ratOf(x), ratOf(y)
			q := new(big.Rat).Mul(rx, rx)
			q.Add(q, new(big.Rat).Mul(ry, ry))
			if q.Cmp(rr) <= 0 {
				corners[k] = [2]*big.Rat{rx, ry}
				break
			}
			if steps > 8 {
				return nil, fmt.Errorf("%w: corner %d would not fit inside the circle", decaderr.ErrUnsupported, k)
			}
			x, y = x*(1-1e-15), y*(1-1e-15)
		}
	}
	p := &Piece{z0: ratOf(z0), z1: ratOf(z1), fz0: z0, fz1: z1}
	for k := range corners {
		a, b := corners[k], corners[(k+1)%sides]
		// The outward normal of the CCW edge a→b is (dy, −dx).
		ux := new(big.Rat).Sub(b[1], a[1])
		uy := new(big.Rat).Sub(a[0], b[0])
		c := new(big.Rat).Mul(ux, a[0])
		c.Add(c, new(big.Rat).Mul(uy, a[1]))
		fux, _ := ux.Float64()
		fuy, _ := uy.Float64()
		fc, _ := c.Float64()
		p.sides = append(p.sides, side{ux, uy, c, fux, fuy, fc})
	}
	for _, a := range slices.Backward(place) {
		e, err := invert(a)
		if err != nil {
			return nil, err
		}
		p.inv = append(p.inv, e)
		p.finv = append(p.finv, e.float())
	}
	for _, z := range []float64{z0, z1} {
		rz := ratOf(z)
		for _, c := range corners {
			v := place.applyExact(xvec{c[0], c[1], rz})
			p.verts = append(p.verts, v)
			p.fverts = append(p.fverts, v.float())
		}
	}
	for k := range sides {
		p.edges = append(p.edges, [2]int{k, (k + 1) % sides}, [2]int{sides + k, sides + (k+1)%sides}, [2]int{k, sides + k})
	}
	p.finishFaces(sides)
	return p, nil
}

// finishFaces sets the float face planes, for the axis and direction search
// only (each side quad and both caps, oriented away from the vertex
// centroid), and the float box, widened one ulp so it holds the exact
// vertices, of a prism piece of n sides.
func (p *Piece) finishFaces(n int) {
	// Face planes for the axis and direction search only: each side quad
	// and both caps, oriented away from the vertex centroid.
	var g r3.Vec
	for _, v := range p.fverts {
		g = g.Add(v)
	}
	g = g.Scale(1 / float64(len(p.fverts)))
	addFace := func(a, b, c r3.Vec) {
		nn, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			return
		}
		if nn.Dot(g.Sub(a)) > 0 {
			nn = nn.Scale(-1)
		}
		p.faces = append(p.faces, face{nn, nn.Dot(a)})
	}
	for k := range n {
		addFace(p.fverts[k], p.fverts[(k+1)%n], p.fverts[n+k])
	}
	addFace(p.fverts[0], p.fverts[1], p.fverts[2])
	addFace(p.fverts[n], p.fverts[n+1], p.fverts[n+2])
	lo, hi := p.fverts[0], p.fverts[0]
	for _, v := range p.fverts {
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	inf := math.Inf(1)
	p.box = [2]r3.Vec{
		{X: math.Nextafter(lo.X, -inf), Y: math.Nextafter(lo.Y, -inf), Z: math.Nextafter(lo.Z, -inf)},
		{X: math.Nextafter(hi.X, inf), Y: math.Nextafter(hi.Y, inf), Z: math.Nextafter(hi.Z, inf)},
	}
}
