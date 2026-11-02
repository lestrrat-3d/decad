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

func dotExact(a r3.Vec, p xvec) *big.Rat {
	s := new(big.Rat).Mul(ratOf(a.X), p[0])
	s.Add(s, new(big.Rat).Mul(ratOf(a.Y), p[1]))
	return s.Add(s, new(big.Rat).Mul(ratOf(a.Z), p[2]))
}

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
	type line struct{ ux, uy, c *big.Rat }
	r2 := new(big.Rat).Mul(ratOf(r), ratOf(r))
	lines := make([]line, sides)
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
		lines[k] = line{rx, ry, ratOf(c)}
	}
	// Corner k is where line k meets line k+1, solved exactly by Cramer's rule.
	corners := make([][2]*big.Rat, sides)
	for k := range corners {
		a, b := lines[k], lines[(k+1)%sides]
		det := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.uy), new(big.Rat).Mul(a.uy, b.ux))
		if det.Sign() <= 0 {
			return nil, fmt.Errorf("%w: sides %d and %d do not turn left", decaderr.ErrUnsupported, k, (k+1)%sides)
		}
		x := new(big.Rat).Sub(new(big.Rat).Mul(a.c, b.uy), new(big.Rat).Mul(a.uy, b.c))
		y := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.c), new(big.Rat).Mul(a.c, b.ux))
		corners[k] = [2]*big.Rat{x.Quo(x, det), y.Quo(y, det)}
	}
	// Every corner must satisfy every half-plane: then the corners' hull is
	// the half-plane intersection, which contains the circle.
	for k, p := range corners {
		for j, l := range lines {
			s := new(big.Rat).Mul(l.ux, p[0])
			s.Add(s, new(big.Rat).Mul(l.uy, p[1]))
			if s.Cmp(l.c) > 0 {
				return nil, fmt.Errorf("%w: corner %d violates side %d", decaderr.ErrUnsupported, k, j)
			}
		}
	}
	p := &Piece{z0: ratOf(z0), z1: ratOf(z1), fz0: z0, fz1: z1}
	for _, l := range lines {
		fux, _ := l.ux.Float64()
		fuy, _ := l.uy.Float64()
		fc, _ := l.c.Float64()
		p.sides = append(p.sides, side{l.ux, l.uy, l.c, fux, fuy, fc})
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
	n := sides
	for k := range n {
		p.edges = append(p.edges, [2]int{k, (k + 1) % n}, [2]int{n + k, n + (k+1)%n}, [2]int{k, n + k})
	}
	p.finishFaces(n)
	return p, nil
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
			d := new(big.Rat).Mul(sd.ux, x[0])
			d.Add(d, new(big.Rat).Mul(sd.uy, x[1]))
			if d.Cmp(sd.c) > 0 {
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
