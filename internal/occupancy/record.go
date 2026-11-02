package occupancy

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Leaf is one convex leaf of a Union tree (agent-docs/body-refine-design.md
// §4): Piece contains its true solid C; Rho bounds how far the held mesh the
// Boolean used lies outside C (A' ⊂ C^{+Rho}); Delta is that mesh's published
// two-sided bound (C ⊂ A'^{+Delta}).
type Leaf struct {
	Piece *Piece
	Rho   float64
	Delta float64
}

// Record is the occupancy record of a held Union result X' of the true union
// X of its leaves (§5): X' ⊂ X^{+Excess}; the union of the held leaf meshes
// lies within Drift of X' (∪ A'_i ⊂ X'^{+Drift}); and every point of X' deeper
// than Deep lies in X. A Record is immutable once built.
type Record struct {
	Leaves []Leaf
	Excess float64
	Drift  float64
	Deep   float64
	// Mesh is the held mesh the record was proven for; a leaf's own record
	// leaves it zero, since the Boolean that composes it binds the result.
	Mesh Binding
}

// LeafRecord is the record of one leaf standing alone: its held mesh lies
// within Rho of C, the leaf itself is the union, and a point of a convex
// solid's held mesh deeper than Rho cannot lie outside C (a ball inside
// C^{+Rho} around a point outside C has radius at most Rho).
func LeafRecord(l Leaf) *Record {
	return &Record{Leaves: []Leaf{l}, Excess: l.Rho, Deep: l.Rho}
}

// Compose is the record of round(X'_a ∪ X'_b) for one mesh Union step:
// weld is the step's largest vertex rounding, minSin its smallest proven
// crossing sine (+Inf when no segment crosses), and gateSlack the smallest
// hidden-tangency slack the step's pre-pass applied to any face pair. It
// answers nil, a record not proven, when that slack does not exceed twice the
// operands' deep bound: two held surfaces that come within that much of each
// other without a proven crossing are then not certain to have been refused,
// and §5's deep-point argument needs them refused.
func Compose(a, b *Record, weld, minSin, gateSlack float64) *Record {
	if a == nil || b == nil || proofbound.IsNonFinite(weld) || weld < 0 || math.IsNaN(minSin) || !(minSin > 0) {
		return nil
	}
	deep := max(a.Deep, b.Deep)
	if !(gateSlack > proofbound.AbsSumUpper(deep, deep)) {
		return nil
	}
	// A point shallow in both held operands but deep in their union lies within
	// deep of both held surfaces, so within 2·deep/sin θ of a crossing line,
	// which lies on the union's boundary.
	near := deep
	if !math.IsInf(minSin, 1) {
		near = proofbound.DivUpper(proofbound.AbsSumUpper(deep, deep), minSin)
	}
	out := &Record{
		Leaves: append(append(make([]Leaf, 0, len(a.Leaves)+len(b.Leaves)), a.Leaves...), b.Leaves...),
		Excess: proofbound.AbsSumUpper(max(a.Excess, b.Excess), weld),
		Drift:  proofbound.AbsSumUpper(max(a.Drift, b.Drift), weld),
		Deep:   proofbound.AbsSumUpper(near, weld),
	}
	if proofbound.IsNonFinite(out.Deep) {
		return nil
	}
	return out
}

// TrueToHeld is §5 (ii)'s bound H: every point of the true union's boundary
// lies within it of the held mesh's boundary.
func (r *Record) TrueToHeld() float64 {
	d := 0.0
	for _, l := range r.Leaves {
		d = max(d, l.Delta)
	}
	return max(proofbound.AbsSumUpper(d, r.Drift), r.Deep)
}

// Pieces returns the leaves' outer pieces.
func (r *Record) Pieces() []*Piece {
	out := make([]*Piece, len(r.Leaves))
	for i, l := range r.Leaves {
		out[i] = l.Piece
	}
	return out
}

// CircleLeaf builds the leaf of the solid cylinder that place maps from
// (u−cu)² + (v−cv)² ≤ r², z0 ≤ z ≤ z1, with the record's section
// displacement sectionDelta and level displacements dz0, dz1 widening it.
// held are the vertices of the mesh the Boolean used and delta its published
// bound. slack widens the piece past any rounding of the record's own maps
// beyond the exact image of its floats.
func CircleLeaf(place Chain, cu, cv, r, z0, z1, sectionDelta, dz0, dz1, slack float64, sides int,
	held []r3.Vec, delta float64) (Leaf, error) {
	if proofbound.IsNonFinite(sectionDelta) || proofbound.IsNonFinite(dz0) || proofbound.IsNonFinite(dz1) ||
		proofbound.IsNonFinite(slack) || proofbound.IsNonFinite(delta) || sectionDelta < 0 || dz0 < 0 || dz1 < 0 || slack < 0 || delta < 0 {
		return Leaf{}, fmt.Errorf("%w: circle leaf displacement", ErrInvalid)
	}
	shifted := append(Chain{{M: [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, T: [3]float64{cu, cv, 0}}}, place...)
	piece, err := CirclePrism(shifted,
		proofbound.AbsSumUpper(r, sectionDelta, slack),
		z0-proofbound.AbsSumUpper(dz0, slack), z1+proofbound.AbsSumUpper(dz1, slack), sides)
	if err != nil {
		return Leaf{}, err
	}
	rho, err := outsideCylinder(shifted, r, z0, z1, held)
	if err != nil {
		return Leaf{}, err
	}
	rho = proofbound.AbsSumUpper(rho, sectionDelta, dz0, dz1)
	return Leaf{Piece: piece, Rho: rho, Delta: delta}, nil
}

// outsideCylinder bounds, in world millimetres, how far any of held lies
// outside the image of the cylinder: each vertex is taken back to local
// coordinates exactly, its radial and axial excess bounded from above, and
// the local excess stretched by the maps' Frobenius norms.
func outsideCylinder(place Chain, r, z0, z1 float64, held []r3.Vec) (float64, error) {
	inv := make([]exactAffine, len(place))
	stretch := 1.0
	for i, a := range place {
		e, err := invert(a)
		if err != nil {
			return 0, err
		}
		inv[len(place)-1-i] = e
		f := 0.0
		for _, row := range a.M {
			for _, x := range row {
				f += x * x
			}
		}
		stretch = proofbound.UpRound(stretch * proofbound.UpRound(math.Sqrt(proofbound.UpRound(f))*(1+1e-15)))
	}
	rr := new(big.Rat).Mul(ratOf(r), ratOf(r))
	twoR := new(big.Rat).Mul(big.NewRat(2, 1), ratOf(r))
	worst := 0.0
	for _, w := range held {
		p := xvecOf(w)
		for _, e := range inv {
			p = e.apply(p)
		}
		q := new(big.Rat).Mul(p[0], p[0])
		q.Add(q, new(big.Rat).Mul(p[1], p[1]))
		radial := 0.0
		if q.Cmp(rr) > 0 {
			// sqrt(q) − r = (q − r²)/(sqrt(q) + r) ≤ (q − r²)/(2r).
			radial = ratUpper(new(big.Rat).Quo(q.Sub(q, rr), twoR))
		}
		axial := 0.0
		if d := new(big.Rat).Sub(ratOf(z0), p[2]); d.Sign() > 0 {
			axial = ratUpper(d)
		}
		if d := new(big.Rat).Sub(p[2], ratOf(z1)); d.Sign() > 0 {
			axial = max(axial, ratUpper(d))
		}
		worst = max(worst, proofbound.AbsSumUpper(radial, axial))
	}
	return proofbound.UpRound(worst * stretch), nil
}

func ratUpper(x *big.Rat) float64 {
	f, exact := x.Float64()
	if !exact && new(big.Rat).SetFloat64(f).Cmp(x) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// exactAffine is a rational affine map.
type exactAffine struct {
	m [3][3]*big.Rat
	t xvec
}

func (e exactAffine) apply(p xvec) xvec {
	var out xvec
	for i := range 3 {
		s := new(big.Rat).Set(e.t[i])
		for j := range 3 {
			s.Add(s, new(big.Rat).Mul(e.m[i][j], p[j]))
		}
		out[i] = s
	}
	return out
}

func (e exactAffine) float() Affine {
	var out Affine
	for i := range 3 {
		for j := range 3 {
			out.M[i][j], _ = e.m[i][j].Float64()
		}
		out.T[i], _ = e.t[i].Float64()
	}
	return out
}

func (a Affine) applyFloat(v r3.Vec) r3.Vec {
	p := [3]float64{v.X, v.Y, v.Z}
	var q [3]float64
	for i := range 3 {
		q[i] = a.M[i][0]*p[0] + a.M[i][1]*p[1] + a.M[i][2]*p[2] + a.T[i]
	}
	return r3.Vec{X: q[0], Y: q[1], Z: q[2]}
}

// invert returns the exact inverse of a's map, by the adjugate.
func invert(a Affine) (exactAffine, error) {
	var m [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			m[i][j] = ratOf(a.M[i][j])
		}
	}
	cof := func(i, j int) *big.Rat {
		r0, r1 := (i+1)%3, (i+2)%3
		c0, c1 := (j+1)%3, (j+2)%3
		return new(big.Rat).Sub(new(big.Rat).Mul(m[r0][c0], m[r1][c1]), new(big.Rat).Mul(m[r0][c1], m[r1][c0]))
	}
	det := new(big.Rat)
	for j := range 3 {
		det.Add(det, new(big.Rat).Mul(m[0][j], cof(0, j)))
	}
	if det.Sign() == 0 {
		return exactAffine{}, fmt.Errorf("%w: singular leaf placement", decaderr.ErrDegenerate)
	}
	var out exactAffine
	for i := range 3 {
		for j := range 3 {
			out.m[i][j] = new(big.Rat).Quo(cof(j, i), det)
		}
	}
	for i := range 3 {
		s := new(big.Rat)
		for j := range 3 {
			s.Sub(s, new(big.Rat).Mul(out.m[i][j], ratOf(a.T[j])))
		}
		out.t[i] = s
	}
	return out, nil
}

// Binding ties a Record to the one held mesh it was proven for: the identity
// of that mesh's vertex and triangle arrays. Any producer that changes the
// held geometry (placement, a modify rewrite) allocates new arrays, so a
// record copied along with the rest of a payload no longer matches and is
// never read for it.
type Binding struct {
	verts *r3.Vec
	tris  *[3]int
	nv    int
	nt    int
}

// Bind returns the Binding of a held mesh.
func Bind(verts []r3.Vec, tris [][3]int) Binding {
	if len(verts) == 0 || len(tris) == 0 {
		return Binding{}
	}
	return Binding{&verts[0], &tris[0], len(verts), len(tris)}
}

// Matches reports whether verts and tris are the arrays b was bound to.
func (b Binding) Matches(verts []r3.Vec, tris [][3]int) bool {
	return b.verts != nil && len(verts) == b.nv && len(tris) == b.nt && &verts[0] == b.verts && &tris[0] == b.tris
}
