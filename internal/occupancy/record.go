package occupancy

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Leaf is one leaf of a Union tree (agent-docs/body-refine-design.md §4): a
// true solid C, held as outer convex pieces that together contain it, the
// shape that bounds how far a held facet can poke out of it, and Delta, the
// published two-sided bound of the held mesh the Boolean used (C ⊂ A'^{+Delta}).
type Leaf struct {
	Pieces []*Piece
	Delta  float64
	shape  shape
	box    [2]r3.Vec
	// operand names the one held mesh this leaf came in with: leaves of one
	// operand (a composite sweep's spans) share it, and their joins are not
	// crossings of two Boolean operands.
	operand *int
}

// shape bounds, for a held triangle, how far any of its points lies outside
// the leaf's true solid: +Inf when it cannot say.
type shape interface {
	poke(tri [3]r3.Vec) float64
}

// Record is the occupancy record of a held Union result X' of the true union
// X of its leaves (§5). The union of the held leaf meshes lies within Drift of
// X' (∪ A'_i ⊂ X'^{+Drift}), and Cross bounds how deep inside X' a point
// outside X can sit near a crossing of two held operands, accumulated over
// every step. A Record is immutable once built.
type Record struct {
	Leaves []Leaf
	Drift  float64
	Cross  float64
	// Mesh is the held mesh the record was proven for; a leaf's own record
	// leaves it zero, since the Boolean that composes it binds the result.
	Mesh Binding
}

// LeafRecord is the record of a set of leaves whose union one held mesh
// stands for, as a composite sweep's spans do.
func LeafRecord(leaves ...Leaf) *Record {
	operand := new(int)
	out := &Record{Leaves: append([]Leaf(nil), leaves...)}
	for i := range out.Leaves {
		out.Leaves[i].operand = operand
	}
	return out
}

// Bulges returns λ(t) for every triangle of a held mesh of r's union
// (§5): every point of X' outside X lies within λ(t) of some facet t. A
// facet's bound is the smallest of its own published two-sided bound delta[t],
// which already says that every held point lies within it of the true
// boundary, and of the largest of two readings of the leaves: how far the
// facet pokes out of the nearest leaf (the least over the leaves near it),
// and, for a facet near leaves of two different operands, the accumulated
// crossing depth: every earlier crossing lies near leaves of the two operands
// that met there.
func (r *Record) Bulges(verts []r3.Vec, tris [][3]int, delta []float64) []float64 {
	out := make([]float64, len(tris))
	for t, tri := range tris {
		tv := [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
		lo, hi := triBox(tv)
		poke := math.Inf(1)
		var first *int
		two := false
		for _, l := range r.Leaves {
			if !l.near(lo, hi, r.Cross+l.Delta) {
				continue
			}
			if first == nil {
				first = l.operand
			} else if l.operand != first {
				two = true
			}
			poke = math.Min(poke, l.shape.poke(tv))
		}
		lam := poke
		if two {
			lam = math.Max(lam, r.Cross)
		}
		out[t] = math.Min(delta[t], lam)
	}
	return out
}

// Compose is the record of round(X'_a ∪ X'_b) for one mesh Union step.
// weld is the step's largest vertex rounding. deepTerm is the largest, over
// the step's contact segments, of 2·max(λ_a(t), λ_b(u))/sin θ(t, u): a point
// shallow in both held operands but deep in their union lies within that of a
// crossing line on the union's boundary. That holds only when every pair of
// held facets that come that close without a proven crossing was refused,
// which the caller checks against the hidden-tangency gate before calling.
// Compose answers nil when deepTerm is not finite.
func Compose(a, b *Record, weld, deepTerm float64) *Record {
	if a == nil || b == nil || proofbound.IsNonFinite(weld) || weld < 0 ||
		proofbound.IsNonFinite(deepTerm) || deepTerm < 0 {
		return nil
	}
	return &Record{
		Leaves: append(append(make([]Leaf, 0, len(a.Leaves)+len(b.Leaves)), a.Leaves...), b.Leaves...),
		Drift:  proofbound.AbsSumUpper(max(a.Drift, b.Drift), weld),
		Cross:  proofbound.AbsSumUpper(max(a.Cross, b.Cross, deepTerm), weld),
	}
}

// Bounds are a record's figures over one held mesh of its union: every held
// point lies within Excess of X, and every true boundary point within
// TrueToHeld of the held boundary (§5 (i), (ii)).
type Bounds struct {
	Excess, TrueToHeld float64
}

// MeshBounds reads Bounds from the bulges of the record's own held mesh. A
// point of X' outside X lies within λ of a facet whose points lie within λ of
// X, so Excess is twice the largest bulge; a true boundary point inside X'
// has a point of X' outside X arbitrarily near, so it lies within the largest
// bulge of the held boundary.
func (r *Record) MeshBounds(bulges []float64) Bounds {
	lam, d := 0.0, 0.0
	for _, b := range bulges {
		lam = max(lam, b)
	}
	for _, l := range r.Leaves {
		d = max(d, l.Delta)
	}
	return Bounds{
		Excess:     proofbound.AbsSumUpper(lam, lam),
		TrueToHeld: max(proofbound.AbsSumUpper(d, r.Drift), lam),
	}
}

// Pieces returns every leaf's outer pieces.
func (r *Record) Pieces() []*Piece {
	var out []*Piece
	for _, l := range r.Leaves {
		out = append(out, l.Pieces...)
	}
	return out
}

// near reports whether the box (lo, hi) comes within pad of one of the leaf's
// pieces: the leaf's true solid lies inside them, so a facet farther from
// every piece is farther than pad from the leaf.
func (l Leaf) near(lo, hi r3.Vec, pad float64) bool {
	if !boxesNear(lo, hi, l.box, pad) {
		return false
	}
	for _, p := range l.Pieces {
		if boxesNear(lo, hi, p.box, pad) {
			return true
		}
	}
	return false
}

func newLeaf(pieces []*Piece, delta float64, s shape) Leaf {
	box := pieces[0].box
	for _, p := range pieces[1:] {
		box = [2]r3.Vec{
			{X: math.Min(box[0].X, p.box[0].X), Y: math.Min(box[0].Y, p.box[0].Y), Z: math.Min(box[0].Z, p.box[0].Z)},
			{X: math.Max(box[1].X, p.box[1].X), Y: math.Max(box[1].Y, p.box[1].Y), Z: math.Max(box[1].Z, p.box[1].Z)},
		}
	}
	return Leaf{Pieces: pieces, Delta: delta, shape: s, box: box}
}

// CircleLeaf builds the leaf of the solid cylinder that place maps from
// (u−cu)² + (v−cv)² ≤ r², z0 ≤ z ≤ z1, with the record's section
// displacement sectionDelta and level displacements dz0, dz1 widening it.
// delta is the published bound of the held mesh the Boolean used. slack
// widens the piece past any rounding of the record's own maps beyond the
// exact image of its floats.
func CircleLeaf(place Chain, cu, cv, r, z0, z1, sectionDelta, dz0, dz1, slack float64, sides int,
	delta float64) (Leaf, error) {
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
	cyl, err := newCylinderShape(shifted, r, z0, z1, proofbound.AbsSumUpper(sectionDelta, dz0, dz1))
	if err != nil {
		return Leaf{}, err
	}
	return newLeaf([]*Piece{piece}, delta, cyl), nil
}

// cylinderShape reads how far a held triangle pokes out of a placed solid
// cylinder: each corner is taken back to local coordinates exactly, its radial
// and axial excess bounded from above, and the local excess stretched by the
// maps' Frobenius norms. Both excesses are convex functions, so the largest
// corner reading bounds the whole triangle.
type cylinderShape struct {
	inv          []exactAffine
	stretch      float64
	r, z0, z1    float64
	displacement float64
}

func newCylinderShape(place Chain, r, z0, z1, displacement float64) (cylinderShape, error) {
	inv := make([]exactAffine, len(place))
	stretch := 1.0
	for i, a := range place {
		e, err := invert(a)
		if err != nil {
			return cylinderShape{}, err
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
	return cylinderShape{inv: inv, stretch: stretch, r: r, z0: z0, z1: z1, displacement: displacement}, nil
}

func (c cylinderShape) poke(tri [3]r3.Vec) float64 {
	rr := new(big.Rat).Mul(ratOf(c.r), ratOf(c.r))
	twoR := new(big.Rat).Mul(big.NewRat(2, 1), ratOf(c.r))
	worst := 0.0
	for _, w := range tri {
		p := xvecOf(w)
		for _, e := range c.inv {
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
		if d := new(big.Rat).Sub(ratOf(c.z0), p[2]); d.Sign() > 0 {
			axial = ratUpper(d)
		}
		if d := new(big.Rat).Sub(p[2], ratOf(c.z1)); d.Sign() > 0 {
			axial = max(axial, ratUpper(d))
		}
		worst = max(worst, proofbound.AbsSumUpper(radial, axial))
	}
	return proofbound.AbsSumUpper(proofbound.UpRound(worst*c.stretch), c.displacement)
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
