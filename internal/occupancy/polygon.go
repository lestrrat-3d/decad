package occupancy

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PolygonLeaf builds the leaf of a placed straight prism whose section is the
// convex polygon pts (either winding), between levels z0 and z1, widened by
// the record's section and level displacements and slack. Its one outer piece
// is the prism itself, each side pushed out by that widening. delta is the
// published bound of the held mesh the Boolean used.
func PolygonLeaf(place Chain, pts [][2]float64, z0, z1, sectionDelta, dz0, dz1, slack, delta float64) (Leaf, error) {
	n := len(pts)
	if n < 3 || slices.ContainsFunc([]float64{z0, z1, sectionDelta, dz0, dz1, slack, delta}, proofbound.IsNonFinite) ||
		!(z0 < z1) || sectionDelta < 0 || dz0 < 0 || dz1 < 0 || slack < 0 || delta < 0 {
		return Leaf{}, fmt.Errorf("%w: polygon leaf", ErrInvalid)
	}
	xs := make([][2]*big.Rat, n)
	for i, p := range pts {
		if proofbound.IsNonFinite(p[0]) || proofbound.IsNonFinite(p[1]) {
			return Leaf{}, fmt.Errorf("%w: polygon corner", ErrInvalid)
		}
		xs[i] = [2]*big.Rat{ratOf(p[0]), ratOf(p[1])}
	}
	// Exact signed area fixes the winding; reverse to counter-clockwise.
	area := new(big.Rat)
	for i := range xs {
		a, b := xs[i], xs[(i+1)%n]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])))
	}
	if area.Sign() == 0 {
		return Leaf{}, fmt.Errorf("%w: polygon of no area", ErrInvalid)
	}
	if area.Sign() < 0 {
		slices.Reverse(xs)
		pts = slices.Clone(pts)
		slices.Reverse(pts)
	}
	widen := proofbound.AbsSumUpper(sectionDelta, slack)
	lines := make([]side, n)
	inward := make([]side, n)
	for i := range xs {
		a, b := xs[i], xs[(i+1)%n]
		ux := new(big.Rat).Sub(b[1], a[1])
		uy := new(big.Rat).Sub(a[0], b[0])
		base := new(big.Rat).Mul(ux, a[0])
		base.Add(base, new(big.Rat).Mul(uy, a[1]))
		// Push the side out, or in, by at least widen: |u| ≤ |ux| + |uy|.
		l1 := new(big.Rat).Add(new(big.Rat).Abs(ux), new(big.Rat).Abs(uy))
		push := new(big.Rat).Mul(l1, ratOf(widen))
		fux, _ := ux.Float64()
		fuy, _ := uy.Float64()
		c := new(big.Rat).Add(base, push)
		fc, _ := c.Float64()
		lines[i] = side{ux, uy, c, fux, fuy, fc}
		ci := new(big.Rat).Sub(base, push)
		fci, _ := ci.Float64()
		inward[i] = side{ux, uy, ci, fux, fuy, fci}
	}
	// Convex: every corner on or inside every side.
	for i, p := range xs {
		for j, l := range lines {
			d := new(big.Rat).Mul(l.ux, p[0])
			d.Add(d, new(big.Rat).Mul(l.uy, p[1]))
			if d.Cmp(l.c) > 0 {
				return Leaf{}, fmt.Errorf("%w: polygon corner %d lies outside side %d: not convex", decaderr.ErrUnsupported, i, j)
			}
		}
	}
	lo := z0 - proofbound.AbsSumUpper(dz0, slack)
	hi := z1 + proofbound.AbsSumUpper(dz1, slack)
	shape, err := newPolygonShape(place, pts, z0, z1, proofbound.AbsSumUpper(sectionDelta, dz0, dz1))
	if err != nil {
		return Leaf{}, err
	}
	scale := 0.0
	var corners []r3.Vec
	for _, p := range pts {
		for _, z := range []float64{lo, hi} {
			v := place.applyFloat(r3.Vec{X: p[0], Y: p[1], Z: z})
			corners = append(corners, v)
			scale = math.Max(scale, v.Len())
		}
	}
	// One cell: the corners' box, widened by the side push and float slack.
	box := [2]r3.Vec{corners[0], corners[0]}
	for _, v := range corners[1:] {
		box = [2]r3.Vec{
			{X: math.Min(box[0].X, v.X), Y: math.Min(box[0].Y, v.Y), Z: math.Min(box[0].Z, v.Z)},
			{X: math.Max(box[1].X, v.X), Y: math.Max(box[1].Y, v.Y), Z: math.Max(box[1].Z, v.Z)},
		}
	}
	pad := 2*widen*shape.stretch + relSlack*scale + 1e-9*scale
	box = [2]r3.Vec{box[0].Sub(r3.Vec{X: pad, Y: pad, Z: pad}), box[1].Add(r3.Vec{X: pad, Y: pad, Z: pad})}
	piece := &lazyPiece{box: box, build: func() (*Piece, error) {
		return linesPrism(place, lines, lo, hi)
	}}
	l := newLeaf([][2]r3.Vec{box}, []*lazyPiece{piece}, delta, shape)
	l.key = fmt.Sprintf("polygon %v %v %v %v %v %v %v %v", place, pts, z0, z1, sectionDelta, dz0, dz1, slack)
	// Inside: every side pushed in by at least the widening, between the
	// levels narrowed the same way, as CircleLeaf narrows its circle. A
	// recorded side lies within sectionDelta of the denoted section and the
	// slack covers the maps' rounding, so the piece lies in the true prism
	// and a Cut may take the leaf as its tool. A prism too thin for the
	// narrowed sides to close offers no piece.
	ilo, ihi := z0+proofbound.AbsSumUpper(dz0, slack), z1-proofbound.AbsSumUpper(dz1, slack)
	if ilo < ihi {
		l.inner = &lazyPieces{build: func() ([]*Piece, error) {
			p, err := linesPrism(place, inward, ilo, ihi)
			if errors.Is(err, decaderr.ErrUnsupported) {
				return nil, nil
			}
			if err != nil {
				return nil, err
			}
			return []*Piece{p}, nil
		}}
	}
	return l, nil
}

// linesPrism is the exact convex piece of the half-planes lines (in order
// around the section) between levels lo and hi, under place.
func linesPrism(place Chain, lines []side, lo, hi float64) (*Piece, error) {
	n := len(lines)
	corners := make([][2]*big.Rat, n)
	for k := range corners {
		a, b := lines[k], lines[(k+1)%n]
		det := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.uy), new(big.Rat).Mul(a.uy, b.ux))
		if det.Sign() <= 0 {
			return nil, fmt.Errorf("%w: sides %d and %d do not turn left", decaderr.ErrUnsupported, k, (k+1)%n)
		}
		x := new(big.Rat).Sub(new(big.Rat).Mul(a.c, b.uy), new(big.Rat).Mul(a.uy, b.c))
		y := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.c), new(big.Rat).Mul(a.c, b.ux))
		corners[k] = [2]*big.Rat{x.Quo(x, det), y.Quo(y, det)}
	}
	for k, p := range corners {
		for j, l := range lines {
			s := new(big.Rat).Mul(l.ux, p[0])
			s.Add(s, new(big.Rat).Mul(l.uy, p[1]))
			if s.Cmp(l.c) > 0 {
				return nil, fmt.Errorf("%w: corner %d violates side %d", decaderr.ErrUnsupported, k, j)
			}
		}
	}
	p := &Piece{z0: ratOf(lo), z1: ratOf(hi), fz0: lo, fz1: hi, sides: lines}
	for _, a := range slices.Backward(place) {
		e, err := invert(a)
		if err != nil {
			return nil, err
		}
		p.inv = append(p.inv, e)
		p.finv = append(p.finv, e.float())
	}
	for _, z := range []float64{lo, hi} {
		rz := ratOf(z)
		for _, c := range corners {
			v := place.applyExact(xvec{c[0], c[1], rz})
			p.verts = append(p.verts, v)
			p.fverts = append(p.fverts, v.float())
		}
	}
	for k := range n {
		p.edges = append(p.edges, [2]int{k, (k + 1) % n}, [2]int{n + k, n + (k+1)%n}, [2]int{k, n + k})
	}
	p.finishFaces(n)
	return p, nil
}

// polygonShape reads how far a held triangle pokes out of a placed convex
// polygon prism: each corner is taken to local coordinates through the maps'
// float inverse, its excess past each side and level read there; the
// distance outside a convex polygon is at most the sum of its side excesses
// over the sine of its sharpest corner, and that sum is convex, so the
// corners bound the facet. The float reading is charged relSlack of its
// magnitude, as the tube readings are.
type polygonShape struct {
	finv         []Affine
	stretch      float64
	nx, ny, c    []float64
	sinMin       float64
	z0, z1       float64
	displacement float64
}

func newPolygonShape(place Chain, pts [][2]float64, z0, z1, displacement float64) (polygonShape, error) {
	s := polygonShape{z0: z0, z1: z1, displacement: displacement, stretch: 1, sinMin: 1}
	for _, a := range place {
		e, err := invert(a)
		if err != nil {
			return polygonShape{}, err
		}
		s.finv = append([]Affine{e.float()}, s.finv...)
		s.stretch = proofbound.UpRound(s.stretch * opNormUpper(a.M))
	}
	n := len(pts)
	for i := range pts {
		a, b := pts[i], pts[(i+1)%n]
		dx, dy := b[0]-a[0], b[1]-a[1]
		l := math.Hypot(dx, dy)
		if !(l > 0) {
			return polygonShape{}, fmt.Errorf("%w: polygon side of no length", ErrInvalid)
		}
		nx, ny := dy/l, -dx/l
		s.nx, s.ny, s.c = append(s.nx, nx), append(s.ny, ny), append(s.c, nx*a[0]+ny*a[1])
	}
	for i := range n {
		j := (i + 1) % n
		// The sine between consecutive outward normals.
		sn := math.Abs(s.nx[i]*s.ny[j] - s.ny[i]*s.nx[j])
		s.sinMin = math.Min(s.sinMin, sn)
	}
	if !(s.sinMin > 1e-6) {
		return polygonShape{}, fmt.Errorf("%w: a polygon corner is too sharp to bound", decaderr.ErrUnsupported)
	}
	return s, nil
}

func (s polygonShape) poke(tri [3]r3.Vec) float64 {
	worst, scale := 0.0, 0.0
	for _, w := range tri {
		p := w
		for _, a := range s.finv {
			p = a.applyFloat(p)
		}
		scale = math.Max(scale, w.Len()+p.Len())
		sum := 0.0
		for k := range s.nx {
			sum += math.Max(0, s.nx[k]*p.X+s.ny[k]*p.Y-s.c[k])
		}
		axial := math.Max(0, math.Max(s.z0-p.Z, p.Z-s.z1))
		worst = math.Max(worst, sum/s.sinMin+axial)
	}
	return proofbound.AbsSumUpper(worst*s.stretch, relSlack*scale*float64(len(s.nx)), s.displacement)
}

// boundaryFar never proves a box far from a polygon prism's boundary, so a
// polygon tool's hole wall is read wherever its leaf is near: the wall's
// bound then applies to every facet near the tool, which only widens the
// bulge.
func (polygonShape) boundaryFar(r3.Vec, r3.Vec, float64) bool { return false }

// PrismRecord is the record of straight prisms whose sections are the outer
// leaves less the hole leaves, each hole a prism through its own region's
// levels: the solid a section with holes sweeps, or a stack of such slabs.
// For a stack, a hole of one slab meets another slab's outer region only on
// their shared level plane, so the union less the holes is the stack up to
// that plane, which a regularized solid ignores.
func PrismRecord(outers []Leaf, holes []Leaf) *Record {
	operand := new(int)
	out := &Record{Leaves: append([]Leaf(nil), outers...), Holes: append([]Leaf(nil), holes...), grid: &siteGrid{}}
	for i := range out.Leaves {
		out.Leaves[i].operand = operand
	}
	for i := range out.Holes {
		out.Holes[i].operand = operand
	}
	return out
}
