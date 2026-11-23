package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads the seats of an Intersect (agent-docs/coplanar-contact-design.md §16): a planar face whose true
// piece lies exactly in its plane (a carrier-exact face), resting on an opposite-facing one of the other operand, and
// the walls beside such a face that provably leave its plane into their own operand's material.

// seatSinMin is the least sine of the angle at which a held wall's rulings must meet a seat plane for the wall to
// count as crossing the plane transversally (§16.2).
const seatSinMin = 1e-3

// rimSeat is a plane a wall leaves along its rim: pf is a facet of the wall's own carrier-exact neighbour face in
// that plane. The wall's true piece lies on the side of pf's plane opposite pf's outward normal, its operand's
// material side, and meets the plane only along the rim.
type rimSeat struct {
	pf int
}

// SetSeats fills bm.Carrier and bm.RimEdge and bm.Seats from m, the tessellation bm was prepared from, for an
// unshifted Intersect (§16.1). A facet is carrier-exact when its true piece lies in the facet's own plane, though the
// face's outline may be chorded: its SlabBound is zero, or its face is an analytic plane the facet lies in exactly.
// Faces on one Cylinder or Cone are read as one wall W, which earns a seat on the plane of a neighbour face F when
// (§16.2)
//
//   - F is carrier-exact: every facet is, and every corner lies in one plane, exactly;
//   - every facet of W has a vertex off its edges with F, and every such vertex lies strictly on F's material side;
//   - W is exact (every facet exact, §3.1), or W is held within a bound, the edges W shares with F form closed loops,
//     every ruling of W's true surface meets the plane at a sine above seatSinMin, and every vertex off the rim lies
//     farther from the plane than W's largest facet bound plus the arithmetic slack.
//
// Each such edge of W is marked in RimEdge.
func (bm *BoolMesh) SetSeats(m *topo.Mesh, placed []bool) {
	n := len(bm.Tris)
	bm.Carrier = bm.carrierFacets(m, placed)
	bm.RimEdge = make([][3]bool, n)
	bm.RimVert = map[int]bool{}
	bm.Seats = map[int][]rimSeat{}
	facetsOf := map[int][]int{}
	var order []int
	for t, s := range bm.Src {
		if _, ok := facetsOf[s]; !ok {
			order = append(order, s)
		}
		facetsOf[s] = append(facetsOf[s], t)
	}
	carrierFace := map[int]bool{}
	for _, s := range order {
		carrierFace[s] = bm.carrierFace(facetsOf[s])
	}
	scale := 1.0
	for _, v := range bm.Verts {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	slack := 64 * 0x1p-52 * scale
	// Faces on one cylinder or cone, such as the two halves of a bore drawn as two arcs, are read as one wall: their
	// seams are no rim, and together their rims can close.
	groups := map[any][]int{}
	var keys []any
	for _, s := range order {
		var key any = s
		if t := facetsOf[s][0]; t < len(m.Source) && m.Source[t] != nil {
			switch sf := m.Source[t].Surface.(type) {
			case surfacegeom.Cylinder, surfacegeom.Cone:
				key = sf
			}
		}
		if _, ok := groups[key]; !ok {
			keys = append(keys, key)
		}
		groups[key] = append(groups[key], s)
	}
	for _, key := range keys {
		members := groups[key]
		var facets []int
		in := map[int]bool{}
		for _, s := range members {
			facets = append(facets, facetsOf[s]...)
			in[s] = true
		}
		if len(facets) == 0 || facets[0] >= len(m.Source) {
			continue
		}
		face := m.Source[facets[0]]
		// Edges of the wall by the face across them.
		rims := map[int][][3]int{} // neighbour face -> (facet, edge, twin)
		for _, t := range facets {
			for k := range 3 {
				twin, ok := bm.TwinFacet(t, k)
				if !ok || in[bm.Src[twin]] {
					continue
				}
				rims[bm.Src[twin]] = append(rims[bm.Src[twin]], [3]int{t, k, twin})
			}
		}
		for _, f := range order {
			edges := rims[f]
			if len(edges) == 0 || !carrierFace[f] {
				continue
			}
			pf := edges[0][2]
			if !bm.wallLeaves(face, facets, edges, pf, slack) {
				continue
			}
			for _, s := range members {
				bm.Seats[s] = append(bm.Seats[s], rimSeat{pf: pf})
			}
			for _, e := range edges {
				bm.RimEdge[e[0]][e[1]] = true
				tri := bm.Tris[e[0]]
				bm.RimVert[tri[e[1]]], bm.RimVert[tri[(e[1]+1)%3]] = true, true
			}
		}
	}
}

// carrierFacets reads, per facet of bm, whether its true piece lies in the facet's own plane (§16.1, carrierFacet).
func (bm *BoolMesh) carrierFacets(m *topo.Mesh, placed []bool) []bool {
	carrier := make([]bool, len(bm.Tris))
	planes := map[*topo.Face]*exactPlane{}
	for t := range carrier {
		carrier[t] = bm.carrierFacet(m, t, placed, planes)
	}
	return carrier
}

// carrierFacet reports whether facet t's true piece lies in the facet's own plane: its SlabBound is zero, its face is
// an unplaced analytic plane it lies in exactly, placed marks it, or a record names its face's exact plane and the
// facet lies in it (§17). planes caches each face's record plane, which is read once per face.
func (bm *BoolMesh) carrierFacet(m *topo.Mesh, t int, placed []bool, planes map[*topo.Face]*exactPlane) bool {
	if bm.SlabBound[t] == 0 || bm.analyticPlaneFacet(m, t) || t < len(placed) && placed[t] {
		return true
	}
	if t >= len(m.Source) || m.Source[t] == nil {
		return false
	}
	f := m.Source[t]
	pl, seen := planes[f]
	if !seen {
		pl = trueFacePlane(f)
		planes[f] = pl
	}
	return pl != nil && pl.holds(bm.Verts[bm.Tris[t][0]], bm.Verts[bm.Tris[t][1]], bm.Verts[bm.Tris[t][2]])
}

// exactPlane is the plane through o normal to n, both exact rationals.
type exactPlane struct {
	o, n [3]*big.Rat
}

// holds reports whether the held points lie exactly in the plane.
func (p *exactPlane) holds(pts ...r3.Vec) bool {
	for _, x := range pts {
		e := ratVec(x)
		s := new(big.Rat)
		for i := range 3 {
			s.Add(s, new(big.Rat).Mul(p.n[i], new(big.Rat).Sub(e[i], p.o[i])))
		}
		if s.Sign() != 0 {
			return false
		}
	}
	return true
}

// trueFacePlane returns the plane a Plane face's true piece lies in exactly, read from its record (§17), or nil:
//
//   - a placed retained plane (topo.Face.PlacedPlane) whose tag still equals the plane the placement proved;
//   - a denoted revolve plane: every meridian segment of its record (topo.Face.Denoted) at one exact level z₀, so the
//     true piece lies in the plane through O + z₀·W spanned by the record's E₀ and E₁, all exact point leaves.
func trueFacePlane(f *topo.Face) *exactPlane {
	tag, ok := f.Surface.(surfacegeom.Plane)
	if !ok || !f.RetainedTag {
		return nil
	}
	if f.PlacedPlane != nil && *f.PlacedPlane == tag && f.AxialDelta == 0 {
		o := ratVec(tag.Frame.Origin())
		n := ratCross(ratVec(tag.Frame.U()), ratVec(tag.Frame.V()))
		if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
			return nil
		}
		return &exactPlane{o: o, n: n}
	}
	if f.Denoted != nil {
		return denotedPlane(f.Denoted)
	}
	return nil
}

// denotedPlane reads the plane of a revolve record whose meridian runs at one exact level, or nil.
func denotedPlane(r *surfacenormal.Revolved) *exactPlane {
	if !r.Valid || r.Circular || r.Cap || len(r.Ends) == 0 {
		return nil
	}
	point := func(iv proofbound.RatInterval) *big.Rat {
		if iv.Lo == nil || iv.Hi == nil || iv.Lo.Cmp(iv.Hi) != 0 {
			return nil
		}
		return iv.Lo
	}
	vec := func(iv proofbound.IvVec3) ([3]*big.Rat, bool) {
		var out [3]*big.Rat
		for k := range 3 {
			if out[k] = point(iv[k]); out[k] == nil {
				return out, false
			}
		}
		return out, true
	}
	var level *big.Rat
	for _, seg := range r.Ends {
		for _, end := range seg {
			z := point(end[0])
			if z == nil || level != nil && z.Cmp(level) != 0 {
				return nil
			}
			level = z
		}
	}
	o, ok0 := vec(r.Origin)
	e0, ok1 := vec(r.Basis[0])
	e1, ok2 := vec(r.Basis[1])
	w, ok3 := vec(r.Basis[2])
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return nil
	}
	n := ratCross(e0, e1)
	if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
		return nil
	}
	for k := range 3 {
		o[k] = new(big.Rat).Add(o[k], new(big.Rat).Mul(level, w[k]))
	}
	return &exactPlane{o: o, n: n}
}

// analyticPlaneFacet reports whether facet t's source face is a plane of an unplaced analytic operand, whose surface is
// the face itself (a plane that is no retained tag, states no departure and a zero displacement), and the facet lies
// exactly in it: its true piece then lies in the facet's own plane (planeSlab reads zero). A placed operand denotes its
// record carried through the placement read as exact rationals, whose image the float tag only approximates; §17 reads
// it from that image instead.
func (bm *BoolMesh) analyticPlaneFacet(m *topo.Mesh, t int) bool {
	if t >= len(m.Source) || m.Source[t] == nil {
		return false
	}
	f := m.Source[t]
	if f.Body == nil || f.Body.Payload == nil || f.Body.Payload.Transform() != r3.Identity() {
		return false
	}
	pl, ok := f.Surface.(surfacegeom.Plane)
	if !ok || f.RetainedTag || f.TagDeparture != 0 || f.NormalBound != 0 || f.AxialDelta != 0 || f.Denoted != nil {
		return false
	}
	slab, ok := bm.planeSlab(t, pl.Frame)
	return ok && slab == 0
}

// facetPlane reads the side of a vertex against the plane of one facet: the sign of det[b−a, c−a, v−a], as
// proof.OrientSign reads it. The uncertain case reads the exact sign from the facet's prepared lift and normal
// (BoolMesh.Xverts, BoolMesh.Norms) and the vertex's prepared lift, so nothing is lifted again.
type facetPlane struct {
	a, b, c r3.Vec
	xa, n   proof.Xpt
}

// newFacetPlane is the plane of facet t of bm.
func newFacetPlane(bm *BoolMesh, t int) facetPlane {
	p := bm.Tris[t]
	return facetPlane{a: bm.Verts[p[0]], b: bm.Verts[p[1]], c: bm.Verts[p[2]], xa: bm.Xverts[p[0]], n: bm.Norms[t]}
}

// side is the sign of vertex v of m against the plane.
func (p facetPlane) side(m *BoolMesh, v int) int {
	return proof.OrientSignPrepared(p.a, p.b, p.c, m.Verts[v], p.xa, m.Xverts[v], p.n)
}

// carrierFace reports whether every facet is carrier-exact and every corner lies in the first facet's plane. A corner
// shared by several facets is tested once.
func (bm *BoolMesh) carrierFace(facets []int) bool {
	ok, _ := bm.carrierFaceBy(facets, func(t int) bool { return bm.Carrier[t] }, nil)
	return ok
}

// carrierFaceBy is carrierFace with each facet's carrier-exactness read by carrier. A non-nil step is charged once per
// facet and once per exact sign, and its error ends the read.
func (bm *BoolMesh) carrierFaceBy(facets []int, carrier func(t int) bool, step func() error) (bool, error) {
	if len(facets) == 0 {
		return false, nil
	}
	pl := newFacetPlane(bm, facets[0])
	tested := make(map[int]struct{}, len(facets)+2)
	for _, t := range facets {
		if step != nil {
			if err := step(); err != nil {
				return false, err
			}
		}
		if !carrier(t) {
			return false, nil
		}
		for _, v := range bm.Tris[t] {
			if _, done := tested[v]; done {
				continue
			}
			tested[v] = struct{}{}
			if step != nil {
				if err := step(); err != nil {
					return false, err
				}
			}
			if pl.side(bm, v) != 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// closedLoops reports whether the edges, each (facet, edge index), meet each vertex they touch exactly twice: their
// union is closed loops.
func closedLoops(bm *BoolMesh, edges [][3]int) bool {
	degree := map[int]int{}
	for _, e := range edges {
		t := bm.Tris[e[0]]
		degree[t[e[1]]]++
		degree[t[(e[1]+1)%3]]++
	}
	for _, d := range degree {
		if d != 2 {
			return false
		}
	}
	return true
}

// wallLeaves reports whether wall W, the face whose facets are given, leaves the plane of facet pf into pf's
// operand's material side along the rim edges, as SetSeats states.
func (bm *BoolMesh) wallLeaves(face *topo.Face, facets []int, edges [][3]int, pf int, slack float64) bool {
	if face == nil {
		return false
	}
	p := bm.Tris[pf]
	a, b, c := bm.Verts[p[0]], bm.Verts[p[1]], bm.Verts[p[2]]
	normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
	if !ok {
		return false
	}
	// A facet with no vertex off the rim would lie in the plane, beside F rather than leaving it.
	onRim := map[int]bool{}
	for _, e := range edges {
		t := bm.Tris[e[0]]
		onRim[t[e[1]]], onRim[t[(e[1]+1)%3]] = true, true
	}
	for _, t := range facets {
		tri := bm.Tris[t]
		if onRim[tri[0]] && onRim[tri[1]] && onRim[tri[2]] {
			return false
		}
	}
	// An exact wall is its held facets: its other vertices need only lie strictly on the material side.
	exact := true
	for _, t := range facets {
		exact = exact && bm.isExact(t)
	}
	// Each vertex off the rim is tested once, however many facets share it.
	pl := newFacetPlane(bm, pf)
	tested := make(map[int]struct{}, len(facets)+2)
	if exact {
		for _, t := range facets {
			for _, v := range bm.Tris[t] {
				if onRim[v] {
					continue
				}
				if _, done := tested[v]; done {
					continue
				}
				tested[v] = struct{}{}
				if pl.side(bm, v) >= 0 {
					return false
				}
			}
		}
		return true
	}
	// A held wall leaves the plane by its surface: transversal rulings along closed rims, and every other vertex
	// farther than its bound.
	if !closedLoops(bm, edges) || !wallTransversal(face, a, b, c, normal) {
		return false
	}
	beta := 0.0
	for _, t := range facets {
		beta = max(beta, bm.FacetBound[t])
	}
	for _, t := range facets {
		for _, v := range bm.Tris[t] {
			if onRim[v] {
				continue
			}
			if _, done := tested[v]; done {
				continue
			}
			tested[v] = struct{}{}
			// The material side of pf is opposite its outward normal: OrientSign reads it negative.
			if pl.side(bm, v) >= 0 {
				return false
			}
			if !(-normal.Dot(bm.Verts[v].Sub(a)) > beta+slack) {
				return false
			}
		}
	}
	return true
}

// wallTransversal reports whether every ruling of the wall's true surface meets the plane of a, b, c at a sine
// above seatSinMin (§16.2). A face that records the surface its revolve denotes (topo.Face.Denoted) is read from
// that record, whose rational leaves enclose the true surface; a revolve cone's tag is a float re-expression with no
// stated normal bound (retained-modify-reach §9.5), so the record is the only exact reading of it. Any other face is
// read from its Cylinder or Cone tag, whose unit normal departs from the true one by at most NormalBound.
func wallTransversal(face *topo.Face, a, b, c, normal r3.Vec) bool {
	if face.Denoted != nil {
		return denotedTransversal(face.Denoted, a, b, c)
	}
	d := face.NormalBound
	if !(d >= 0) || !(d < 2) {
		return false
	}
	// Unit vectors a distance d apart are 2·asin(d/2) apart in angle; the factor pads the float rounding of that and
	// of transversal's own trigonometry, all far below seatSinMin.
	return transversal(face.Surface, normal, 2*math.Asin(d/2)*(1+1e-9)+1e-12)
}

// denotedTransversal bounds the rulings of the revolve record r against the plane of a, b, c in exact rational
// arithmetic. A ruling of a meridian segment from (z₀, ρ₀) to (z₁, ρ₁) runs along
//
//	d(φ) = Δz·W + Δρ·(cos φ·E₀ + sin φ·E₁)
//
// for the record's axis W and basis E₀, E₁, each only known to lie in its intervals. With N = (b − a) × (c − a),
//
//	|d·N| ≥ |Δz|·|W·N| − |Δρ|·(|E₀·N| + |E₁·N|),   |d|·|N| ≤ (|Δz|·|W|₁ + |Δρ|·(|E₀|₁ + |E₁|₁))·|N|₁,
//
// taking each absolute value at its least or greatest over the intervals, and the 1-norm above the Euclidean one. The
// sine of the ruling's angle with the plane is |d·N| / (|d|·|N|), so the lower bound over the upper bound above
// seatSinMin proves every ruling transversal. A circular meridian (a sphere or torus) has no rulings and refuses.
func denotedTransversal(r *surfacenormal.Revolved, a, b, c r3.Vec) bool {
	if !r.Valid || r.Circular || len(r.Ends) == 0 {
		return false
	}
	// The facet's exact plane: its corners are finite floats, so their differences are exact rationals.
	n := ratCross(ratSub(ratVec(b), ratVec(a)), ratSub(ratVec(c), ratVec(a)))
	nNorm := new(big.Rat)
	for _, x := range n {
		nNorm.Add(nNorm, new(big.Rat).Abs(x))
	}
	if nNorm.Sign() == 0 {
		return false
	}
	w := ivDot(r.Basis[2], n)
	e0, e1 := ivDot(r.Basis[0], n), ivDot(r.Basis[1], n)
	if w == nil || e0 == nil || e1 == nil {
		return false
	}
	wLen, e0Len, e1Len := ivNorm1(r.Basis[2]), ivNorm1(r.Basis[0]), ivNorm1(r.Basis[1])
	if wLen == nil || e0Len == nil || e1Len == nil {
		return false
	}
	sinMin := new(big.Rat).SetFloat64(seatSinMin)
	for _, seg := range r.Ends {
		dz, okZ := ivSub(seg[1][0], seg[0][0])
		drho, okR := ivSub(seg[1][1], seg[0][1])
		if !okZ || !okR {
			return false
		}
		lower := new(big.Rat).Mul(ivAbsMin(dz), ivAbsMin(w))
		lower.Sub(lower, new(big.Rat).Mul(ivAbsMax(drho), new(big.Rat).Add(ivAbsMax(e0), ivAbsMax(e1))))
		upper := new(big.Rat).Mul(ivAbsMax(dz), wLen)
		upper.Add(upper, new(big.Rat).Mul(ivAbsMax(drho), new(big.Rat).Add(e0Len, e1Len)))
		upper.Mul(upper, nNorm)
		if lower.Cmp(new(big.Rat).Mul(sinMin, upper)) <= 0 {
			return false
		}
	}
	return true
}

// ratVec reads a finite held vertex as exact rationals.
func ratVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func ratSub(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for k := range out {
		out[k] = new(big.Rat).Sub(a[k], b[k])
	}
	return out
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	m := func(x, y, z, w *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(x, y), new(big.Rat).Mul(z, w))
	}
	return [3]*big.Rat{m(a[1], b[2], a[2], b[1]), m(a[2], b[0], a[0], b[2]), m(a[0], b[1], a[1], b[0])}
}

// ivDot encloses v·n for v in the box iv.
func ivDot(iv proofbound.IvVec3, n [3]*big.Rat) *proofbound.RatInterval {
	lo, hi := new(big.Rat), new(big.Rat)
	for k := range 3 {
		if iv[k].Lo == nil || iv[k].Hi == nil {
			return nil
		}
		x, y := new(big.Rat).Mul(iv[k].Lo, n[k]), new(big.Rat).Mul(iv[k].Hi, n[k])
		if x.Cmp(y) > 0 {
			x, y = y, x
		}
		lo.Add(lo, x)
		hi.Add(hi, y)
	}
	return &proofbound.RatInterval{Lo: lo, Hi: hi}
}

// ivNorm1 bounds the 1-norm of every vector in the box iv from above.
func ivNorm1(iv proofbound.IvVec3) *big.Rat {
	sum := new(big.Rat)
	for k := range 3 {
		if iv[k].Lo == nil || iv[k].Hi == nil {
			return nil
		}
		sum.Add(sum, ivAbsMax(&iv[k]))
	}
	return sum
}

func ivSub(x, y proofbound.RatInterval) (*proofbound.RatInterval, bool) {
	if x.Lo == nil || x.Hi == nil || y.Lo == nil || y.Hi == nil {
		return nil, false
	}
	return &proofbound.RatInterval{Lo: new(big.Rat).Sub(x.Lo, y.Hi), Hi: new(big.Rat).Sub(x.Hi, y.Lo)}, true
}

// ivAbsMax is the greatest |x| over the interval.
func ivAbsMax(iv *proofbound.RatInterval) *big.Rat {
	lo, hi := new(big.Rat).Abs(iv.Lo), new(big.Rat).Abs(iv.Hi)
	if lo.Cmp(hi) > 0 {
		return lo
	}
	return hi
}

// ivAbsMin is the least |x| over the interval: zero when it straddles zero.
func ivAbsMin(iv *proofbound.RatInterval) *big.Rat {
	if iv.Lo.Sign() <= 0 && iv.Hi.Sign() >= 0 {
		return new(big.Rat)
	}
	lo, hi := new(big.Rat).Abs(iv.Lo), new(big.Rat).Abs(iv.Hi)
	if lo.Cmp(hi) < 0 {
		return lo
	}
	return hi
}

func transversal(s surfacegeom.Surface, n r3.Vec, departure float64) bool {
	limit := math.Asin(seatSinMin) + departure
	if !(limit < math.Pi/2) {
		return false
	}
	switch t := s.(type) {
	case surfacegeom.Cylinder:
		axis, ok := t.Axis.Normalize()
		if !ok {
			return false
		}
		return math.Asin(math.Min(1, math.Abs(axis.Dot(n)))) > limit
	case surfacegeom.Cone:
		axis, ok := t.Axis.Normalize()
		if !ok {
			return false
		}
		half, err := t.HalfAngle.In(units.Radian)
		if err != nil || !(half > 0) || !(half < math.Pi/2) {
			return false
		}
		tilt := math.Acos(math.Min(1, math.Abs(axis.Dot(n))))
		// The steepest-leaning ruling makes an angle half + tilt with the plane's normal.
		return math.Pi/2-(half+tilt) > limit
	}
	return false
}

// seatPair reports whether coplanar facets i of ma and j of mb form a seat the pair pass admits (§16): an Intersect
// that read seats on both operands, two carrier-exact facets, opposite facing.
func seatPair(ma, mb *BoolMesh, i, j int) bool {
	return ma.Seats != nil && mb.Seats != nil && ma.Carrier[i] && mb.Carrier[j] &&
		proof.XdotSign(ma.Norms[i], mb.Norms[j]) < 0
}

// rimFacet reports whether facet t of m has an edge SetSeats marked as a wall's rim.
func rimFacet(m *BoolMesh, t int) bool {
	return m.RimEdge != nil && (m.RimEdge[t][0] || m.RimEdge[t][1] || m.RimEdge[t][2])
}

// rimEdge reports whether edge k of facet t of m is a wall's rim (SetSeats).
func rimEdge(m *BoolMesh, t, k int) bool { return m.RimEdge != nil && k >= 0 && m.RimEdge[t][k] }

// rimOnCarrier reports whether edge k of facet t of m is a wall's rim and facet x of other is carrier-exact (§16.3).
func rimOnCarrier(m *BoolMesh, t, k int, other *BoolMesh, x int) bool {
	return rimEdge(m, t, k) && other.Carrier != nil && other.Carrier[x]
}

// rimPointOnCarrier reports whether contact c is a single point at a vertex of facet t of m that lies on a wall's rim,
// and facet x of other is carrier-exact (§16.3).
func rimPointOnCarrier(m *BoolMesh, t int, c TriContact, other *BoolMesh, x int, keyOf func(proof.Xpt) string) bool {
	if c.Kind != ContactPoint || m.RimVert == nil || other.Carrier == nil || !other.Carrier[x] {
		return false
	}
	key := ""
	for _, v := range m.Tris[t] {
		if !m.RimVert[v] {
			continue
		}
		if key == "" {
			key = keyOf(c.P0)
		}
		// The vertex's prepared lift (BoolMesh.Xverts) keeps one set of pointers, so the memo reads its key once.
		if keyOf(m.Xverts[v]) == key {
			return true
		}
	}
	return false
}

// seatPoint reports whether a point contact of facets i of ma and j of mb may be owned by the coplanar pass: two
// exact facets, as §3.3 states, or two facets of a flush pair (agent-docs/flush-contact-design.md §4.6), or a
// carrier-exact facet against a wall's rim facet, or two carrier-exact facets, of an Intersect that read seats (§16).
func seatPoint(ma, mb *BoolMesh, i, j int) (bool, error) {
	exact, err := pairExact(ma, mb, i, j)
	if err != nil || exact {
		return exact, err
	}
	if ma.Seats == nil || mb.Seats == nil {
		return false, nil
	}
	okA := ma.Carrier[i] || rimFacet(ma, i)
	okB := mb.Carrier[j] || rimFacet(mb, j)
	return okA && okB && (ma.Carrier[i] || mb.Carrier[j]), nil
}

// seatSkip reports whether the face pair of facets fx of bmX and fw of bmW needs no proximity proof (§16): fx is a
// carrier-exact face lying exactly in a plane wall fw leaves along a closed rim, and it faces the side the wall
// leaves into. The true surfaces then meet only along the wall's rims in that plane, which the mesh pass classifies
// as in-plane edges of an admitted pair or refuses.
func seatSkip(bmX *BoolMesh, fx []int, bmW *BoolMesh, fw []int) bool {
	if bmX.Seats == nil || bmW.Seats == nil || len(fx) == 0 || len(fw) == 0 {
		return false
	}
	seats := bmW.Seats[bmW.Src[fw[0]]]
	var tested map[int]struct{}
	for _, s := range seats {
		pl := newFacetPlane(bmW, s.pf)
		// A vertex of fx shared by several facets is tested once against this seat's plane.
		if tested == nil {
			tested = make(map[int]struct{}, len(fx)+2)
		} else {
			clear(tested)
		}
		ok := true
		for _, x := range fx {
			if !bmX.Carrier[x] {
				ok = false
				break
			}
			for _, v := range bmX.Tris[x] {
				if _, done := tested[v]; done {
					continue
				}
				tested[v] = struct{}{}
				if pl.side(bmX, v) != 0 {
					ok = false
					break
				}
			}
			if !ok || proof.XdotSign(bmX.Norms[x], bmW.Norms[s.pf]) >= 0 {
				ok = false
				break
			}
		}
		if ok {
			return true
		}
	}
	return false
}
