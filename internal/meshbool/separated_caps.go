package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/flushface"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file decides separated cap pairs (agent-docs/coplanar-contact-design.md §19): two operands whose true solids
// lie in opposite closed half-spaces of one exact plane, each meeting that plane only in one planar face, with the two
// faces' true regions sharing a disc of positive radius.

// CapSide is a producer's claim about one planar face of an operand (agent-docs/coplanar-contact-design.md §19.2):
// the operand's true solid lies in the closed half-space {x : N·(x − O) ≤ 0}, the true face Face lies in its boundary
// plane N·(x − O) = 0, and the true solid meets that plane only in the closure of Face. O and N are exact.
type CapSide struct {
	Face *topo.Face
	O, N [3]*big.Rat
}

// SlabCapSides reads caps, the faces carrying a CapRecord of an operand whose true solid lies between the planes of
// its two caps and meets each plane only in its own cap: a straight prism or a plain two-section loft, which the
// caller decides from the payload. It returns one CapSide per cap, its normal pointing away from the other cap, or nil
// unless there are exactly two caps whose record planes (capRecordPlane) are exact, parallel and distinct.
func SlabCapSides(caps []*topo.Face) []CapSide {
	if len(caps) != 2 {
		return nil
	}
	p0, p1 := capRecordPlane(caps[0]), capRecordPlane(caps[1])
	if p0 == nil || p1 == nil || !ratIsZero(ratCross(p0.n, p1.n)) {
		return nil
	}
	away := func(p, other *exactPlane) ([3]*big.Rat, bool) {
		s := ratDot(p.n, ratSub(other.o, p.o)).Sign()
		if s == 0 {
			return p.n, false
		}
		if s > 0 {
			return ratNeg(p.n), true
		}
		return p.n, true
	}
	n0, ok0 := away(p0, p1)
	n1, ok1 := away(p1, p0)
	if !ok0 || !ok1 {
		return nil
	}
	return []CapSide{{Face: caps[0], O: p0.o, N: n0}, {Face: caps[1], O: p1.o, N: n1}}
}

// RevolveCapSides reads faces, every face of a revolve whose recorded meridian is the one it denotes, from their
// revolve records (topo.Face.Denoted). Every face but a partial sweep's end cap must record straight meridian
// segments with exact ends and one exact axis frame. A point at meridian coordinates (z, ρ) sits at
// O + B₀·ρ·cos φ + B₁·ρ·sin φ + B₂·z, so n = B₀ × B₁ gives n·(x − O − z₀·B₂) = (n·B₂)(z − z₀): the solid lies
// between the levels of the lowest and highest meridian ends, since straight segments reach their extremes at their
// ends. At each of those two levels it returns a CapSide when exactly one face lies wholly at that level, and every
// other meridian end at that level has a ρ that face's ends also have, so the solid meets the plane only in that face.
// It returns nil when a face fails the reading.
func RevolveCapSides(faces []*topo.Face) []CapSide {
	type end struct{ z, rho *big.Rat }
	type wall struct {
		face *topo.Face
		ends []end
	}
	var walls []wall
	var frame [4][3]*big.Rat
	framed := false
	for _, f := range faces {
		r := f.Denoted
		if r == nil || !r.Valid {
			return nil
		}
		if r.Cap {
			// A partial sweep's end cap lies in a plane through the axis and adds no meridian point.
			continue
		}
		if r.Circular {
			return nil
		}
		var fr [4][3]*big.Rat
		for k, iv := range [4]proofbound.IvVec3{r.Origin, r.Basis[0], r.Basis[1], r.Basis[2]} {
			v, ok := ivPoint3(iv)
			if !ok {
				return nil
			}
			fr[k] = v
		}
		if !framed {
			frame, framed = fr, true
		} else if !ratFrameEqual(frame, fr) {
			return nil
		}
		w := wall{face: f}
		for _, seg := range r.Ends {
			for _, e := range seg {
				z, rho := ivPoint(e[0]), ivPoint(e[1])
				if z == nil || rho == nil {
					return nil
				}
				w.ends = append(w.ends, end{z: z, rho: rho})
			}
		}
		if len(w.ends) == 0 {
			return nil
		}
		walls = append(walls, w)
	}
	if !framed {
		return nil
	}
	n := ratCross(frame[1], frame[2])
	sense := ratDot(n, frame[3]).Sign()
	if sense == 0 {
		return nil
	}
	lo, hi := walls[0].ends[0].z, walls[0].ends[0].z
	for _, w := range walls {
		for _, e := range w.ends {
			if e.z.Cmp(lo) < 0 {
				lo = e.z
			}
			if e.z.Cmp(hi) > 0 {
				hi = e.z
			}
		}
	}
	var out []CapSide
	for _, level := range []struct {
		z       *big.Rat
		outward int
	}{{lo, -sense}, {hi, sense}} {
		capAt := -1
		for k, w := range walls {
			whole := true
			for _, e := range w.ends {
				whole = whole && e.z.Cmp(level.z) == 0
			}
			if !whole {
				continue
			}
			if capAt >= 0 {
				capAt = -2
				break
			}
			capAt = k
		}
		if capAt < 0 {
			continue
		}
		onCap := func(rho *big.Rat) bool {
			for _, e := range walls[capAt].ends {
				if e.rho.Cmp(rho) == 0 {
					return true
				}
			}
			return false
		}
		alone := true
		for k, w := range walls {
			if k == capAt {
				continue
			}
			for _, e := range w.ends {
				alone = alone && (e.z.Cmp(level.z) != 0 || onCap(e.rho))
			}
		}
		if !alone {
			continue
		}
		var o [3]*big.Rat
		for k := range o {
			o[k] = new(big.Rat).Add(frame[0][k], new(big.Rat).Mul(level.z, frame[3][k]))
		}
		dir := n
		if level.outward < 0 {
			dir = ratNeg(n)
		}
		out = append(out, CapSide{Face: walls[capAt].face, O: o, N: dir})
	}
	return out
}

// AttachCapSides gives the flush memo both operands bmA and bmB share (AttachFlushFaces) each operand's cap sides, in
// operand order. It does nothing when the operands share no memo.
func AttachCapSides(bmA, bmB *BoolMesh, a, b []CapSide) {
	ff := bmA.Flush
	if ff == nil || bmB.Flush != ff || ff.ops[0].bm != bmA {
		return
	}
	ff.sides = [2][]CapSide{a, b}
}

// SeparatedOperands reports whether the two operands bmA and bmB carry cap sides that form a separated cap pair
// (agent-docs/coplanar-contact-design.md §19.3): their true solids then meet only where their caps' true regions
// overlap, and those overlap over a disc of positive radius, so the proximity gate has nothing a chord could hide.
func SeparatedOperands(bmA, bmB *BoolMesh) (bool, error) {
	ff := bmA.Flush
	if ff == nil || bmB.Flush != ff || len(ff.sides[0]) == 0 || len(ff.sides[1]) == 0 {
		return false, nil
	}
	for _, ca := range ff.sides[0] {
		sx, ok := ff.ops[0].srcOf(ca.Face)
		if !ok {
			continue
		}
		for _, cb := range ff.sides[1] {
			sy, ok := ff.ops[1].srcOf(cb.Face)
			if !ok {
				continue
			}
			sep, err := ff.separatedPair(sx, sy)
			if err != nil {
				return false, err
			}
			if sep.ok {
				ff.plane, ff.planeReach = &ca, sep.reach
				return true, nil
			}
		}
	}
	return false, nil
}

// separatedEdge reports whether edge k of facet f of m lies in the plane SeparatedOperands found the operands on
// opposite sides of, exactly. It is false before SeparatedOperands has found one.
func separatedEdge(m *BoolMesh, f, k int) bool {
	ff := m.Flush
	if ff == nil || ff.plane == nil {
		return false
	}
	t := m.Tris[f]
	return sideOf(*ff.plane, m.Verts[t[k]]) == 0 && sideOf(*ff.plane, m.Verts[t[(k+1)%3]]) == 0
}

// separatedPoint reports whether exact point p lies in the plane SeparatedOperands found the operands on opposite sides
// of, and returns the bound §19.5 gives a result vertex there: the found pair's reach. It is false before
// SeparatedOperands has found a plane.
func separatedPoint(m *BoolMesh, p proof.Xpt) (float64, bool) {
	ff := m.Flush
	if ff == nil || ff.plane == nil || p.W == nil || p.W.Sign() == 0 {
		return 0, false
	}
	side := ff.plane
	w := new(big.Rat).SetInt(p.W)
	s := new(big.Rat)
	for k, c := range [3]*big.Int{p.X, p.Y, p.Z} {
		x := new(big.Rat).SetInt(c)
		x.Quo(x, w)
		s.Add(s, new(big.Rat).Mul(side.N[k], x.Sub(x, side.O[k])))
	}
	return ff.planeReach, s.Sign() == 0
}

// separation is a separated cap pair verdict: ok, reach, the sum of the two faces' outline reaches, which bounds how
// far a point the coplanar pass keeps on either face lies from the true boundary of the result (§19.5), and band, the
// strip about both outlines where such a point may lie off that boundary.
type separation struct {
	ok    bool
	reach float64
	band  *separatedBand
}

// separatedBand is the strip about the held outlines of a separated cap pair (§19.5). A point a face of the pair keeps
// lies off the result's true boundary only within a face's outline reach of that face's or the other face's held
// outline, so a kept piece farther than the larger reach from every outline edge needs no bound for it. boxes holds
// every outline edge's box, half the larger reach, and reach the bound a kept piece near the strip takes at each corner.
type separatedBand struct {
	boxes [][2]r3.Vec
	half  float64
	reach float64
}

func newSeparatedBand(ox, oy flushface.Outline, rx, ry, reach float64) *separatedBand {
	b := &separatedBand{reach: reach}
	b.half = proofbound.AbsSumUpper(max(rx, ry), proofbound.ProductUpper(max(rx, ry), 1e-9))
	for _, o := range [2]flushface.Outline{ox, oy} {
		for _, e := range o.Edges {
			b.boxes = append(b.boxes, [2]r3.Vec{
				{X: min(e.A.X, e.B.X), Y: min(e.A.Y, e.B.Y), Z: min(e.A.Z, e.B.Z)},
				{X: max(e.A.X, e.B.X), Y: max(e.A.Y, e.B.Y), Z: max(e.A.Z, e.B.Z)},
			})
		}
	}
	return b
}

// raise lifts beta, the bounds of the corners of a piece tri the side pass keeps, to the band's reach where the piece
// comes near the band: at each corner near it, or at every corner when the piece is near and no corner is. The piece's
// facet bound is its largest corner bound, so either way every point of the piece is charged the reach, while a corner
// far from both outlines keeps its own bound for the other facets that share it.
func (b *separatedBand) raise(tri [3]proof.Xpt, beta *[3]float64) {
	if !b.near(tri) {
		return
	}
	lifted := false
	for k, p := range tri {
		if b.near([3]proof.Xpt{p, p, p}) {
			beta[k] = max(beta[k], b.reach)
			lifted = true
		}
	}
	if lifted {
		return
	}
	for k := range beta {
		beta[k] = max(beta[k], b.reach)
	}
}

// near reports whether the triangle tri may come within the band's half-width of an outline edge. Its box spans the
// rounded corners widened by two floats each way, which holds the exact corners, and a box pair counts as apart only
// when the float gap stepped toward minus infinity exceeds the half-width, so rounding can only report more pieces
// near: the safe direction, since a near piece only takes a larger bound.
func (b *separatedBand) near(tri [3]proof.Xpt) bool {
	lo, hi := tri[0].Vec(), tri[0].Vec()
	for _, p := range tri[1:] {
		v := p.Vec()
		lo = r3.Vec{X: min(lo.X, v.X), Y: min(lo.Y, v.Y), Z: min(lo.Z, v.Z)}
		hi = r3.Vec{X: max(hi.X, v.X), Y: max(hi.Y, v.Y), Z: max(hi.Z, v.Z)}
	}
	if !proofbound.FiniteVec(lo) || !proofbound.FiniteVec(hi) {
		return true
	}
	down := func(x float64) float64 { return math.Nextafter(math.Nextafter(x, math.Inf(-1)), math.Inf(-1)) }
	up := func(x float64) float64 { return math.Nextafter(math.Nextafter(x, math.Inf(1)), math.Inf(1)) }
	lo = r3.Vec{X: down(lo.X), Y: down(lo.Y), Z: down(lo.Z)}
	hi = r3.Vec{X: up(hi.X), Y: up(hi.Y), Z: up(hi.Z)}
	apart := func(l, h float64) bool { return math.Nextafter(l-h, math.Inf(-1)) > b.half }
	for _, e := range b.boxes {
		if !(apart(e[0].X, hi.X) || apart(lo.X, e[1].X) || apart(e[0].Y, hi.Y) || apart(lo.Y, e[1].Y) ||
			apart(e[0].Z, hi.Z) || apart(lo.Z, e[1].Z)) {
			return true
		}
	}
	return false
}

// separatedPair decides whether face sx of the first operand and face sy of the second are a separated cap pair,
// memoised per face pair.
func (ff *FlushFaces) separatedPair(sx, sy int) (separation, error) {
	key := [2]int{sx, sy}
	if v, ok := ff.sepMemo[key]; ok {
		return v, nil
	}
	v, err := ff.decideSeparated(sx, sy)
	if err != nil {
		return separation{}, err
	}
	if ff.sepMemo == nil {
		ff.sepMemo = map[[2]int]separation{}
	}
	ff.sepMemo[key] = v
	return v, nil
}

func (ff *FlushFaces) decideSeparated(sx, sy int) (separation, error) {
	o0, o1 := ff.ops[0], ff.ops[1]
	fx, fy := o0.head(sx), o1.head(sy)
	if fx == nil || fy == nil || fx.source == nil || fy.source == nil {
		return separation{}, nil
	}
	for ia, ca := range ff.sides[0] {
		if ca.Face != fx.source {
			continue
		}
		for ib, cb := range ff.sides[1] {
			if cb.Face != fy.source {
				continue
			}
			v, err := ff.separatedSides(ia, ib, fx, fy)
			if err != nil || v.ok {
				return v, err
			}
		}
	}
	return separation{}, nil
}

// separatedSides checks cap side ia of the first operand, on face fx, against cap side ib of the second, on face fy
// (§19.3): one exact plane with opposite outward normals, each face's held corners in it and facing out of its own
// operand, each operand's held mesh in its own closed half-space, and a point of the plane that lies inside both held
// faces deeper than each face's outline reach.
func (ff *FlushFaces) separatedSides(ia, ib int, fx, fy *flushFace) (separation, error) {
	ca, cb := ff.sides[0][ia], ff.sides[1][ib]
	if err := ff.budget.Step(); err != nil {
		return separation{}, err
	}
	if !ratIsZero(ratCross(ca.N, cb.N)) || ratDot(ca.N, cb.N).Sign() >= 0 || ratDot(ca.N, ratSub(cb.O, ca.O)).Sign() != 0 {
		return separation{}, nil
	}
	for _, f := range [2]struct {
		op   *flushOperand
		face *flushFace
		side CapSide
	}{{ff.ops[0], fx, ca}, {ff.ops[1], fy, cb}} {
		ok, err := f.op.faceOnSide(ff.budget, f.face, f.side)
		if err != nil || !ok {
			return separation{}, err
		}
	}
	for k, idx := range [2]int{ia, ib} {
		ok, err := ff.heldInside(k, idx)
		if err != nil || !ok {
			return separation{}, err
		}
	}
	for _, f := range [2]struct {
		op   *flushOperand
		face *flushFace
	}{{ff.ops[0], fx}, {ff.ops[1], fy}} {
		if err := f.op.readOutline(ff.budget, f.face); err != nil {
			return separation{}, err
		}
	}
	ox, oy := fx.outline, fy.outline
	if ox.Axis != oy.Axis {
		return separation{}, nil
	}
	rx, ry := ox.Reach(), oy.Reach()
	reach := proofbound.AbsSumUpper(rx, ry)
	if proofbound.IsNonFinite(reach) {
		return separation{}, nil
	}
	for _, p := range ff.overlapCandidates(fx, fy) {
		inX, err := flushface.Deep(ff.budget, ox, p, rx)
		if err != nil {
			return separation{}, err
		}
		if !inX {
			continue
		}
		inY, err := flushface.Deep(ff.budget, oy, p, ry)
		if err != nil {
			return separation{}, err
		}
		if inY {
			return separation{ok: true, reach: reach, band: newSeparatedBand(ox, oy, rx, ry, reach)}, nil
		}
	}
	return separation{}, nil
}

// maxCentroidCandidates caps how many facet centroids of each face overlapCandidates tries.
const maxCentroidCandidates = 64

// overlapCandidates lists the points separatedSides tries as a common deep point of faces fx and fy: the mean of each
// face's outline corners, their midpoint, and the centroids of up to maxCentroidCandidates facets of each face. Any
// float point serves, since Deep certifies the point of the plane that projects onto it.
func (ff *FlushFaces) overlapCandidates(fx, fy *flushFace) []r3.Vec {
	mean := func(o flushface.Outline) (r3.Vec, bool) {
		var s r3.Vec
		for _, e := range o.Edges {
			s = s.Add(e.A)
		}
		if len(o.Edges) == 0 {
			return r3.Vec{}, false
		}
		return s.Scale(1 / float64(len(o.Edges))), true
	}
	var out []r3.Vec
	mx, okx := mean(fx.outline)
	my, oky := mean(fy.outline)
	if okx {
		out = append(out, mx)
	}
	if oky {
		out = append(out, my)
	}
	if okx && oky {
		out = append(out, mx.Add(my).Scale(0.5))
	}
	for _, f := range [2]struct {
		op   *flushOperand
		face *flushFace
	}{{ff.ops[0], fx}, {ff.ops[1], fy}} {
		for k, t := range f.face.facets {
			if k == maxCentroidCandidates {
				break
			}
			c := TriCorners(f.op.bm, t)
			out = append(out, c[0].Add(c[1]).Add(c[2]).Scale(1.0/3))
		}
	}
	return out
}

// faceOnSide reports whether every held corner of face f lies exactly in side's plane and f's first facet faces along
// side's outward normal, so the held face is the side's cap seen from outside its operand. It charges one step per
// distinct corner.
func (op *flushOperand) faceOnSide(budget *proofbound.WorkBudget, f *flushFace, side CapSide) (bool, error) {
	seen := make(map[int]struct{}, len(f.facets)+2)
	for _, t := range f.facets {
		for _, v := range op.bm.Tris[t] {
			if _, done := seen[v]; done {
				continue
			}
			seen[v] = struct{}{}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if sideOf(side, op.bm.Verts[v]) != 0 {
				return false, nil
			}
		}
	}
	c := TriCorners(op.bm, f.first)
	a := ratVec(c[0])
	normal := ratCross(ratSub(ratVec(c[1]), a), ratSub(ratVec(c[2]), a))
	return ratDot(normal, side.N).Sign() > 0, nil
}

// heldInside reports whether operand k's held mesh lies in the closed half-space of its cap side idx and meets the
// side's plane only in the closure of the side's face: every held vertex lies on the inner side or on the plane, no
// facet of another face lies in the plane, and a facet of another face that touches the plane does so along an edge or
// at a vertex of the face's own facets. It is memoised per side.
func (ff *FlushFaces) heldInside(k, idx int) (bool, error) {
	key := [2]int{k, idx}
	if v, ok := ff.inside[key]; ok {
		return v, nil
	}
	ok, err := ff.readHeldInside(k, idx)
	if err != nil {
		return false, err
	}
	if ff.inside == nil {
		ff.inside = map[[2]int]bool{}
	}
	ff.inside[key] = ok
	return ok, nil
}

func (ff *FlushFaces) readHeldInside(k, idx int) (bool, error) {
	side := ff.sides[k][idx]
	op := ff.ops[k]
	bm := op.bm
	src, ok := op.srcOf(side.Face)
	if !ok {
		return false, nil
	}
	on := make([]bool, len(bm.Verts))
	for v, p := range bm.Verts {
		if err := ff.budget.Step(); err != nil {
			return false, err
		}
		if !proofbound.FiniteVec(p) {
			return false, nil
		}
		switch sideOf(side, p) {
		case 1:
			return false, nil
		case 0:
			on[v] = true
		}
	}
	verts := map[int]struct{}{}
	edges := map[[2]int]struct{}{}
	for _, t := range op.facetsOf[src] {
		tri := bm.Tris[t]
		for e := range 3 {
			a, b := tri[e], tri[(e+1)%3]
			verts[a] = struct{}{}
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	for t, tri := range bm.Tris {
		if err := ff.budget.Step(); err != nil {
			return false, err
		}
		if bm.Src[t] == src {
			continue
		}
		var in []int
		for _, v := range tri {
			if on[v] {
				in = append(in, v)
			}
		}
		switch len(in) {
		case 3:
			return false, nil
		case 2:
			if _, ok := edges[[2]int{min(in[0], in[1]), max(in[0], in[1])}]; !ok {
				return false, nil
			}
		case 1:
			if _, ok := verts[in[0]]; !ok {
				return false, nil
			}
		}
	}
	return true, nil
}

// srcOf returns the source id of face f in the operand, or false when no facet comes from it.
func (op *flushOperand) srcOf(f *topo.Face) (int, bool) {
	if f == nil {
		return 0, false
	}
	if op.srcByFace == nil {
		op.srcByFace = map[*topo.Face]int{}
		for t, s := range op.bm.Src {
			if t < len(op.m.Source) && op.m.Source[t] != nil {
				op.srcByFace[op.m.Source[t]] = s
			}
		}
	}
	s, ok := op.srcByFace[f]
	return s, ok
}

// sideOf is the exact sign of N·(v − O): negative inside side's half-space, zero on its plane.
func sideOf(side CapSide, v r3.Vec) int {
	return ratDot(side.N, ratSub(ratVec(v), side.O)).Sign()
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	s := new(big.Rat)
	for k := range 3 {
		s.Add(s, new(big.Rat).Mul(a[k], b[k]))
	}
	return s
}

func ratNeg(a [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Neg(a[0]), new(big.Rat).Neg(a[1]), new(big.Rat).Neg(a[2])}
}

func ratIsZero(a [3]*big.Rat) bool { return a[0].Sign() == 0 && a[1].Sign() == 0 && a[2].Sign() == 0 }

// ivPoint is the exact value of a degenerate interval, or nil.
func ivPoint(iv proofbound.RatInterval) *big.Rat {
	if iv.Lo == nil || iv.Hi == nil || iv.Lo.Cmp(iv.Hi) != 0 {
		return nil
	}
	return iv.Lo
}

func ivPoint3(iv proofbound.IvVec3) ([3]*big.Rat, bool) {
	var out [3]*big.Rat
	for k := range 3 {
		if out[k] = ivPoint(iv[k]); out[k] == nil {
			return out, false
		}
	}
	return out, true
}

func ratFrameEqual(a, b [4][3]*big.Rat) bool {
	for i := range a {
		for k := range 3 {
			if a[i][k].Cmp(b[i][k]) != 0 {
				return false
			}
		}
	}
	return true
}
