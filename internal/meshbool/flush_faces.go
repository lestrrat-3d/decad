package meshbool

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/flushface"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file decides flush pairs (agent-docs/flush-contact-design.md §4.1–§4.5): two planar carrier faces of the two
// operands in one exact plane whose held outlines keep their arrangement under every move their vertex bounds allow.

// sqrt3Up is the float just above √3: the largest factor by which dropping the normal's largest axis shrinks a
// distance in the plane.
const sqrt3Up = 1.7320508075688776

// FlushFaces decides, once per face pair, whether two faces of the two operands of one attempt are a flush pair. Both
// prepared operands of an unshifted, unfixed live attempt share one value (AttachFlushFaces); the mesh pass reads it
// through flushPair and pairExact. It is read from the pair pass's serial consumer only.
type FlushFaces struct {
	ops    [2]*flushOperand
	budget *proofbound.WorkBudget
	// memo holds each decided pair's verdict, keyed by the global source ids of the first and the second operand's face.
	memo map[[2]int]flushface.Verdict
}

// flushOperand is one operand's lazily read faces.
type flushOperand struct {
	bm *BoolMesh
	m  *topo.Mesh
	// planar reports whether a source face's true face is provably a plane (design Assumptions).
	planar func(*topo.Face) bool
	// analytic is set when the mesh publishes no per-vertex record, so its vertex bounds are faceted-vertex-bounds
	// §2.1's reading (design §4.3).
	analytic bool
	facetsOf map[int][]int
	faces    map[int]*flushFace
	// planes caches each face's record plane for carrierFacet.
	planes map[*topo.Face]*exactPlane
}

// flushFace is one source face's reading. head fills src, facets, first and planar; the carrier and outline reads
// follow on demand.
type flushFace struct {
	src    int
	facets []int
	first  int
	planar bool
	// source is the face's record, nil when the mesh names none.
	source      *topo.Face
	carrierRead bool
	carrier     bool
	outlineRead bool
	outline     flushface.Outline
}

// AttachFlushFaces attaches one flush-pair memo to both prepared operands of an unshifted, unfixed live attempt. ma and
// mb are the tessellations bmA and bmB were prepared from; planarA and planarB report whether a source face of each is
// planar (agent-docs/flush-contact-design.md Assumptions). budget is charged once per facet read, exact sign, and box
// or crossing test.
func AttachFlushFaces(ma, mb *topo.Mesh, bmA, bmB *BoolMesh, planarA, planarB func(*topo.Face) bool,
	budget *proofbound.WorkBudget) {
	ff := &FlushFaces{budget: budget, memo: map[[2]int]flushface.Verdict{}}
	ff.ops[0] = newFlushOperand(ma, bmA, planarA)
	ff.ops[1] = newFlushOperand(mb, bmB, planarB)
	bmA.Flush, bmB.Flush = ff, ff
}

func newFlushOperand(m *topo.Mesh, bm *BoolMesh, planar func(*topo.Face) bool) *flushOperand {
	op := &flushOperand{bm: bm, m: m, planar: planar, analytic: m.VertexBound == nil,
		facetsOf: map[int][]int{}, faces: map[int]*flushFace{}, planes: map[*topo.Face]*exactPlane{}}
	for t, s := range bm.Src {
		op.facetsOf[s] = append(op.facetsOf[s], t)
	}
	return op
}

// flushVerdict is the flush verdict of the faces of facet i of mx and facet j of my, which belong to different
// operands. A pair the operands share no memo for is not flush.
func flushVerdict(mx, my *BoolMesh, i, j int) (flushface.Verdict, error) {
	ff := mx.Flush
	if ff == nil || my.Flush != ff || mx == my {
		return flushface.Verdict{}, nil
	}
	return ff.pair(mx, i, my, j)
}

// flushPair reports whether facet i of mx and facet j of my lie on a flush pair of faces.
func flushPair(mx, my *BoolMesh, i, j int) (bool, error) {
	v, err := flushVerdict(mx, my, i, j)
	return v.Clear, err
}

// pairExact is the coplanar admission of facet i of ma and facet j of mb: both facets exact, or a flush pair
// (agent-docs/flush-contact-design.md §4.5).
func pairExact(ma, mb *BoolMesh, i, j int) (bool, error) {
	if ma.isExact(i) && mb.isExact(j) {
		return true, nil
	}
	return flushPair(ma, mb, i, j)
}

func (ff *FlushFaces) pair(mx *BoolMesh, i int, my *BoolMesh, j int) (flushface.Verdict, error) {
	var x, y int
	switch {
	case ff.ops[0].bm == mx && ff.ops[1].bm == my:
		x, y = i, j
	case ff.ops[0].bm == my && ff.ops[1].bm == mx:
		x, y = j, i
	default:
		return flushface.Verdict{}, nil
	}
	o0, o1 := ff.ops[0], ff.ops[1]
	key := [2]int{o0.bm.Src[x], o1.bm.Src[y]}
	if v, ok := ff.memo[key]; ok {
		return v, nil
	}
	v, err := ff.decide(o0, key[0], o1, key[1])
	if err != nil {
		return flushface.Verdict{}, err
	}
	ff.memo[key] = v
	return v, nil
}

// decide reads faces sx of ox and sy of oy and decides whether they are a flush pair: both planar carrier faces, in
// one exact plane, with outlines that keep their arrangement (flushface.Clear).
func (ff *FlushFaces) decide(ox *flushOperand, sx int, oy *flushOperand, sy int) (flushface.Verdict, error) {
	fx, fy := ox.head(sx), oy.head(sy)
	if fx == nil || fy == nil || !fx.planar || !fy.planar {
		return flushface.Verdict{}, nil
	}
	// The cheap check first: the first facets share a plane (§4.5).
	pl := newFacetPlane(ox.bm, fx.first)
	for _, v := range oy.bm.Tris[fy.first] {
		if err := ff.budget.Step(); err != nil {
			return flushface.Verdict{}, err
		}
		if pl.side(oy.bm, v) != 0 {
			return flushface.Verdict{}, nil
		}
	}
	for _, f := range [2]struct {
		op   *flushOperand
		face *flushFace
	}{{ox, fx}, {oy, fy}} {
		carrier, err := f.op.carrierOf(ff.budget, f.face)
		if err != nil || !carrier {
			return flushface.Verdict{}, err
		}
	}
	for _, f := range [2]struct {
		op   *flushOperand
		face *flushFace
	}{{ox, fx}, {oy, fy}} {
		if err := f.op.readOutline(ff.budget, f.face); err != nil {
			return flushface.Verdict{}, err
		}
	}
	return flushface.Clear(ff.budget, fx.outline, fy.outline)
}

// head returns face s's cheap reading, or nil when the operand holds no facet of it.
func (op *flushOperand) head(s int) *flushFace {
	if f, ok := op.faces[s]; ok {
		return f
	}
	facets := op.facetsOf[s]
	if len(facets) == 0 {
		return nil
	}
	f := &flushFace{src: s, facets: facets, first: facets[0]}
	if t := f.first; t < len(op.m.Source) && op.m.Source[t] != nil {
		f.source = op.m.Source[t]
		f.planar = op.planar(f.source)
	}
	op.faces[s] = f
	return f
}

// carrierOf reports whether f is a carrier face: planar, every facet carrier-exact, every corner in the first facet's
// plane (§4.1). It reads the face once.
func (op *flushOperand) carrierOf(budget *proofbound.WorkBudget, f *flushFace) (bool, error) {
	if f.carrierRead {
		return f.carrier, nil
	}
	carrier := false
	if f.planar {
		var err error
		carrier, err = op.bm.carrierFaceBy(f.facets, func(t int) bool {
			return op.bm.carrierFacet(op.m, t, nil, op.planes)
		}, budget.Step)
		if err != nil {
			return false, err
		}
	}
	// A face that carries a CapRecord lies in the plane the record denotes, which its float tag only approximates
	// where the level rounds: every corner must lie in that exact plane (agent-docs/flush-contact-design.md §4.1).
	if carrier && f.source.CapRecord != nil {
		var err error
		carrier, err = op.onCapRecord(budget, f)
		if err != nil {
			return false, err
		}
	}
	f.carrierRead, f.carrier = true, carrier
	return carrier, nil
}

// onCapRecord reports whether every corner of face f lies exactly in the plane its CapRecord denotes (capRecordPlane).
// It charges one step per distinct corner.
func (op *flushOperand) onCapRecord(budget *proofbound.WorkBudget, f *flushFace) (bool, error) {
	pl := capRecordPlane(f.source)
	if pl == nil {
		return false, nil
	}
	tested := make(map[int]struct{}, len(f.facets)+2)
	for _, t := range f.facets {
		for _, v := range op.bm.Tris[t] {
			if _, done := tested[v]; done {
				continue
			}
			tested[v] = struct{}{}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !pl.holds(op.bm.Verts[v]) {
				return false, nil
			}
		}
	}
	return true, nil
}

// capRecordPlane is the exact plane face's CapRecord denotes, {B(O + u·U + v·V + z·N) + t} with B and t the linear
// part and translation of its Xform, every held value read as an exact rational: z is the denoted level ZDenoted where
// the producer records one, and otherwise the held level Z when the face states a zero axial displacement. It is nil
// for a face with no record, a displaced face with no denoted level, a non-finite value, or a zero normal (B·U)×(B·V).
func capRecordPlane(face *topo.Face) *exactPlane {
	rec := face.CapRecord
	if rec == nil {
		return nil
	}
	z := rec.ZDenoted
	if z == nil {
		if !face.HasAxialDelta || face.AxialDelta != 0 {
			return nil
		}
		if z = proof.FloatRat(rec.Z); z == nil {
			return nil
		}
	}
	b := rec.Xform.Basis()
	var vecs [8][3]*big.Rat
	for i, v := range []r3.Vec{rec.Frame.U(), rec.Frame.V(), rec.Frame.N(), rec.Frame.Origin(), b.EX, b.EY, b.EZ,
		rec.Xform.Translation()} {
		for k, c := range [3]float64{v.X, v.Y, v.Z} {
			if vecs[i][k] = proof.FloatRat(c); vecs[i][k] == nil {
				return nil
			}
		}
	}
	apply := func(a [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for k := range out {
			out[k] = new(big.Rat)
			for j := range 3 {
				out[k].Add(out[k], new(big.Rat).Mul(vecs[4+j][k], a[j]))
			}
		}
		return out
	}
	n := ratCross(apply(vecs[0]), apply(vecs[1]))
	if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
		return nil
	}
	var local [3]*big.Rat
	for k := range local {
		local[k] = new(big.Rat).Add(vecs[3][k], new(big.Rat).Mul(z, vecs[2][k]))
	}
	o := apply(local)
	for k := range o {
		o[k].Add(o[k], vecs[7][k])
	}
	return &exactPlane{o: o, n: n}
}

// outlineEdge is one held outline edge (a, b) of a face, walked in its facet's order, and the neighbour face g its
// twin facet belongs to.
type outlineEdge struct {
	a, b     int
	g        int
	line     bool
	straight bool
}

// readOutline reads carrier face f's outline (§4.3): its edges, which are straight and which line edges, and each
// outline vertex's radius ρ.
func (op *flushOperand) readOutline(budget *proofbound.WorkBudget, f *flushFace) error {
	if f.outlineRead {
		return nil
	}
	bm := op.bm
	var edges []outlineEdge
	for _, t := range f.facets {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			twin, ok := bm.TwinFacet(t, k)
			if !ok {
				return fmt.Errorf(`%w: a flush face's held edge has no twin facet`, decaderr.ErrBooleanFailed)
			}
			if g := bm.Src[twin]; g != f.src {
				edges = append(edges, outlineEdge{a: bm.Tris[t][k], b: bm.Tris[t][(k+1)%3], g: g})
			}
		}
	}
	pl := newFacetPlane(bm, f.first)
	n, _ := bm.Norms[f.first].Vec().Normalize()
	axis := dropAxis(n)
	// Each neighbour's relation to f's plane is read once.
	type neighbour struct {
		straight, off bool
		plane         facetPlane
	}
	seen := map[int]neighbour{}
	for e := range edges {
		g := edges[e].g
		nb, ok := seen[g]
		if !ok {
			var err error
			nb.straight, nb.off, err = op.neighbourOf(budget, f, pl, n, g)
			if err != nil {
				return err
			}
			if nb.off {
				nb.plane = newFacetPlane(bm, op.faces[g].first)
			}
			seen[g] = nb
		}
		a, b := edges[e].a, edges[e].b
		edges[e].straight = nb.straight
		edges[e].line = bm.VertexBound[a] == 0 && bm.VertexBound[b] == 0
		if !edges[e].line && nb.off {
			if _, err := budget.StepN(2); err != nil {
				return err
			}
			edges[e].line = nb.plane.side(bm, a) == 0 && nb.plane.side(bm, b) == 0
		}
	}
	incident := map[int][]int{}
	for e, oe := range edges {
		incident[oe.a] = append(incident[oe.a], e)
		incident[oe.b] = append(incident[oe.b], e)
	}
	rho := make(map[int]float64, len(incident))
	for v, at := range incident {
		if err := budget.Step(); err != nil {
			return err
		}
		rho[v] = op.radius(v, at, edges, axis)
	}
	out := flushface.Outline{Axis: axis, Edges: make([]flushface.Edge, len(edges))}
	for e, oe := range edges {
		out.Edges[e] = flushface.Edge{A: bm.Verts[oe.a], B: bm.Verts[oe.b], RA: rho[oe.a], RB: rho[oe.b],
			Line: oe.line, Straight: oe.straight}
	}
	f.outline, f.outlineRead = out, true
	return nil
}

// neighbourOf reads neighbour face g of carrier face f, whose plane is pl with float unit normal n. straight reports
// that f and g are planar and g's true plane provably differs from f's: g is a carrier face with a corner of its first
// facet off pl (off), or g's Plane tag has a unit normal farther than its stated departure, padded, from both n and −n.
func (op *flushOperand) neighbourOf(budget *proofbound.WorkBudget, f *flushFace, pl facetPlane, n r3.Vec, g int) (bool, bool, error) {
	gf := op.head(g)
	if gf == nil || !f.planar || !gf.planar {
		return false, false, nil
	}
	carrier, err := op.carrierOf(budget, gf)
	if err != nil {
		return false, false, err
	}
	if carrier {
		for _, v := range op.bm.Tris[gf.first] {
			if err := budget.Step(); err != nil {
				return false, false, err
			}
			if pl.side(op.bm, v) != 0 {
				return true, true, nil
			}
		}
	}
	return tagLeavesPlane(gf.source, n), false, nil
}

// tagLeavesPlane reports whether face g's Plane tag proves g's true plane differs from a plane with float unit normal
// n: the tag's unit normal lies farther from both n and −n than NormalBound + TagDeparture, padded by a relative 1e-9
// and an absolute 1e-12 that cover the rounding of both float normals and of the distances.
func tagLeavesPlane(g *topo.Face, n r3.Vec) bool {
	if g == nil {
		return false
	}
	tag, ok := g.Surface.(surfacegeom.Plane)
	if !ok {
		return false
	}
	m, ok := tag.Frame.N().Normalize()
	if !ok {
		return false
	}
	departure := proofbound.AbsSumUpper(g.NormalBound, g.TagDeparture)
	if !(departure >= 0) || math.IsInf(departure, 0) {
		return false
	}
	pad := proofbound.AbsSumUpper(departure, proofbound.ProductUpper(departure, 1e-9), 1e-12)
	return m.Sub(n).Len() > pad && m.Add(n).Len() > pad
}

// radius is outline vertex v's ρ (§4.3). at lists the outline edges that meet v.
func (op *flushOperand) radius(v int, at []int, edges []outlineEdge, axis int) float64 {
	bm := op.bm
	beta := bm.VertexBound[v]
	if beta == 0 {
		return 0
	}
	if op.analytic {
		return beta
	}
	if len(at) != 2 {
		return math.Inf(1)
	}
	e1, e2 := edges[at[0]], edges[at[1]]
	if e1.g == e2.g {
		return beta
	}
	width := func(e outlineEdge) float64 {
		if e.line {
			return 0
		}
		return max(bm.VertexBound[e.a], bm.VertexBound[e.b])
	}
	w := proofbound.AbsSumUpper(width(e1), width(e2))
	sinLo := projectedSinLower(bm, e1, e2, axis)
	if !(sinLo > 0) || math.IsInf(sinLo, 0) {
		return math.Inf(1)
	}
	if w == 0 {
		return 0
	}
	return proofbound.ProductUpper(proofbound.DivUpper(w, sinLo), sqrt3Up)
}

// projectedSinLower is a lower bound of the sine of the angle between the two edges projected to the plane that drops
// axis: their exact projected cross product, rounded down, over the product of their rounded-up projected lengths,
// rounded down.
func projectedSinLower(bm *BoolMesh, e1, e2 outlineEdge, axis int) float64 {
	d1 := ratSub2(bm.Verts[e1.b], bm.Verts[e1.a], axis)
	d2 := ratSub2(bm.Verts[e2.b], bm.Verts[e2.a], axis)
	cr := new(big.Rat).Sub(new(big.Rat).Mul(d1[0], d2[1]), new(big.Rat).Mul(d1[1], d2[0]))
	crLo, _ := new(big.Rat).Abs(cr).Float64()
	crLo = math.Nextafter(crLo, 0)
	den := proofbound.ProductUpper(ratLenUpper(d1), ratLenUpper(d2))
	if !(crLo > 0) || !(den > 0) || math.IsInf(den, 0) {
		return 0
	}
	return math.Nextafter(crLo/den, 0)
}

// ratSub2 is q − p projected to the plane that drops axis, exactly.
func ratSub2(q, p r3.Vec, axis int) [2]*big.Rat {
	qc, pc := [3]float64{q.X, q.Y, q.Z}, [3]float64{p.X, p.Y, p.Z}
	var out [2]*big.Rat
	k := 0
	for i := range 3 {
		if i == axis {
			continue
		}
		out[k] = new(big.Rat).Sub(new(big.Rat).SetFloat64(qc[i]), new(big.Rat).SetFloat64(pc[i]))
		k++
	}
	return out
}

// ratLenUpper is an upper bound of the length of an exact 2D vector.
func ratLenUpper(d [2]*big.Rat) float64 {
	s := new(big.Rat).Add(new(big.Rat).Mul(d[0], d[0]), new(big.Rat).Mul(d[1], d[1]))
	f, _ := s.Float64()
	return proofbound.UpRound(math.Sqrt(math.Nextafter(f, math.Inf(1))))
}

// dropAxis is the index of n's largest component.
func dropAxis(n r3.Vec) int {
	ax, ay, az := math.Abs(n.X), math.Abs(n.Y), math.Abs(n.Z)
	switch {
	case ax >= ay && ax >= az:
		return 0
	case ay >= az:
		return 1
	}
	return 2
}
