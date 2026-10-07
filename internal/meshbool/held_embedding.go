package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file keeps a held faceted mesh embedded through its float rounding
// (docs/evaluator-design.md §9, "The final rounding to float64 keeps the mesh
// embedded"). The exact mesh a boolean stitches, or the exact affine image a
// placement moves, is embedded. Rounding each exact vertex to binary64 moves it
// by up to an ulp, and where a vertex sits closer than that to an edge or facet
// it does not touch, the rounded mesh can fold through itself. Only a facet
// with a vertex that moved can meet anything differently from the exact mesh,
// so only those facets are checked, against every held facet near them, with
// the exact predicates over the held binary64 values. A vertex on a failing
// pair may move to another float of its exact point's float box; a mesh still
// failing afterwards is refused.

// HeldRounding is one held mesh under the embedding check: the rounded float
// vertices it holds, its facets, and per held vertex the exact point it stands
// for. Moved marks a held vertex that differs from what it stands for — rounded
// off its exact point, or welded from several exact points — and Movable marks
// one the search may place at another corner of its exact point's float box.
// A Movable vertex must stand for exactly one exact point, Exact[v], and be
// Moved.
type HeldRounding struct {
	Verts   []r3.Vec
	Tris    [][3]int
	Exact   []proof.Xpt
	Moved   []bool
	Movable []bool
}

// heldEmbeddingSweeps caps the passes over the offending vertices. A pass that
// moves nothing ends the search early; the cap only bounds the work.
const heldEmbeddingSweeps = 16

// heldGridCellSpan is the most grid cells one facet registers in. A facet
// spanning more (a long trunk side, a large cap) joins the list every query
// scans, so one huge facet cannot cost a cell per unit of its length.
const heldGridCellSpan = 512

// EnforceHeldEmbedding proves the held mesh embedded wherever rounding moved
// it, moving offending vertices to other float-box corners when that is what
// it takes, and refuses (ErrUnsupported) a mesh no such choice clears. It
// updates h.Verts in place and returns how many vertices it moved off their
// first rounding.
//
// The rule is deterministic. Each pass visits the movable vertices of the
// failing pairs in ascending held index, and each moves to the corner of its
// float box that leaves the fewest failing pairs among its own facets, when
// that is fewer than where it sits (improve). Passes repeat over the facets
// that failed or that a move touched until one moves nothing; the facets
// that are still failing then are refused.
func EnforceHeldEmbedding(ctx context.Context, h HeldRounding) (int, error) {
	return enforceHeldEmbedding(ctx, h, true)
}

// enforceHeldEmbedding is EnforceHeldEmbedding with the per-facet box cache
// and the pair memo switched by memo. Both only replay answers the same
// positions already gave, so the result is the same either way; the tests
// run both paths and compare them.
func enforceHeldEmbedding(ctx context.Context, h HeldRounding, memo bool) (int, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return 0, err
	}
	e := newHeldEmbedding(h, budget, memo)
	var query []int
	for i, t := range h.Tris {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		if h.Moved[t[0]] || h.Moved[t[1]] || h.Moved[t[2]] {
			query = append(query, i)
		}
	}
	if len(query) == 0 {
		return 0, nil
	}
	if err := e.buildGrid(); err != nil {
		return 0, err
	}
	if memo {
		// A query facet meets a few dozen boxes around it; sizing the memo
		// for that up front spares it the rehashing of growing there.
		e.pairs = make(map[uint64]heldPair, 32*len(query))
	}
	// Every facet outside check is known embedded against all its
	// neighbours; a move puts the facets around the moved vertex back in.
	check := query
	moved := map[int]struct{}{}
	for range heldEmbeddingSweeps {
		offenders, failing, err := e.offenders(check)
		if err != nil {
			return 0, err
		}
		if len(failing) == 0 {
			return len(moved), nil
		}
		next := map[int]struct{}{}
		for _, f := range failing {
			next[f] = struct{}{}
		}
		changed := false
		for _, v := range offenders {
			placed, err := e.improve(v)
			if err != nil {
				return 0, err
			}
			if !placed {
				continue
			}
			moved[v] = struct{}{}
			changed = true
			for _, f := range e.inc[v] {
				next[f] = struct{}{}
			}
		}
		check = make([]int, 0, len(next))
		for f := range next {
			check = append(check, f)
		}
		slices.Sort(check)
		if !changed {
			break
		}
	}
	_, failing, err := e.offenders(check)
	if err != nil {
		return 0, err
	}
	if len(failing) > 0 {
		return 0, fmt.Errorf(`%w: the result's float rounding folds its held mesh through itself, and no float within an ulp of the exact vertices clears it — the held mesh would not be embedded, so this evaluator refuses the result rather than publish it`, decaderr.ErrUnsupported)
	}
	return len(moved), nil
}

type heldEmbedding struct {
	h      HeldRounding
	budget *proofbound.WorkBudget
	inc    [][]int
	taken  map[r3.Vec]int

	// Exact lifts of the held float vertices and facets, invalidated when a
	// vertex moves.
	xv     []proof.Xpt
	xvOK   []bool
	norm   []proof.Xpt
	normOK []bool
	// plane is each facet's exact plane, interned so two facets are coplanar
	// exactly when their ids match; -1 is not yet computed, -2 no plane.
	plane  []int
	planes map[string]int
	fnorm  []r3.Vec
	// cone caches, per facet and corner, a box around the unit directions
	// the facet leaves that corner in (coneBox).
	cone    [][3][2]r3.Vec
	coneOK  [][3]bool
	stamp   []int
	stampID int
	cands   []int

	// memoOn enables the two caches below. box is each facet's float box
	// (TriBox), invalidated when a vertex moves. pairs replays pairEmbedded
	// for an unordered pair of facets: gen stamps each facet's current
	// vertex positions, a move gives every incident facet a fresh stamp, and
	// an entry holds only while both facets carry the stamps it was stored
	// under. pairEmbedded is symmetric in its facets and depends on nothing
	// but their six vertex positions, so a replayed answer is the answer.
	memoOn  bool
	box     [][2]r3.Vec
	boxOK   []bool
	gen     []uint64
	nextGen uint64
	pairs   map[uint64]heldPair

	// The grid every query reads, built over facet boxes padded by pad so a
	// vertex's move inside its float box never leaves its facets' cells.
	pad    float64
	origin r3.Vec
	cell   float64
	dims   [3]int
	cells  [][]int32
	wide   []int32
}

// heldPair is one replayable pairEmbedded answer: the stamps of the lower and
// the higher facet when it was decided, and whether the pair was embedded.
type heldPair struct {
	genLo, genHi uint64
	ok           bool
}

func newHeldEmbedding(h HeldRounding, budget *proofbound.WorkBudget, memo bool) *heldEmbedding {
	e := &heldEmbedding{
		h:      h,
		budget: budget,
		memoOn: memo,
		inc:    make([][]int, len(h.Verts)),
		taken:  make(map[r3.Vec]int, len(h.Verts)),
		xv:     make([]proof.Xpt, len(h.Verts)),
		xvOK:   make([]bool, len(h.Verts)),
		norm:   make([]proof.Xpt, len(h.Tris)),
		normOK: make([]bool, len(h.Tris)),
		plane:  make([]int, len(h.Tris)),
		fnorm:  make([]r3.Vec, len(h.Tris)),
		cone:   make([][3][2]r3.Vec, len(h.Tris)),
		coneOK: make([][3]bool, len(h.Tris)),
		planes: map[string]int{},
		stamp:  make([]int, len(h.Tris)),
	}
	for i, t := range h.Tris {
		for _, v := range t {
			e.inc[v] = append(e.inc[v], i)
		}
	}
	for v, p := range h.Verts {
		e.taken[p] = v
	}
	for i := range e.plane {
		e.plane[i] = -1
	}
	if memo {
		e.box = make([][2]r3.Vec, len(h.Tris))
		e.boxOK = make([]bool, len(h.Tris))
		e.gen = make([]uint64, len(h.Tris))
		e.nextGen = 1
	}
	return e
}

// triBox is facet i's float box, TriBox over the held vertices.
func (e *heldEmbedding) triBox(i int) [2]r3.Vec {
	if !e.memoOn {
		return TriBox(e.h.Verts, e.h.Tris[i])
	}
	if !e.boxOK[i] {
		e.box[i] = TriBox(e.h.Verts, e.h.Tris[i])
		e.boxOK[i] = true
	}
	return e.box[i]
}

// pairOK is pairEmbedded, replayed from the pair memo when neither facet has
// changed since the pair was last decided.
func (e *heldEmbedding) pairOK(i, j int) bool {
	if !e.memoOn {
		return e.pairEmbedded(i, j)
	}
	if !BoxesOverlap(e.triBox(i), e.triBox(j)) {
		return true
	}
	lo, hi := min(i, j), max(i, j)
	key := uint64(lo)<<32 | uint64(hi)
	if p, ok := e.pairs[key]; ok && p.genLo == e.gen[lo] && p.genHi == e.gen[hi] {
		return p.ok
	}
	ok := e.pairEmbedded(i, j)
	e.pairs[key] = heldPair{genLo: e.gen[lo], genHi: e.gen[hi], ok: ok}
	return ok
}

// stamps copies the current stamps of v's incident facets.
func (e *heldEmbedding) stamps(v int) []uint64 {
	out := make([]uint64, len(e.inc[v]))
	for k, f := range e.inc[v] {
		out[k] = e.gen[f]
	}
	return out
}

// restamp gives v's incident facets back the stamps saved by stamps, which
// is sound only while every vertex of those facets is where it was then.
func (e *heldEmbedding) restamp(v int, saved []uint64) {
	for k, f := range e.inc[v] {
		e.gen[f] = saved[k]
	}
}

func (e *heldEmbedding) lift(v int) proof.Xpt {
	if !e.xvOK[v] {
		e.xv[v] = proof.XptOf(e.h.Verts[v])
		e.xvOK[v] = true
	}
	return e.xv[v]
}

func (e *heldEmbedding) facet(i int) ([3]r3.Vec, [3]proof.Xpt, proof.Xpt) {
	t := e.h.Tris[i]
	f := [3]r3.Vec{e.h.Verts[t[0]], e.h.Verts[t[1]], e.h.Verts[t[2]]}
	x := [3]proof.Xpt{e.lift(t[0]), e.lift(t[1]), e.lift(t[2])}
	if !e.normOK[i] {
		e.norm[i] = proof.Xcross(proof.Xsub(x[1], x[0]), proof.Xsub(x[2], x[0]))
		e.fnorm[i] = e.norm[i].Vec()
		e.normOK[i] = true
	}
	return f, x, e.norm[i]
}

func (e *heldEmbedding) setVert(v int, p r3.Vec) {
	delete(e.taken, e.h.Verts[v])
	e.h.Verts[v] = p
	e.taken[p] = v
	e.xvOK[v] = false
	for _, f := range e.inc[v] {
		e.normOK[f] = false
		e.plane[f] = -1
		e.coneOK[f] = [3]bool{}
		if e.memoOn {
			e.boxOK[f] = false
			e.gen[f] = e.nextGen
			e.nextGen++
		}
	}
}

func zeroXpt(n proof.Xpt) bool {
	return n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0
}

// pairEmbedded decides, exactly, whether facets i and j meet only along what
// their indices share: nothing, the one shared vertex, or the one shared edge
// (docs/tessellation-design.md §1, the Embedding row). An unclassifiable pair
// fails, the refusing direction.
func (e *heldEmbedding) pairEmbedded(i, j int) bool {
	ti, tj := e.h.Tris[i], e.h.Tris[j]
	if !BoxesOverlap(e.triBox(i), e.triBox(j)) {
		return true
	}
	var sharedI, sharedJ []int
	for a, va := range ti {
		for b, vb := range tj {
			if va == vb {
				sharedI = append(sharedI, a)
				sharedJ = append(sharedJ, b)
			}
		}
	}
	if len(sharedI) == 3 {
		return false
	}
	if len(sharedI) == 1 && !BoxesOverlap(e.coneBox(i, sharedI[0]), e.coneBox(j, sharedJ[0])) {
		return true
	}
	if ok, decided := e.pairSeparated(i, j, sharedI, sharedJ); decided {
		return ok
	}
	fa, xa, na := e.facet(i)
	fb, xb, nb := e.facet(j)
	if zeroXpt(na) || zeroXpt(nb) {
		return false
	}
	c, err := TriTriClassifyPrepared(fa, fb, xa, xb, na, nb, e.fnorm[i], e.fnorm[j])
	if err != nil {
		return false
	}
	switch len(sharedI) {
	case 0:
		return c.Kind == ContactNone
	case 1:
		switch c.Kind {
		case ContactPoint:
			return true
		case ContactRegion:
			// Coplanar facets sharing one vertex: embedded exactly when
			// they share no positive-length piece of boundary or area.
			return !CoplanarOverlap(xa, xb, na)
		default:
			return false
		}
	default:
		switch c.Kind {
		case ContactSegment:
			return true
		case ContactRegion:
			// Coplanar facets sharing an edge: embedded exactly when their
			// apexes lie strictly on opposite sides of it.
			a, b := xa[sharedI[0]], xa[sharedI[1]]
			apexI := xa[3-sharedI[0]-sharedI[1]]
			apexJ := xb[3-sharedJ[0]-sharedJ[1]]
			return PlaneSide(a, b, apexI, na)*PlaneSide(a, b, apexJ, na) < 0
		default:
			return false
		}
	}
}

// coneBox bounds the unit directions in which facet i leaves its corner k:
// the arc of the unit sphere between its two edge directions there. Two facets
// sharing only that corner meet beyond it exactly when they leave it in a
// common direction — their intersection is convex and holds the corner — so
// boxes that do not overlap prove the pair meets only at the corner.
//
// The box is conservative. Each edge direction is a float normalisation good
// to a few ulps, the arc bulges past the box of its endpoints by at most the
// factor 1/cos(θ/2) for the angle θ between them, and a margin of 1e-9 covers
// every float error in that with room to spare. An edge shorter than 2⁻²⁰ of
// the coordinates' own scale, or an angle past 170°, gets the whole cube: its
// direction is not worth trusting to floats.
func (e *heldEmbedding) coneBox(i, k int) [2]r3.Vec {
	if e.coneOK[i][k] {
		return e.cone[i][k]
	}
	whole := [2]r3.Vec{r3.NewVec(-2, -2, -2), r3.NewVec(2, 2, 2)}
	t := e.h.Tris[i]
	v, a, b := e.h.Verts[t[k]], e.h.Verts[t[(k+1)%3]], e.h.Verts[t[(k+2)%3]]
	scale := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	floor := math.Ldexp(math.Max(scale, math.SmallestNonzeroFloat64), -20)
	da, db := a.Sub(v), b.Sub(v)
	la, lb := da.Len(), db.Len()
	box := whole
	if la > floor && lb > floor {
		ua, ub := da.Scale(1/la), db.Scale(1/lb)
		cosHalf := math.Sqrt(math.Max(0, (1+ua.Dot(ub))/2))
		if cosHalf > math.Cos(85*math.Pi/180) {
			grow := 1 / cosHalf
			lo := r3.NewVec(math.Min(ua.X, ub.X), math.Min(ua.Y, ub.Y), math.Min(ua.Z, ub.Z))
			hi := r3.NewVec(math.Max(ua.X, ub.X), math.Max(ua.Y, ub.Y), math.Max(ua.Z, ub.Z))
			ext := func(l, h float64) (float64, float64) {
				return math.Min(l, l*grow) - 1e-9, math.Max(h, h*grow) + 1e-9
			}
			lx, hx := ext(lo.X, hi.X)
			ly, hy := ext(lo.Y, hi.Y)
			lz, hz := ext(lo.Z, hi.Z)
			box = [2]r3.Vec{r3.NewVec(lx, ly, lz), r3.NewVec(hx, hy, hz)}
		}
	}
	e.cone[i][k] = box
	e.coneOK[i][k] = true
	return box
}

// pairSeparated decides the common pairs without the full classifier, by the
// facets' interned exact planes and exact adaptive orientation signs over the
// held floats:
//
//   - a facet with no plane fails;
//   - one shared edge, two planes: the planes meet only in the edge's line,
//     which each facet meets only in that edge;
//   - no shared vertex or one, two planes, and the unshared corners of one
//     facet strictly on one side of the other's plane: that facet meets the
//     plane, and so the other facet, only at what they share;
//   - one plane, decided in it: a shared edge is embedded exactly when the
//     apexes lie strictly on opposite sides of it, and otherwise an edge line
//     of one facet, through every shared vertex, that leaves every unshared
//     corner of the other strictly outside separates them.
//
// Anything else is left undecided, for the classifier.
func (e *heldEmbedding) pairSeparated(i, j int, sharedI, sharedJ []int) (bool, bool) {
	ti, tj := e.h.Tris[i], e.h.Tris[j]
	otherI, otherJ := unshared(ti, sharedI), unshared(tj, sharedJ)
	// strictSide reports whether pts lie strictly on one side of facet f's
	// plane.
	strictSide := func(f int, pts []int) bool {
		s0 := 0
		for _, p := range pts {
			s := e.planeSign(f, p)
			if s == 0 || (s0 != 0 && s != s0) {
				return false
			}
			s0 = s
		}
		return true
	}
	pi, pj := e.planeOf(i), e.planeOf(j)
	if pi == -2 || pj == -2 {
		return false, true
	}
	coplanar := pi == pj
	if len(sharedI) == 2 {
		a, b := ti[sharedI[0]], ti[sharedI[1]]
		if !coplanar {
			return true, true
		}
		u, v, _ := e.planeAxes(i)
		return e.orient2(a, b, otherI[0], u, v)*e.orient2(a, b, otherJ[0], u, v) < 0, true
	}
	if !coplanar {
		if strictSide(i, otherJ) || strictSide(j, otherI) {
			return true, true
		}
		return false, false
	}
	u, v, ok := e.planeAxes(i)
	if !ok {
		return false, true
	}
	var shared []int
	for _, k := range sharedI {
		shared = append(shared, ti[k])
	}
	if e.edgeSeparates(ti, otherJ, shared, u, v) || e.edgeSeparates(tj, otherI, shared, u, v) {
		return true, true
	}
	return false, false
}

// unshared lists the corners of t whose positions are not in shared.
func unshared(t [3]int, shared []int) []int {
	var out []int
	for k, x := range t {
		if !slices.Contains(shared, k) {
			out = append(out, x)
		}
	}
	return out
}

// planeOf interns facet i's exact plane: its normal reduced to the primitive
// integer direction with a positive leading component, and that direction's
// exact offset at one corner. Two facets share an id exactly when they lie in
// one plane, whichever way each is wound. A facet with no plane answers -2.
func (e *heldEmbedding) planeOf(i int) int {
	if e.plane[i] != -1 {
		return e.plane[i]
	}
	_, x, n := e.facet(i)
	if zeroXpt(n) {
		e.plane[i] = -2
		return -2
	}
	g := new(big.Int).GCD(nil, nil, new(big.Int).Abs(n.X), new(big.Int).Abs(n.Y))
	g.GCD(nil, nil, g, new(big.Int).Abs(n.Z))
	d := [3]*big.Int{new(big.Int).Quo(n.X, g), new(big.Int).Quo(n.Y, g), new(big.Int).Quo(n.Z, g)}
	lead := d[0].Sign()
	if lead == 0 {
		lead = d[1].Sign()
	}
	if lead == 0 {
		lead = d[2].Sign()
	}
	if lead < 0 {
		for k := range d {
			d[k].Neg(d[k])
		}
	}
	off := new(big.Int).Mul(d[0], x[0].X)
	off.Add(off, new(big.Int).Mul(d[1], x[0].Y))
	off.Add(off, new(big.Int).Mul(d[2], x[0].Z))
	key := d[0].String() + "," + d[1].String() + "," + d[2].String() + "|" + new(big.Rat).SetFrac(off, x[0].W).RatString()
	id, ok := e.planes[key]
	if !ok {
		id = len(e.planes)
		e.planes[key] = id
	}
	e.plane[i] = id
	return id
}

// planeAxes is the projection that keeps facet i's plane non-degenerate, or
// false for a facet with no plane.
func (e *heldEmbedding) planeAxes(i int) (int, int, bool) {
	_, _, n := e.facet(i)
	if zeroXpt(n) {
		return 0, 0, false
	}
	u, v := ProjAxes(n)
	return u, v, true
}

// edgeSeparates reports whether some edge line of t through every shared
// vertex leaves every point of pts strictly on the side away from t's third
// corner, in the (u, v) projection. The other facet then lies, apart from the
// shared vertices on the line, strictly outside t.
func (e *heldEmbedding) edgeSeparates(t [3]int, pts, shared []int, u, v int) bool {
	for k := range 3 {
		a, b, c := t[k], t[(k+1)%3], t[(k+2)%3]
		if len(shared) == 1 && shared[0] != a && shared[0] != b {
			continue
		}
		inner := e.orient2(a, b, c, u, v)
		if inner == 0 {
			return false
		}
		out := true
		for _, p := range pts {
			if e.orient2(a, b, p, u, v) != -inner {
				out = false
				break
			}
		}
		if out {
			return true
		}
	}
	return false
}

// orient2 is the exact orientation of held vertices a, b, c projected on axes
// (u, v): a float evaluation whose error bound decides it, else rationals.
func (e *heldEmbedding) orient2(a, b, c, u, v int) int {
	vs := e.h.Verts
	pa := [2]float64{AxisCoord(vs[a], u), AxisCoord(vs[a], v)}
	pb := [2]float64{AxisCoord(vs[b], u), AxisCoord(vs[b], v)}
	pc := [2]float64{AxisCoord(vs[c], u), AxisCoord(vs[c], v)}
	if s := Orient2Float(pa, pb, pc); s != 0 {
		return s
	}
	r := func(x float64) *big.Rat { return polynomial.MustRatOf(x) }
	bu := new(big.Rat).Sub(r(pb[0]), r(pa[0]))
	bv := new(big.Rat).Sub(r(pb[1]), r(pa[1]))
	cu := new(big.Rat).Sub(r(pc[0]), r(pa[0]))
	cv := new(big.Rat).Sub(r(pc[1]), r(pa[1]))
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu)).Sign()
}

// planeSign is the exact side of held vertex p against facet f's plane: the
// float filter first, then f's cached exact normal.
func (e *heldEmbedding) planeSign(f, p int) int {
	t, v := e.h.Tris[f], e.h.Verts
	if s, certain := proof.OrientSignFloat(v[t[0]], v[t[1]], v[t[2]], v[p]); certain {
		return s
	}
	_, x, n := e.facet(f)
	return proof.XdotSign(n, proof.Xsub(e.lift(p), x[0]))
}

// facetFailures returns the facets j that facet i meets improperly, and
// whether i itself has collapsed to no plane.
func (e *heldEmbedding) facetFailures(i int, firstOnly bool) ([]int, bool, error) {
	if _, _, n := e.facet(i); zeroXpt(n) {
		return nil, true, nil
	}
	cands, err := e.candidates(i)
	if err != nil {
		return nil, false, err
	}
	var bad []int
	for _, j := range cands {
		if err := e.budget.Step(); err != nil {
			return nil, false, err
		}
		if j == i || e.pairOK(i, j) {
			continue
		}
		bad = append(bad, j)
		if firstOnly {
			break
		}
	}
	return bad, false, nil
}

// vertexFailures counts the failing pairs that involve a facet incident to v,
// each pair once, plus each incident facet that has collapsed to no plane.
func (e *heldEmbedding) vertexFailures(v int) (int, error) {
	n := 0
	for _, f := range e.inc[v] {
		bad, collapsed, err := e.facetFailures(f, false)
		if err != nil {
			return 0, err
		}
		if collapsed {
			n++
			continue
		}
		for _, j := range bad {
			if j < f && slices.Contains(e.inc[v], j) {
				continue
			}
			n++
		}
	}
	return n, nil
}

// offenders scans the query facets and returns, in ascending order, the
// movable vertices of every failing facet and of every facet it fails
// against, and the failing query facets themselves.
func (e *heldEmbedding) offenders(query []int) ([]int, []int, error) {
	set := map[int]struct{}{}
	var failing []int
	for _, i := range query {
		bad, collapsed, err := e.facetFailures(i, false)
		if err != nil {
			return nil, nil, err
		}
		if !collapsed && len(bad) == 0 {
			continue
		}
		failing = append(failing, i)
		e.addMovable(set, i)
		for _, j := range bad {
			e.addMovable(set, j)
		}
	}
	out := make([]int, 0, len(set))
	for v := range set {
		out = append(out, v)
	}
	slices.Sort(out)
	return out, failing, nil
}

func (e *heldEmbedding) addMovable(set map[int]struct{}, f int) {
	for _, v := range e.h.Tris[f] {
		if e.h.Movable[v] {
			set[v] = struct{}{}
		}
	}
}

// improve moves v to the corner of its exact point's float box that leaves
// the fewest failing pairs among the facets incident to v, when that is fewer
// than where v sits now; ties go to the earlier corner. A corner another held
// vertex occupies is skipped. Each move strictly lowers the mesh's count of
// failing pairs — only pairs touching v's facets change — so the search
// cannot cycle.
func (e *heldEmbedding) improve(v int) (bool, error) {
	orig := e.h.Verts[v]
	best, err := e.vertexFailures(v)
	if err != nil || best == 0 {
		return false, err
	}
	bestAt := orig
	// Each trial moves only v, so putting v back restores every incident
	// facet's positions, and with them the stamps they carried.
	var origGen []uint64
	if e.memoOn {
		origGen = e.stamps(v)
	}
	for _, c := range FloatBoxCorners(e.h.Exact[v]) {
		if err := e.budget.Step(); err != nil {
			return false, err
		}
		if c == orig {
			continue
		}
		if _, ok := e.taken[c]; ok {
			continue
		}
		e.setVert(v, c)
		n, err := e.vertexFailures(v)
		e.setVert(v, orig)
		if e.memoOn {
			e.restamp(v, origGen)
		}
		if err != nil {
			return false, err
		}
		if n < best {
			best, bestAt = n, c
		}
	}
	if bestAt == orig {
		return false, nil
	}
	e.setVert(v, bestAt)
	return true, nil
}

// FloatBoxCorners lists the binary64 points within an ulp of p per coordinate
// that bracket it: per coordinate the float at or below and the float at or
// above, one value when the coordinate is itself a float. Order is fixed — x
// before y before z, the lower float before the upper — so the search that
// reads it is deterministic.
func FloatBoxCorners(p proof.Xpt) []r3.Vec {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	x, y, z := floatBracket(px), floatBracket(py), floatBracket(pz)
	out := make([]r3.Vec, 0, 8)
	for _, a := range x {
		for _, b := range y {
			for _, c := range z {
				out = append(out, r3.NewVec(a, b, c))
			}
		}
	}
	return out
}

func floatBracket(r *big.Rat) []float64 {
	f, _ := r.Float64()
	switch polynomial.MustRatOf(f).Cmp(r) {
	case 0:
		return []float64{f}
	case -1:
		return []float64{f, math.Nextafter(f, math.Inf(1))}
	default:
		return []float64{math.Nextafter(f, math.Inf(-1)), f}
	}
}

// buildGrid registers every facet's padded box in a uniform grid sized to
// about one cell per facet.
func (e *heldEmbedding) buildGrid() error {
	maxAbs, maxDev := 0.0, 0.0
	for v, p := range e.h.Verts {
		if err := e.budget.Step(); err != nil {
			return err
		}
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))))
		if !e.h.Movable[v] {
			continue
		}
		q := e.h.Exact[v].Vec()
		maxDev = math.Max(maxDev, math.Max(math.Abs(p.X-q.X), math.Max(math.Abs(p.Y-q.Y), math.Abs(p.Z-q.Z))))
	}
	// A corner lies within an ulp of the exact point, and the exact point
	// within maxDev (plus that rounding's own ulp) of the first rounding, so
	// no move leaves this pad.
	e.pad = 2*maxDev + 8*proofbound.UlpOf(maxAbs)

	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, t := range e.h.Tris {
		b := TriBox(e.h.Verts, t)
		lo = r3.NewVec(math.Min(lo.X, b[0].X), math.Min(lo.Y, b[0].Y), math.Min(lo.Z, b[0].Z))
		hi = r3.NewVec(math.Max(hi.X, b[1].X), math.Max(hi.Y, b[1].Y), math.Max(hi.Z, b[1].Z))
	}
	lo = lo.Sub(r3.NewVec(e.pad, e.pad, e.pad))
	hi = hi.Add(r3.NewVec(e.pad, e.pad, e.pad))
	ext := hi.Sub(lo)
	n := float64(len(e.h.Tris))
	longest := math.Max(ext.X, math.Max(ext.Y, ext.Z))
	cell := math.Cbrt(ext.X * ext.Y * ext.Z / n)
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = longest / math.Cbrt(n)
	}
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = 1
	}
	cell = math.Max(cell, longest/1024)
	dims := [3]int{}
	for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
		dims[k] = max(1, min(1024, int(math.Ceil(x/cell))))
	}
	for dims[0]*dims[1]*dims[2] > 8*len(e.h.Tris)+64 {
		cell *= 2
		for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
			dims[k] = max(1, int(math.Ceil(x/cell)))
		}
	}
	e.origin, e.cell, e.dims = lo, cell, dims
	e.cells = make([][]int32, dims[0]*dims[1]*dims[2])
	for i, t := range e.h.Tris {
		if err := e.budget.Step(); err != nil {
			return err
		}
		c0, c1 := e.cellRange(TriBox(e.h.Verts, t))
		span := (c1[0] - c0[0] + 1) * (c1[1] - c0[1] + 1) * (c1[2] - c0[2] + 1)
		if span > heldGridCellSpan {
			e.wide = append(e.wide, int32(i))
			continue
		}
		for x := c0[0]; x <= c1[0]; x++ {
			for y := c0[1]; y <= c1[1]; y++ {
				for z := c0[2]; z <= c1[2]; z++ {
					k := (x*dims[1]+y)*dims[2] + z
					e.cells[k] = append(e.cells[k], int32(i))
				}
			}
		}
	}
	return nil
}

func (e *heldEmbedding) cellOf(x float64, axis int) int {
	o := [3]float64{e.origin.X, e.origin.Y, e.origin.Z}[axis]
	c := int(math.Floor((x - o) / e.cell))
	return max(0, min(e.dims[axis]-1, c))
}

func (e *heldEmbedding) cellRange(b [2]r3.Vec) ([3]int, [3]int) {
	lo := b[0].Sub(r3.NewVec(e.pad, e.pad, e.pad))
	hi := b[1].Add(r3.NewVec(e.pad, e.pad, e.pad))
	return [3]int{e.cellOf(lo.X, 0), e.cellOf(lo.Y, 1), e.cellOf(lo.Z, 2)},
		[3]int{e.cellOf(hi.X, 0), e.cellOf(hi.Y, 1), e.cellOf(hi.Z, 2)}
}

// candidates lists, once each, the facets registered in any cell facet i's
// padded box reaches, plus every wide facet.
func (e *heldEmbedding) candidates(i int) ([]int, error) {
	e.stampID++
	id := e.stampID
	// The list is reused across calls: each caller is done with it before
	// the next query.
	out := e.cands[:0]
	add := func(j int32) {
		if e.stamp[j] == id {
			return
		}
		e.stamp[j] = id
		out = append(out, int(j))
	}
	c0, c1 := e.cellRange(e.triBox(i))
	for x := c0[0]; x <= c1[0]; x++ {
		for y := c0[1]; y <= c1[1]; y++ {
			for z := c0[2]; z <= c1[2]; z++ {
				if err := e.budget.Step(); err != nil {
					return nil, err
				}
				for _, j := range e.cells[(x*e.dims[1]+y)*e.dims[2]+z] {
					add(j)
				}
			}
		}
	}
	for _, j := range e.wide {
		add(j)
	}
	e.cands = out
	return out, nil
}

// ExactRigidImage is the exact image of p under the linear part b followed by
// the translation t, every product and sum taken in rationals: the point a
// float evaluation of the motion only approximates.
func ExactRigidImage(b r3.Basis, t, p r3.Vec) proof.Xpt {
	coord := func(ex, ey, ez, tc float64) *big.Rat {
		s := new(big.Rat).Mul(polynomial.MustRatOf(ex), polynomial.MustRatOf(p.X))
		s.Add(s, new(big.Rat).Mul(polynomial.MustRatOf(ey), polynomial.MustRatOf(p.Y)))
		s.Add(s, new(big.Rat).Mul(polynomial.MustRatOf(ez), polynomial.MustRatOf(p.Z)))
		return s.Add(s, polynomial.MustRatOf(tc))
	}
	x := coord(b.EX.X, b.EY.X, b.EZ.X, t.X)
	y := coord(b.EX.Y, b.EY.Y, b.EZ.Y, t.Y)
	z := coord(b.EX.Z, b.EY.Z, b.EZ.Z, t.Z)
	w := new(big.Int).Set(x.Denom())
	for _, r := range []*big.Rat{y, z} {
		g := new(big.Int).GCD(nil, nil, w, r.Denom())
		w.Mul(w, new(big.Int).Quo(r.Denom(), g))
	}
	num := func(r *big.Rat) *big.Int {
		return new(big.Int).Mul(r.Num(), new(big.Int).Quo(w, r.Denom()))
	}
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{X: num(x), Y: num(y), Z: num(z), W: w}))
}
