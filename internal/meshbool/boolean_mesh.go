package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// BoolMesh is one operand's tessellation prepared for exact work: float
// vertices lifted to rationals, exact facet normals and their lazy rounded-
// float cache, facet boxes for pair pruning, and the global source-face id
// each facet remembers.
type BoolMesh struct {
	Verts       []r3.Vec
	Xverts      []proof.Xpt
	Tris        [][3]int
	Norms       []proof.Xpt
	Fnorms      []r3.Vec
	FnormsReady []bool
	Boxes       [][2]r3.Vec
	Src         []int
	// VertexBound is β(v) per vertex and FacetBound δ(t) per facet, the
	// largest of its three corners' β (docs/faceted-vertex-bounds-design.md
	// §2): a true boundary point lies within VertexBound[v] of vertex v, and
	// facet t's true piece within FacetBound[t] of the facet. The rim, the
	// cutter and the stitch compose every result vertex's own bound from them
	// (§3).
	VertexBound []float64
	FacetBound  []float64
	// Gate is the pair's chord tolerance when this operand restates a held
	// mesh (a boolean result or a mitred sweep), and zero for every other
	// operand. A gated operand's facets that meet the other operand, or come
	// within the pre-pass slack of it, must carry a FacetBound no larger than
	// Gate (docs/faceted-vertex-bounds-design.md §5, RefuseCoarseHeldContact).
	Gate float64
	// owner maps each directed mesh edge to the facet that walks it, so the
	// twin across a facet edge is one lookup. The graze-or-crossing call
	// needs it: whether an in-plane edge grazes the other operand or crosses
	// it is a property of the edge's TWO adjacent facets, which no facet pair
	// can see (docs/evaluator-design.md §9).
	Owner map[[2]int]int
	// parity caches this operand's vertex projections for the exact ray-parity
	// kernel, which every uncut-component seed, unanchored region probe and
	// near-miss depth witness of one operation asks about THIS operand. The
	// cache is empty until a query sweeps an axis, and it lives exactly as long
	// as the prepared operand does.
	Parity *ParityMesh
}

// twinFacet returns the facet across edge k of facet f — the neighbour a
// watertight mesh's reversed directed edge names.
func (bm *BoolMesh) TwinFacet(f, k int) (int, bool) {
	tri := bm.Tris[f]
	t, ok := bm.Owner[[2]int{tri[(k+1)%3], tri[k]}]
	return t, ok && t != f
}

// prepareFloatNormal computes the correctly rounded float value of one exact
// facet normal. RunContactBatch calls it before starting workers, so workers
// only read the cache; ContactMemo's serial path does the same before use.
func (bm *BoolMesh) PrepareFloatNormal(i int) {
	if bm.FnormsReady[i] {
		return
	}
	bm.Fnorms[i] = bm.Norms[i].Vec()
	bm.FnormsReady[i] = true
}

// TriBox is the facet's float bounding box — float min/max are exact, so the
// box is a true bound.
func TriBox(verts []r3.Vec, tri [3]int) [2]r3.Vec {
	lo, hi := verts[tri[0]], verts[tri[0]]
	for _, vi := range tri[1:] {
		v := verts[vi]
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return [2]r3.Vec{lo, hi}
}

func BoxesOverlap(a, b [2]r3.Vec) bool {
	return a[0].X <= b[1].X && b[0].X <= a[1].X &&
		a[0].Y <= b[1].Y && b[0].Y <= a[1].Y &&
		a[0].Z <= b[1].Z && b[0].Z <= a[1].Z
}

// KeptFacet is one facet of the boolean result, still exact: its corners,
// the global source-face id it approximates, and which operand it came from.
type KeptFacet struct {
	V   [3]proof.Xpt
	Src int
	// Beta is each corner's pre-weld bound (docs/faceted-vertex-bounds-
	// design.md §3.1–§3.3): an operand vertex's own β, a rim vertex's
	// RimBound, or, for a point the cutter placed on the operand facet, that
	// facet's δ. Each is a two-sided claim the corner carries into the stitch.
	Beta [3]float64
}

// BooleanKeep is the classification table of §9: which side of the other
// solid each operand's boundary keeps, and whether the kept B side flips
// orientation (a cut turns the tool's skin inside out).
func BooleanKeep(op OperationKind) (bool, bool, bool, error) {
	switch op {
	case OpUnion:
		return false, false, false, nil
	case OpIntersect:
		return true, true, false, nil
	case OpCut:
		return false, true, true, nil
	default:
		return false, false, false, fmt.Errorf(`%w: %q is not a boolean op`, decaderr.ErrBooleanFailed, op)
	}
}

// PairContact is one classified facet pair, kept whole so the decisions the
// PAIR cannot make are made later with the mesh in hand.
type PairContact struct {
	I, J   int // the facet in ma, the facet in mb
	Ta, Tb [3]r3.Vec
	C      TriContact
}

// RimBound is docs/faceted-vertex-bounds-design.md §3.2's pre-weld bound of
// a rim vertex one facet pair creates: the true pieces of the two facets lie
// within deltaA and deltaB of their planes, the two slabs meet in a tube of
// half-width (deltaA + deltaB)/sin θ about the exact crossing line, and the
// true rim point lies in that tube. sin2 is THIS pair's exact squared sine.
// Two exactly held facets amplify nothing and answer 0 at any angle; a
// crossing with no proven positive sine answers +Inf, which the caller
// refuses.
func RimBound(deltaA, deltaB float64, sin2 *big.Rat) float64 {
	d := proofbound.AbsSumUpper(deltaA, deltaB)
	if d <= 0 {
		return 0
	}
	if sin2 == nil {
		return math.Inf(1)
	}
	return proofbound.DivUpper(d, SinLowerBound(sin2))
}

// MeshBoolean runs the exact-predicate boolean over two prepared operand
// tessellations. It returns the kept, still-exact facets, each corner carrying
// its pre-weld bound, and the largest RimBound any contact segment it used
// gives a rim vertex — the figure the caller refuses at the pair diameter
// (docs/faceted-vertex-bounds-design.md §3.2).
func MeshBoolean(ctx context.Context, op OperationKind, ma, mb *BoolMesh, memo *ContactMemo) ([]KeptFacet, float64, error) {
	wantA, wantB, flipB, err := BooleanKeep(op)
	if err != nil {
		return nil, 0, err
	}

	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]Xseg{}
	cutsB := map[int][]Xseg{}
	var pointTouches []proof.Xpt
	var segEnds []proof.Xpt
	var inPlane []PairContact
	// rims maps each rim vertex (a contact segment's endpoint, by exact key)
	// to the largest RimBound over the facet pairs whose segment ends there.
	rims := map[string]float64{}
	maxRim := 0.0
	// touchedA and touchedB are the facets of each operand that meet the
	// other, which the held-mesh gate reads once classification is done.
	var touchedA, touchedB []int
	work := 0
	contacts := NewContactBatchExecutor(ctx, ma, mb, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		ta := TriCorners(ma, i)
		tb := TriCorners(mb, j)
		if c.Kind == ContactRegion {
			return ErrUnclassifiableContact(`two operand facets overlap in one plane`)
		}
		if c.Kind == ContactNone {
			return nil
		}
		touchedA = append(touchedA, i)
		touchedB = append(touchedB, j)
		if c.Kind == ContactPoint {
			pointTouches = append(pointTouches, c.P0)
			return nil
		}
		segEnds = append(segEnds, c.P0, c.P1)
		rim := RimBound(ma.FacetBound[i], mb.FacetBound[j], c.Sin2)
		maxRim = max(maxRim, rim)
		for _, p := range []proof.Xpt{c.P0, c.P1} {
			k := p.Key()
			if prev, ok := rims[k]; !ok || rim > prev {
				rims[k] = rim
			}
		}
		if c.EdgeA >= 0 || c.EdgeB >= 0 {
			// The segment runs ALONG a facet edge. Graze or crossing is not
			// decidable here — hold it for the mesh-level call below.
			inPlane = append(inPlane, PairContact{I: i, J: j, Ta: ta, Tb: tb, C: c})
			return nil
		}
		cutsA[i] = append(cutsA[i], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnA, BOnEdge: c.P1OnA,
			Partner: tb,
		})
		cutsB[j] = append(cutsB[j], Xseg{
			A: c.P0, B: c.P1,
			AOnEdge: c.P0OnB, BOnEdge: c.P1OnB,
			Partner: ta,
		})
		return nil
	})
	for i := range ma.Tris {
		for j := range mb.Tris {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, 0, err
				}
			}
			if !BoxesOverlap(ma.Boxes[i], mb.Boxes[j]) {
				continue
			}
			if err := contacts.Add(i, j); err != nil {
				return nil, 0, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		return nil, 0, err
	}
	// The held-mesh gate runs on the classification's own answer and before
	// any facet is cut (docs/faceted-vertex-bounds-design.md §5).
	if err := RefuseCoarseHeldContact(ma, 0, touchedA); err != nil {
		return nil, 0, err
	}
	if err := RefuseCoarseHeldContact(mb, 1, touchedB); err != nil {
		return nil, 0, err
	}

	// The graze-or-crossing call, made once, OUTSIDE the pair loop: an in-plane
	// facet edge grazes the other operand exactly when the edge's two adjacent
	// facets lie strictly on ONE side of the other facet's plane — the operand's
	// boundary touches the plane and comes back, a tangency no side
	// classification can be proven for. Apexes that STRADDLE the plane are an
	// ordinary crossing: the boundary genuinely passes through, and the segment
	// is a real rim of the result.
	blockedA := map[[2]int]struct{}{}
	blockedB := map[[2]int]struct{}{}
	for i, p := range inPlane {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, 0, err
			}
		}
		if p.C.EdgeA >= 0 {
			key, crossing, err := EdgeCrosses(ma, p.I, p.C.EdgeA, p.Tb)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, ErrUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedA[key] = struct{}{}
		}
		if p.C.EdgeB >= 0 {
			key, crossing, err := EdgeCrosses(mb, p.J, p.C.EdgeB, p.Ta)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, ErrUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedB[key] = struct{}{}
		}
		// A crossing segment subdivides whichever facet does NOT already own it
		// as an edge — a segment lying along a facet's own boundary cuts nothing
		// off it. Its regions classify by exact parity, never by the partner
		// facet's plane: the other operand's boundary there is the DIHEDRAL
		// between two facets, and one plane of it decides nothing.
		if p.C.EdgeA < 0 {
			cutsA[p.I] = append(cutsA[p.I], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnA, BOnEdge: p.C.P1OnA,
				Partner: p.Tb, ViaParity: true,
			})
		}
		if p.C.EdgeB < 0 {
			cutsB[p.J] = append(cutsB[p.J], Xseg{
				A: p.C.P0, B: p.C.P1,
				AOnEdge: p.C.P0OnB, BOnEdge: p.C.P1OnB,
				Partner: p.Ta, ViaParity: true,
			})
		}
	}

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		ends := map[string]struct{}{}
		for _, p := range segEnds {
			ends[p.Key()] = struct{}{}
		}
		for _, p := range pointTouches {
			if _, ok := ends[p.Key()]; !ok {
				return nil, 0, ErrUnclassifiableContact(`the operand boundaries touch at an isolated point`)
			}
		}
	}

	var kept []KeptFacet
	if err := ctx.Err(); err != nil {
		return nil, 0, err
	}
	keep, err := KeepSide(ctx, ma, mb, cutsA, blockedA, rims, wantA, false)
	if err != nil {
		return nil, 0, err
	}
	kept = append(kept, keep...)
	keep, err = KeepSide(ctx, mb, ma, cutsB, blockedB, rims, wantB, flipB)
	if err != nil {
		return nil, 0, err
	}
	return append(kept, keep...), maxRim, nil
}

// EdgeCrosses decides whether edge k of facet f — which lies exactly in the
// partner facet's plane — GRAZES that plane or genuinely CROSSES it, and
// returns the undirected mesh-edge key either way. The verdict is read off the
// edge's two adjacent facets: their apex vertices strictly on one side is a
// graze; straddling is a crossing. An apex exactly ON the plane proves nothing
// (the facet lies in it), so it reads as a graze — reject-only.
func EdgeCrosses(m *BoolMesh, f, k int, partner [3]r3.Vec) ([2]int, bool, error) {
	tri := m.Tris[f]
	u, v := tri[k], tri[(k+1)%3]
	key := [2]int{min(u, v), max(u, v)}
	twin, ok := m.TwinFacet(f, k)
	if !ok {
		return key, false, fmt.Errorf(`%w: an in-plane facet edge has no twin facet`, decaderr.ErrBooleanFailed)
	}
	tt := m.Tris[twin]
	apex := m.Verts[tri[0]+tri[1]+tri[2]-u-v]
	apexTwin := m.Verts[tt[0]+tt[1]+tt[2]-u-v]
	s0 := proof.OrientSign(partner[0], partner[1], partner[2], apex)
	s1 := proof.OrientSign(partner[0], partner[1], partner[2], apexTwin)
	return key, s0*s1 < 0, nil
}

// BlockedBetween reports whether the mesh edge two adjacent facets share
// carries an in-plane CROSSING contact. The other operand's boundary passes
// exactly along that edge, so the two facets sit on opposite sides of it: they
// must not flood-fill into one classification, or one of them inherits the
// other's answer.
func BlockedBetween(m *BoolMesh, a, b int, blocked map[[2]int]struct{}) bool {
	if len(blocked) == 0 {
		return false
	}
	ta, tb := m.Tris[a], m.Tris[b]
	for k := range 3 {
		u, v := ta[k], ta[(k+1)%3]
		if _, ok := blocked[[2]int{min(u, v), max(u, v)}]; !ok {
			continue
		}
		if SharesVertices(tb, u, v) {
			return true
		}
	}
	return false
}

func SharesVertices(t [3]int, u, v int) bool {
	hasU, hasV := false, false
	for _, x := range t {
		switch x {
		case u:
			hasU = true
		case v:
			hasV = true
		}
	}
	return hasU && hasV
}

func TriCorners(m *BoolMesh, i int) [3]r3.Vec {
	t := m.Tris[i]
	return [3]r3.Vec{m.Verts[t[0]], m.Verts[t[1]], m.Verts[t[2]]}
}

func XtriCorners(m *BoolMesh, i int) [3]proof.Xpt {
	t := m.Tris[i]
	return [3]proof.Xpt{m.Xverts[t[0]], m.Xverts[t[1]], m.Xverts[t[2]]}
}

// KeepSide classifies one operand's boundary against the other solid and
// returns the kept facets: subdivided pieces for the cut facets, whole
// facets for the uncut regions (classified per connected component by exact
// ray parity — the classification is constant on a component that crosses
// nothing). Every kept corner carries its pre-weld bound (KeptFacet.Beta):
// an operand vertex keeps its own β, and a subdivision vertex takes rims'
// bound when it is a rim vertex and its facet's δ otherwise (CutCornerBound).
func KeepSide(ctx context.Context, m, other *BoolMesh, cuts map[int][]Xseg, blocked map[[2]int]struct{}, rims map[string]float64, wantInside, flip bool) ([]KeptFacet, error) {
	var kept []KeptFacet
	emit := func(tri [3]proof.Xpt, beta [3]float64, src int) {
		if flip {
			tri[1], tri[2] = tri[2], tri[1]
			beta[1], beta[2] = beta[2], beta[1]
		}
		kept = append(kept, KeptFacet{V: tri, Src: src, Beta: beta})
	}

	// The cut facets: exact subdivision along the contact chains, then the
	// exact local side-of-contact classification per region.
	for i := range m.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		segs, ok := cuts[i]
		if !ok {
			continue
		}
		regions, err := CutTriangle(ctx, XtriCorners(m, i), m.Norms[i], segs)
		if err != nil {
			return nil, err
		}
		corners := m.Tris[i]
		cornerKeys := [3]string{m.Xverts[corners[0]].Key(), m.Xverts[corners[1]].Key(), m.Xverts[corners[2]].Key()}
		for _, reg := range regions {
			inside, err := ClassifyRegion(ctx, reg, other)
			if err != nil {
				return nil, err
			}
			if inside != wantInside {
				continue
			}
			for _, tri := range reg.Tris {
				var beta [3]float64
				for k, p := range tri {
					beta[k] = CutCornerBound(m, i, cornerKeys, p.Key(), rims)
				}
				emit(tri, beta, m.Src[i])
			}
		}
	}

	// The uncut facets: constant classification per connected uncut
	// component, one exact parity seed each.
	comp := make([]int, len(m.Tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := FacetAdjacencyContext(ctx, m.Tris)
	if err != nil {
		return nil, err
	}
	next := 0
	componentWork := 0
	all := AllFacets(other)
	for i := range m.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if _, cut := cuts[i]; cut || comp[i] != -1 {
			continue
		}
		id := next
		next++
		queue := []int{i}
		comp[i] = id
		var members []int
		for len(queue) > 0 {
			componentWork++
			if componentWork%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			f := queue[0]
			queue = queue[1:]
			members = append(members, f)
			for _, nb := range adj[f] {
				if _, cut := cuts[nb]; cut || comp[nb] != -1 {
					continue
				}
				if BlockedBetween(m, f, nb, blocked) {
					continue
				}
				comp[nb] = id
				queue = append(queue, nb)
			}
		}
		// Every facet has a real interior to probe: prepBoolMesh refused the
		// operand outright if any facet collapsed.
		seed := XtriCorners(m, members[0])
		probe := XCentroid(seed[0], seed[1], seed[2])
		inside, onBoundary, err := MeshParityPreparedContext(ctx, probe, other.Parity, all)
		if err != nil {
			return nil, err
		}
		if onBoundary {
			return nil, ErrUnclassifiableContact(`an operand facet touches the other operand's boundary`)
		}
		if inside != wantInside {
			continue
		}
		for _, f := range members {
			t := m.Tris[f]
			emit(XtriCorners(m, f), [3]float64{m.VertexBound[t[0]], m.VertexBound[t[1]], m.VertexBound[t[2]]}, m.Src[f])
		}
	}
	return kept, nil
}

// CutCornerBound is the pre-weld bound of one corner of a piece CutTriangle
// cut from facet i (docs/faceted-vertex-bounds-design.md §3.1–§3.3), the
// corner named by its exact key: one of the facet's own corners keeps that
// vertex's β, a rim vertex takes the largest RimBound recorded for it, a
// point that is both takes the larger, and any other point — a split-line or
// edge point the cutter placed on the held facet — lies on the facet and
// takes its δ.
func CutCornerBound(m *BoolMesh, i int, cornerKeys [3]string, key string, rims map[string]float64) float64 {
	beta, known := 0.0, false
	for k, ck := range cornerKeys {
		if ck == key {
			beta, known = max(beta, m.VertexBound[m.Tris[i][k]]), true
		}
	}
	if rim, ok := rims[key]; ok {
		beta, known = max(beta, rim), true
	}
	if !known {
		return m.FacetBound[i]
	}
	return beta
}

// ClassifyRegion decides whether a subdivision region lies inside the other
// solid. A region anchored to a contact chain is decided by the exact side
// of the partner facet's plane — the other solid's boundary IS that facet
// along the shared chain edge, so the side is the answer; an unanchored
// region (an artifact of the loop-opening split lines) falls back to exact
// parity.
func ClassifyRegion(ctx context.Context, reg CutRegion, other *BoolMesh) (bool, error) {
	if reg.HasAnchor {
		switch s := proof.OrientSignMixed(reg.Partner[0], reg.Partner[1], reg.Partner[2], reg.Probe); {
		case s < 0:
			return true, nil
		case s > 0:
			return false, nil
		default:
			return false, fmt.Errorf(`%w: a region probe landed on its contact plane`, decaderr.ErrBooleanFailed)
		}
	}
	inside, onBoundary, err := MeshParityPreparedContext(ctx, reg.Probe, other.Parity, AllFacets(other))
	if err != nil {
		return false, err
	}
	if onBoundary {
		return false, ErrUnclassifiableContact(`a subdivision region touches the other operand's boundary`)
	}
	return inside, nil
}

func AllFacets(m *BoolMesh) []int {
	out := make([]int, len(m.Tris))
	for i := range out {
		out[i] = i
	}
	return out
}

// FacetAdjacencyContext maps each facet to its edge neighbors through the
// watertight mesh's paired directed edges, with bounded cancellation checks.
func FacetAdjacencyContext(ctx context.Context, tris [][3]int) ([][]int, error) {
	owner := map[[2]int]int{}
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj, nil
}

// XCentroid is (a+b+c)/3, exact. Dividing by 3 multiplies the shared
// denominator by 3 rather than dividing the numerators by it, since a
// numerator need not be a multiple of 3 — the same "multiply w, never divide
// the numerator" rule every homogeneous construction here follows.
func XCentroid(a, b, c proof.Xpt) proof.Xpt {
	bwcw := new(big.Int).Mul(b.W, c.W)
	awcw := new(big.Int).Mul(a.W, c.W)
	awbw := new(big.Int).Mul(a.W, b.W)
	axis := func(ax, bx, cx *big.Int) *big.Int {
		s := new(big.Int).Mul(ax, bwcw)
		s.Add(s, new(big.Int).Mul(bx, awcw))
		s.Add(s, new(big.Int).Mul(cx, awbw))
		return s
	}
	w := new(big.Int).Mul(awbw, c.W)
	w.Mul(w, big.NewInt(3))
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{
		X: axis(a.X, b.X, c.X),
		Y: axis(a.Y, b.Y, c.Y),
		Z: axis(a.Z, b.Z, c.Z),
		W: w,
	}))
}
