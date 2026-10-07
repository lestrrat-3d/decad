package meshbool

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// StitchedMesh is the boolean output after welding, conforming and rounding:
// the held float mesh, its per-facet source-face ids, and the proven terms the
// rounding's own error composes from.
type StitchedMesh struct {
	Verts []r3.Vec
	Tris  [][3]int
	Src   []int
	// round is the proven displacement (mm) of one vertex under the final
	// float rounding: no held vertex is farther than this from the exact
	// point it stands for.
	Round float64
	// preArea upper-bounds the area of the surface the rounding acted ON —
	// the stitched surface BEFORE any facet was dropped, and along the whole
	// motion from it to the held mesh. It is what the volume error is charged
	// against, NOT the held mesh's area: a facet the weld collapses is gone
	// from the held mesh, and charging the rounding against what survived
	// would leave that facet's own swept volume out of the bound.
	PreArea float64
	// dropArea upper-bounds the area of the facets the weld collapsed, which
	// the held mesh no longer carries. The reported surface area is short by
	// exactly this much, so the area bound must cover it.
	DropArea float64
	// VertexBound is β(v) per held vertex (docs/faceted-vertex-bounds-design.md
	// §3.4): the largest, over the exact vertices the held one stands for, of
	// that exact vertex's pre-weld bound plus its own rounding distance to the
	// held float, rounded up. An exact vertex held at its own float adds
	// nothing, so an operand vertex keeps its operand β to the bit.
	VertexBound []float64
}

// stitchFacets welds the kept facets by shared exact vertices, makes the
// subdivision conforming (a vertex lying exactly on another facet's edge
// splits that facet — exact incidence, so nothing moves), audits closure,
// and rounds to float64. The audit is the §9 guarantee: every directed edge
// pairs with its reverse, or the boolean fails — never a cracked mesh.
func StitchFacetsContext(ctx context.Context, kept []KeptFacet) (*StitchedMesh, error) {
	if len(kept) == 0 {
		return nil, fmt.Errorf(`%w: the operation leaves no boundary at all`, decaderr.ErrBooleanFailed)
	}
	var xverts []proof.Xpt
	// xbeta is each exact vertex's pre-weld bound: the largest claim any kept
	// facet corner makes for it, raised below by every conforming split that
	// inserts it into another facet's edge.
	var xbeta []float64
	index := map[string]int{}
	// Exact vertices are immutable after construction. Reuse the canonical key
	// when another incident facet carries the same four integer pointers.
	keyByRaw := map[[4]*big.Int]string{}
	addVert := func(p proof.Xpt) int {
		raw := [4]*big.Int{p.X, p.Y, p.Z, p.W}
		k, ok := keyByRaw[raw]
		if !ok {
			k = p.Key()
			keyByRaw[raw] = k
		}
		if i, ok := index[k]; ok {
			return i
		}
		index[k] = len(xverts)
		xverts = append(xverts, p)
		xbeta = append(xbeta, 0)
		return len(xverts) - 1
	}
	var tris [][3]int
	var src []int
	// facetBound is each stitched triangle's δ: the largest corner claim of
	// the kept facet it descends from. A conforming split keeps its parent's
	// figure, since every sub-triangle lies on that held facet.
	var facetBound []float64
	for i, f := range kept {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := addVert(f.V[0]), addVert(f.V[1]), addVert(f.V[2])
		if a == b || b == c || c == a {
			return nil, fmt.Errorf(`%w: a kept facet collapsed`, decaderr.ErrBooleanFailed)
		}
		tris = append(tris, [3]int{a, b, c})
		src = append(src, f.Src)
		for k, vi := range [3]int{a, b, c} {
			xbeta[vi] = max(xbeta[vi], f.Beta[k])
		}
		facetBound = append(facetBound, max(f.Beta[0], f.Beta[1], f.Beta[2]))
	}

	for round := 0; ; round++ {
		if round > 6 {
			return nil, fmt.Errorf(`%w: the conforming pass did not converge`, decaderr.ErrBooleanFailed)
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		split, err := ConformOnce(ctx, xverts, &tris, &src, &facetBound, xbeta)
		if err != nil {
			return nil, err
		}
		if !split {
			break
		}
	}

	// Closure audit: each directed edge exactly once, each with its reverse.
	directed := map[[2]int]int{}
	for _, tri := range tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the stitched boundary does not close`, decaderr.ErrBooleanFailed)
		}
	}

	// Round to float64, welding vertices whose roundings coincide — two exact
	// points closer than an ulp become one held vertex — and drop the facets
	// that collapse under the weld: a collapsed facet's two real directed
	// edges cancel each other, so closure survives, and the final audit
	// re-proves it. A collapsed facet is a zero-area triangle, so it moves
	// neither the volume integral nor the area sum of the HELD mesh — but the
	// facet it stands for was not zero-area before the weld, and what it did
	// carry has to be answered for. Two answers, below: the swept volume and
	// the missing area are charged against the PRE-ROUND surface (preArea,
	// dropArea), and a whole component welded out of existence is REFUSED —
	// there no bound would help, because a lump would be gone from the body
	// (its volume, its place in the lump count, its share of the bounds) while
	// the closure audit, which the surviving components still pass, reports
	// nothing wrong.
	out := &StitchedMesh{}
	floatIdx := map[r3.Vec]int{}
	remap := make([]int, len(xverts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		v := p.Vec()
		fi, ok := floatIdx[v]
		if !ok {
			fi = len(out.Verts)
			floatIdx[v] = fi
			out.Verts = append(out.Verts, v)
		}
		remap[i] = fi
	}

	dropped := make([]bool, len(tris))
	welded := make([][3]int, len(tris))
	for ti, tri := range tris {
		a, b, c := remap[tri[0]], remap[tri[1]], remap[tri[2]]
		welded[ti] = [3]int{a, b, c}
		if a == b || b == c || c == a {
			dropped[ti] = true
			continue
		}
		out.Tris = append(out.Tris, [3]int{a, b, c})
		out.Src = append(out.Src, src[ti])
	}
	if len(out.Tris) == 0 {
		return nil, fmt.Errorf(`%w: the whole result collapsed under rounding`, decaderr.ErrBooleanFailed)
	}
	if err := RefuseWeldedAwayComponent(ctx, tris, dropped); err != nil {
		return nil, err
	}
	if err := keepRoundedEmbedded(ctx, xverts, remap, out); err != nil {
		return nil, err
	}
	// worst is the max PER-COORDINATE distance from an exact vertex to the
	// held float that stands for it — measured AFTER the embedding check, so a
	// vertex it placed at another corner of its float box is charged where it
	// actually sits. The consumers read a 3D distance bound, and all three
	// coordinates can round at once (internal/proofbound/bounds.go,
	// proofbound.Radius3D). A positive rational error can round to zero as
	// float64, so preserve that proof before widening it to a 3D radius.
	worst := new(big.Rat)
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if d := CoordDistance(p, out.Verts[remap[i]]); d.Cmp(worst) > 0 {
			worst = d
		}
	}
	if worst.Sign() > 0 {
		w, _ := worst.Float64()
		out.Round = proofbound.Radius3D(proofbound.ProvenUpRound(w))
	}
	vertexBound, err := HeldVertexBounds(ctx, xverts, xbeta, remap, out.Verts)
	if err != nil {
		return nil, err
	}
	out.VertexBound = vertexBound
	// The pre-round surface: every facet the exact stitch produced, dropped
	// ones included, measured on the held vertices and inflated by the
	// rounding they may each have travelled (internal/proofbound/bounds.go, proofbound.PerturbedAreaUpper).
	out.PreArea = proofbound.PerturbedAreaUpper(out.Verts, welded, out.Round)
	var droppedTris [][3]int
	for ti := range tris {
		if dropped[ti] {
			droppedTris = append(droppedTris, welded[ti])
		}
	}
	out.DropArea = proofbound.PerturbedAreaUpper(out.Verts, droppedTris, out.Round)

	directed = map[[2]int]int{}
	for _, tri := range out.Tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the rounded boundary does not close`, decaderr.ErrBooleanFailed)
		}
	}
	return out, nil
}

// HeldVertexBounds is docs/faceted-vertex-bounds-design.md §3.4's weld, per
// vertex: each exact vertex's own rounding distance to the held float that
// stands for it, read exactly and widened to a 3D radius, is added to that
// exact vertex's pre-weld bound, and a held vertex takes the largest sum over
// the exact vertices welded into it. An exact vertex its float holds exactly
// adds nothing and is not rounded up, so its bound passes through unchanged.
func HeldVertexBounds(ctx context.Context, xverts []proof.Xpt, xbeta []float64, remap []int, held []r3.Vec) ([]float64, error) {
	out := make([]float64, len(held))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		fi := remap[i]
		beta := xbeta[i]
		if gap := CoordDistance(p, held[fi]); gap.Sign() > 0 {
			g, _ := gap.Float64()
			beta = proofbound.AbsSumUpper(beta, proofbound.Radius3D(proofbound.ProvenUpRound(g)))
		}
		if proofbound.IsNonFinite(beta) {
			return nil, fmt.Errorf(`%w: a result vertex's displacement bound is not finite`, decaderr.ErrUnsupported)
		}
		out[fi] = max(out[fi], beta)
	}
	return out, nil
}

// CoordDistance is the exact largest per-coordinate distance from p to v.
func CoordDistance(p proof.Xpt, v r3.Vec) *big.Rat {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	d := new(big.Rat)
	for _, pair := range [][2]*big.Rat{{px, polynomial.MustRatOf(v.X)}, {py, polynomial.MustRatOf(v.Y)}, {pz, polynomial.MustRatOf(v.Z)}} {
		dd := new(big.Rat).Sub(pair[0], pair[1])
		dd.Abs(dd)
		if dd.Cmp(d) > 0 {
			d = dd
		}
	}
	return d
}

// keepRoundedEmbedded runs EnforceHeldEmbedding over the rounded result. A
// held vertex is moved when it differs from an exact vertex it stands for,
// and the search may place it only when it stands for exactly one exact
// vertex: a welded vertex is checked where it sits, never moved, because no
// single float box belongs to it.
func keepRoundedEmbedded(ctx context.Context, xverts []proof.Xpt, remap []int, out *StitchedMesh) error {
	h := HeldRounding{
		Verts:   out.Verts,
		Tris:    out.Tris,
		Exact:   make([]proof.Xpt, len(out.Verts)),
		Moved:   make([]bool, len(out.Verts)),
		Movable: make([]bool, len(out.Verts)),
	}
	count := make([]int, len(out.Verts))
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		fi := remap[i]
		count[fi]++
		h.Exact[fi] = p
		if CoordDistance(p, out.Verts[fi]).Sign() != 0 {
			h.Moved[fi] = true
		}
	}
	for fi, n := range count {
		if n > 1 {
			h.Moved[fi] = true
		}
		h.Movable[fi] = n == 1 && h.Moved[fi]
	}
	_, err := EnforceHeldEmbedding(ctx, h)
	return err
}

// RefuseWeldedAwayComponent refuses the one class of weld collapse no bound can
// answer for: a connected component of the stitched surface EVERY facet of
// which the weld collapses. That component is a shell — a lump of the result,
// or a cavity inside one — and dropping all of it removes the lump from the
// body entirely: its volume, its place in Lumps(), its reach in the bounds box.
// The closure audit does not catch it, because the components that remain still
// close; nothing else in the pipeline would report it either. Every other
// collapse is an edge contraction WITHIN a component that survives, and the
// rounding bounds account for it: the swept volume is charged against the
// pre-round surface (preArea) and the missing facet area against dropArea, both
// of which count the dropped facets.
func RefuseWeldedAwayComponent(ctx context.Context, tris [][3]int, dropped []bool) error {
	comp := make([]int, len(tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := FacetAdjacencyContext(ctx, tris)
	if err != nil {
		return err
	}
	work := 0
	for i := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		if comp[i] != -1 {
			continue
		}
		queue := []int{i}
		comp[i] = i
		alive := false
		for len(queue) > 0 {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			f := queue[0]
			queue = queue[1:]
			alive = alive || !dropped[f]
			for _, nb := range adj[f] {
				if comp[nb] != -1 {
					continue
				}
				comp[nb] = i
				queue = append(queue, nb)
			}
		}
		if !alive {
			return fmt.Errorf(`%w: a whole component of the result welds away under float rounding — the lump would vanish from the body, and no volume, lump count or bound could then be trusted; this evaluator refuses rather than drop it`, decaderr.ErrUnsupported)
		}
	}
	return nil
}
