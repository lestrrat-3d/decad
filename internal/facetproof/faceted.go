package facetproof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// KeepPlacedEmbedded keeps a placed faceted mesh embedded through the float
// evaluation of its motion (docs/evaluator-design.md §9): the exact image of
// an embedded mesh under the motion is embedded, and each held vertex is the
// motion's float evaluation of its exact image. Every vertex that differs
// from its exact image is checked and, where its facets fold, may move to a
// corner of that image's float box (meshbool.EnforceHeldEmbedding); a fold no
// corner clears is refused. It returns, per vertex, the proven 3D
// displacement of each vertex the check moved from its exact image — zero for
// a vertex it left alone, and nil when it moved none — which the caller
// charges alongside that vertex's own rounding allowance, since a corner can
// sit farther from the exact image than the float evaluation did.
func KeepPlacedEmbedded(ctx context.Context, src, held []r3.Vec, tris [][3]int, delta r3.Transform) ([]float64, error) {
	b, t := delta.Basis(), delta.Translation()
	h := meshbool.HeldRounding{
		Verts:   held,
		Tris:    tris,
		Exact:   make([]proof.Xpt, len(held)),
		Moved:   make([]bool, len(held)),
		Movable: make([]bool, len(held)),
	}
	first := make([]r3.Vec, len(held))
	copy(first, held)
	for i, p := range src {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		x := meshbool.ExactRigidImage(b, t, p)
		h.Exact[i] = x
		if meshbool.CoordDistance(x, held[i]).Sign() != 0 {
			h.Moved[i] = true
			h.Movable[i] = true
		}
	}
	n, err := meshbool.EnforceHeldEmbedding(ctx, h)
	if err != nil {
		return nil, err
	}
	if n == 0 {
		return nil, nil
	}
	out := make([]float64, len(held))
	for i := range held {
		if held[i] == first[i] {
			continue
		}
		if d := meshbool.CoordDistance(h.Exact[i], held[i]); d.Sign() > 0 {
			w, _ := d.Float64()
			out[i] = proofbound.Radius3D(proofbound.ProvenUpRound(w))
		}
	}
	return out, nil
}

// MeshAreaUpper is a proven upper bound on the held mesh's total facet area:
// the float sum padded by the PROVEN naive-summation bound of the very loop
// that computed it (internal/proofbound/bounds.go, proofbound.SumSlop) — not a fixed fraction, which no proof
// backs at any facet count.
func MeshAreaUpper(verts []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		total += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	return total + proofbound.SumSlop(len(tris), total) + 1e-300
}

// FacetedAreaGeom is a face's geometric area allowance: the smaller of two
// proven upper bounds on how far its held facets' area can differ from its
// true piece's, the face's own displacement delta times its upper perimeter
// (the product rounded outward, so a positive product never underflows to
// zero) and facetAllow, the sum of each facet's perturbation at its own δ(t).
func FacetedAreaGeom(delta, perimeterUpper, facetAllow float64) float64 {
	return math.Min(proofbound.ProductUpper(delta, perimeterUpper), facetAllow)
}

// FacetedExtremeError is docs/faceted-vertex-bounds-design.md §4.2's box
// reading: the largest per-coordinate error of the six held extremes lo, hi.
// B is each vertex's largest facet bound δ(t) over the facets touching it and
// beta its own β. Every true boundary point lies within δ(t) of some held
// facet, so the true maximum on an axis is at most max_v (v + B(v)); and the
// held vertex v has a true point within beta(v), so the true maximum is at
// least max_v (v − beta(v)). The error of the held maximum m is the larger of
// the two gaps, and the minimum is the mirror image. Each gap is formed in
// exact arithmetic and rounded up; only a vertex whose own bound reaches past
// the extreme can widen it beyond the extreme vertex's own figure.
func FacetedExtremeError(budget *proofbound.WorkBudget, verts []r3.Vec, beta, B []float64, lo, hi r3.Vec) (float64, error) {
	worst := new(big.Rat)
	widen := func(gap *big.Rat) {
		if gap.Cmp(worst) > 0 {
			worst = gap
		}
	}
	for axis := range 3 {
		m := [2]*big.Rat{new(big.Rat).SetFloat64(meshbool.CoordOf(lo, axis)), new(big.Rat).SetFloat64(meshbool.CoordOf(hi, axis))}
		// reach[side] is the extreme a true point can reach past the held
		// one; held[side] the extreme some true point is proven to attain.
		var reach, held [2]*big.Rat
		for i, v := range verts {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			c := new(big.Rat).SetFloat64(meshbool.CoordOf(v, axis))
			b := new(big.Rat).SetFloat64(B[i])
			own := new(big.Rat).SetFloat64(beta[i])
			if out := new(big.Rat).Sub(c, b); reach[0] == nil || out.Cmp(reach[0]) < 0 {
				reach[0] = out
			}
			if out := new(big.Rat).Add(c, b); reach[1] == nil || out.Cmp(reach[1]) > 0 {
				reach[1] = out
			}
			if in := new(big.Rat).Add(c, own); held[0] == nil || in.Cmp(held[0]) < 0 {
				held[0] = in
			}
			if in := new(big.Rat).Sub(c, own); held[1] == nil || in.Cmp(held[1]) > 0 {
				held[1] = in
			}
		}
		widen(new(big.Rat).Sub(m[0], reach[0]))
		widen(new(big.Rat).Sub(held[0], m[0]))
		widen(new(big.Rat).Sub(reach[1], m[1]))
		widen(new(big.Rat).Sub(m[1], held[1]))
	}
	if worst.Sign() == 0 {
		return 0, nil
	}
	w, _ := worst.Float64()
	return proofbound.ProvenUpRound(w), nil
}

// MeshAudit is the shared, geometry-only component and shell proof.
// It allocates no topology objects and holds no document reference, so the
// read-only evaluator can run the same invariant checks as body construction.
type MeshAudit struct {
	XVerts   []proof.Xpt
	Comp     []int
	Adj      [][]int
	Members  [][]int
	CompVol  []*big.Rat
	Contains [][]bool
}

func AuditFacetedMesh(ctx context.Context, verts []r3.Vec, tris [][3]int) (*MeshAudit, error) {
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: the boolean result holds no boundary`, decaderr.ErrBooleanFailed)
	}

	directed := map[[2]int]int{}
	for i, t := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			directed[[2]int{t[k], t[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the held boundary does not close`, decaderr.ErrBooleanFailed)
		}
	}

	audit := &MeshAudit{XVerts: make([]proof.Xpt, len(verts))}
	for i, v := range verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		audit.XVerts[i] = proof.XptOf(v)
	}

	audit.Comp = make([]int, len(tris))
	for i := range audit.Comp {
		audit.Comp[i] = -1
	}
	Adj, err := meshbool.FacetAdjacencyContext(ctx, tris)
	if err != nil {
		return nil, err
	}
	audit.Adj = Adj
	work := 0
	for i := range tris {
		if audit.Comp[i] != -1 {
			continue
		}
		id := len(audit.Members)
		queue := []int{i}
		audit.Comp[i] = id
		var list []int
		for len(queue) > 0 {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			f := queue[0]
			queue = queue[1:]
			list = append(list, f)
			for _, nb := range audit.Adj[f] {
				if audit.Comp[nb] == -1 {
					audit.Comp[nb] = id
					queue = append(queue, nb)
				}
			}
		}
		audit.Members = append(audit.Members, list)
	}

	sixth := big.NewRat(1, 6)
	audit.CompVol = make([]*big.Rat, len(audit.Members))
	for ci, list := range audit.Members {
		v := new(big.Rat)
		for i, fi := range list {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			t := tris[fi]
			v.Add(v, proof.XdotRat(audit.XVerts[t[0]], meshbool.Xcross(audit.XVerts[t[1]], audit.XVerts[t[2]])))
		}
		audit.CompVol[ci] = v.Mul(v, sixth)
		if audit.CompVol[ci].Sign() == 0 {
			return nil, fmt.Errorf(`%w: a result shell encloses no volume`, decaderr.ErrBooleanFailed)
		}
	}

	audit.Contains = make([][]bool, len(audit.Members))
	for i := range audit.Members {
		audit.Contains[i] = make([]bool, len(audit.Members))
	}
	for inner := range audit.Members {
		t := tris[audit.Members[inner][0]]
		probe := meshbool.XCentroid(audit.XVerts[t[0]], audit.XVerts[t[1]], audit.XVerts[t[2]])
		for outer := range audit.Members {
			if outer == inner {
				continue
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			// The raw-buffer entry point on purpose: these are the RESULT
			// mesh's own vertices, freshly stitched, and no operand's prepared
			// projection cache describes them. Reusing one here would project
			// the wrong mesh.
			inside, onBoundary, err := meshbool.MeshParityContext(ctx, probe, verts, tris, audit.Members[outer])
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: two result shells touch`, decaderr.ErrBooleanFailed)
			}
			audit.Contains[outer][inner] = inside
		}
	}

	// Closed oriented material shells alternate with containment depth:
	// positive outer shell, negative void, positive island, and so on. Every
	// pair of containers must also be nested; intersecting shell relations are
	// impossible after stitching and therefore an evaluator failure.
	budget := proofbound.NewWorkBudget(ctx)
	for inner := range audit.Members {
		var containers []int
		for outer := range audit.Members {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if audit.Contains[outer][inner] {
				containers = append(containers, outer)
			}
		}
		for i := range containers {
			for j := i + 1; j < len(containers); j++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				a, b := containers[i], containers[j]
				if !audit.Contains[a][b] && !audit.Contains[b][a] {
					return nil, fmt.Errorf(`%w: result shells have an impossible containment relation`, decaderr.ErrBooleanFailed)
				}
			}
		}
		want := 1
		if len(containers)%2 == 1 {
			want = -1
		}
		if audit.CompVol[inner].Sign() != want {
			return nil, fmt.Errorf(`%w: a result shell has inconsistent orientation`, decaderr.ErrBooleanFailed)
		}
	}
	return audit, nil
}
