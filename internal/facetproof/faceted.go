package facetproof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// KeepPlacedEmbedded keeps a placed faceted mesh embedded through float
// rounding of its exact motion (agent-docs/evaluator-design.md §9): the exact image of
// an embedded mesh under the motion is embedded, and each held vertex starts
// at the nearest float to its exact image. Every vertex that differs
// from its exact image is checked and, where its facets fold, may move to a
// corner of that image's float box (meshbool.EnforceHeldEmbedding); a fold
// remaining after the bounded search is refused. When ctx carries a gap
// tolerance (internal/gaptol), a fold the float box cannot clear is also tried
// at positions up to that far from the exact image. It returns, per vertex, the
// proven 3D displacement of each vertex the check moved from its exact image:
// zero for a vertex it left alone, and nil when it moved none. The caller
// charges alongside that vertex's own rounding allowance, since a corner can
// sit farther from the exact image than nearest rounding did.
func KeepPlacedEmbedded(ctx context.Context, exact []proof.Xpt, held []r3.Vec, tris [][3]int) ([]float64, error) {
	charge, _, err := KeepPlacedEmbeddedWithSummary(ctx, exact, held, tris)
	return charge, err
}

// KeepPlacedEmbeddedWithSummary is KeepPlacedEmbedded with actual-use evidence
// for final accepted positive-reach positions, independent of rounding charges.
func KeepPlacedEmbeddedWithSummary(ctx context.Context, exact []proof.Xpt, held []r3.Vec,
	tris [][3]int) ([]float64, *meshbool.EmbeddingReach, error) {
	return keepPlacedEmbeddedWithAxes(ctx, exact, held, tris, [3]r3.Vec{})
}

func keepPlacedEmbeddedWithAxes(ctx context.Context, exact []proof.Xpt, held []r3.Vec,
	tris [][3]int, axes [3]r3.Vec) ([]float64, *meshbool.EmbeddingReach, error) {
	h := meshbool.HeldRounding{
		Verts:    held,
		Tris:     tris,
		Exact:    exact,
		Moved:    make([]bool, len(held)),
		Movable:  make([]bool, len(held)),
		Reach:    gaptol.Reach(ctx),
		ScanAxes: axes,
	}
	first := make([]r3.Vec, len(held))
	copy(first, held)
	for i := range held {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, err
			}
		}
		if !proof.XptDenotes(exact[i], held[i]) {
			h.Moved[i] = true
			h.Movable[i] = true
		}
	}
	n, summary, err := meshbool.EnforceHeldEmbeddingWithSummary(ctx, h)
	if err != nil {
		return nil, nil, err
	}
	if n == 0 {
		return nil, nil, nil
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
	return out, summary, nil
}

// MeshAreaUpper is a proven upper bound on the held mesh's total facet area:
// the float sum padded by the PROVEN naive-summation bound of the very loop
// that computed it (internal/proofbound/bounds.go, proofbound.SumSlop) — not a
// fixed fraction, which no proof backs at any facet count — and by each
// facet's own evaluation charge, proofbound.FacetAreaTermSlop, which a sliver
// facet's cancelling cross product needs at the scale of its edge products.
func MeshAreaUpper(verts []r3.Vec, tris [][3]int) float64 {
	total, termSlop := 0.0, 0.0
	for _, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		total += b.Sub(a).Cross(c.Sub(a)).Len() / 2
		termSlop = proofbound.AbsSumUpper(termSlop, proofbound.FacetAreaTermSlop(a, b, c))
	}
	return total + proofbound.SumSlop(len(tris), total) + termSlop + 1e-300
}

// FacetedAreaGeom is a face's geometric area allowance: the smaller of two
// proven upper bounds on how far its held facets' area can differ from its
// true piece's, the face's own displacement delta times its upper perimeter
// (the product rounded outward, so a positive product never underflows to
// zero) and facetAllow, the sum of each facet's perturbation at its own δ(t).
func FacetedAreaGeom(delta, perimeterUpper, facetAllow float64) float64 {
	return math.Min(proofbound.ProductUpper(delta, perimeterUpper), facetAllow)
}

// FacetedExtremeError is agent-docs/faceted-vertex-bounds-design.md §4.2's box
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
	// Held coordinates and bounds are binary floats. Dyadic keeps their
	// extrema exact without allocating a rational at every vertex.
	var worst proof.Dyadic
	widen := func(gap proof.Dyadic) {
		if proof.DyCmp(gap, worst) > 0 {
			worst = gap
		}
	}
	for axis := range 3 {
		m := [2]proof.Dyadic{
			proof.MustDyOf(meshbool.CoordOf(lo, axis)), proof.MustDyOf(meshbool.CoordOf(hi, axis)),
		}
		// reach[side] is the extreme a true point can reach past the held
		// one; held[side] the extreme some true point is proven to attain.
		var reach, held [2]proof.Dyadic
		seen := false
		for i, v := range verts {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			c := proof.MustDyOf(meshbool.CoordOf(v, axis))
			b := proof.MustDyOf(B[i])
			own := proof.MustDyOf(beta[i])
			if out := proof.DySubScalar(c, b); !seen || proof.DyCmp(out, reach[0]) < 0 {
				reach[0] = out
			}
			if out := proof.DyAdd(c, b); !seen || proof.DyCmp(out, reach[1]) > 0 {
				reach[1] = out
			}
			if in := proof.DyAdd(c, own); !seen || proof.DyCmp(in, held[0]) < 0 {
				held[0] = in
			}
			if in := proof.DySubScalar(c, own); !seen || proof.DyCmp(in, held[1]) > 0 {
				held[1] = in
			}
			seen = true
		}
		widen(proof.DySubScalar(m[0], reach[0]))
		widen(proof.DySubScalar(held[0], m[0]))
		widen(proof.DySubScalar(reach[1], m[1]))
		widen(proof.DySubScalar(m[1], held[1]))
	}
	if worst.Sign() == 0 {
		return 0, nil
	}
	w, _ := worst.Rat().Float64()
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
	// Det is each facet's exact a·(b×c) over XVerts when every facet's term
	// was a Dyadic; nil when any facet took the big.Rat fallback.
	Det []proof.Dyadic
}

// auditParallelMin is the facet count from which AuditFacetedMesh lifts its
// vertices and computes its facet determinants on several goroutines.
const auditParallelMin = 2048

// AuditFacetedMesh proves the held mesh closed, splits it into shells and
// computes each shell's exact volume, containment and orientation. Large
// meshes lift their vertices and facet determinants on up to
// orderedwork.MaxEachWorkers goroutines; every fold, error and context poll
// stays on the calling goroutine in facet order, so the result and the first
// error are those of the single-goroutine loop.
func AuditFacetedMesh(ctx context.Context, verts []r3.Vec, tris [][3]int) (*MeshAudit, error) {
	return auditFacetedMesh(ctx, verts, tris, orderedwork.EachWorkers(len(tris), auditParallelMin))
}

// trisIndexVerts reports whether every facet corner names a vertex.
func trisIndexVerts(tris [][3]int, n int) bool {
	for _, t := range tris {
		for _, c := range t {
			if c < 0 || c >= n {
				return false
			}
		}
	}
	return true
}

func auditFacetedMesh(ctx context.Context, verts []r3.Vec, tris [][3]int, workers int) (*MeshAudit, error) {
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: the boolean result holds no boundary`, decaderr.ErrBooleanFailed)
	}

	neighbors, err := closedFacetEdges(ctx, tris)
	if err != nil {
		return nil, err
	}

	// A lift or determinant that panics on a bad input must do so where the
	// single-goroutine loop does, so such meshes take that path.
	if workers >= 2 && (!proofbound.AllFinite(verts) || !trisIndexVerts(tris, len(verts))) {
		workers = 1
	}
	audit := &MeshAudit{XVerts: make([]proof.Xpt, len(verts))}
	if workers < 2 {
		for i, v := range verts {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			audit.XVerts[i] = proof.XptOf(v)
		}
	} else {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if _, err := orderedwork.Each(len(verts), workers, 512, ctx.Err, func(i int) error {
			audit.XVerts[i] = proof.XptOf(verts[i])
			return nil
		}); err != nil {
			return nil, err
		}
	}

	audit.Comp = make([]int, len(tris))
	for i := range audit.Comp {
		audit.Comp[i] = -1
	}
	Adj, err := closedFacetAdjacency(ctx, tris, neighbors)
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

	// preDet[fi] is facet fi's determinant term, computed ahead
	// of the component loop that consumes them in its own order.
	var preDet []proof.Dyadic
	if workers >= 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		preDet = make([]proof.Dyadic, len(tris))
		if _, err := orderedwork.Each(len(tris), workers, 256, ctx.Err, func(fi int) error {
			t := tris[fi]
			preDet[fi] = floatFacetDeterminant(verts, t)
			return nil
		}); err != nil {
			return nil, err
		}
	}

	sixth := big.NewRat(1, 6)
	det := make([]proof.Dyadic, len(tris))
	audit.CompVol = make([]*big.Rat, len(audit.Members))
	for ci, list := range audit.Members {
		// Every term is a Dyadic (power-of-two denominators), so the
		// sum accumulates without a GCD and converts once; the value is the
		// same rational the big.Rat sum produced.
		var total proof.DyAcc
		for i, fi := range list {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			var term proof.Dyadic
			if preDet != nil {
				term = preDet[fi]
			} else {
				term = floatFacetDeterminant(verts, tris[fi])
			}
			det[fi] = term
			total.Add(term)
		}
		v := total.Rat()
		audit.CompVol[ci] = v.Mul(v, sixth)
		if audit.CompVol[ci].Sign() == 0 {
			return nil, fmt.Errorf(`%w: a result shell encloses no volume`, decaderr.ErrBooleanFailed)
		}
	}

	audit.Det = det
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

// floatFacetDeterminant reads the exact scalar triple product directly from
// finite held coordinates. Their dyadic denominators need no homogeneous
// alignment before the cross product. The audit keeps its homogeneous lifts
// for containment and later moment consumers.
func floatFacetDeterminant(verts []r3.Vec, t [3]int) proof.Dyadic {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	return proof.FloatDeterminant(a, b, c)
}
