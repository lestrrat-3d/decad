package facetproof

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// FacetDeltas computes each facet's largest vertex displacement.
func FacetDeltas(tris [][3]int, vertexBound []float64) []float64 {
	out := make([]float64, len(tris))
	for i, t := range tris {
		out[i] = max(vertexBound[t[0]], vertexBound[t[1]], vertexBound[t[2]])
	}
	return out
}

// SourcePatches groups edge-connected facets that name the same source group.
// The traversal keeps facet and adjacency order because they fix face order.
func SourcePatches(budget *proofbound.WorkBudget, facetCount int, source []int, adj [][]int) ([]int, error) {
	patch := make([]int, facetCount)
	for i := range patch {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		patch[i] = -1
	}
	nPatch := 0
	for i := range facetCount {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if patch[i] != -1 {
			continue
		}
		id := nPatch
		nPatch++
		patch[i] = id
		queue := []int{i}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			f := queue[0]
			queue = queue[1:]
			for _, nb := range adj[f] {
				if patch[nb] != -1 || source[nb] != source[f] {
					continue
				}
				patch[nb] = id
				queue = append(queue, nb)
			}
		}
	}
	return patch, nil
}

// VoidParents finds the innermost positive shell containing each negative
// shell. Positive shells keep parent -1.
func VoidParents(budget *proofbound.WorkBudget, compVol []*big.Rat, contains [][]bool) ([]int, error) {
	parents := make([]int, len(compVol))
	for ci := range compVol {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		parents[ci] = -1
		if compVol[ci].Sign() > 0 {
			continue
		}
		for outer := range compVol {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if outer == ci || compVol[outer].Sign() <= 0 || !contains[outer][ci] {
				continue
			}
			if parents[ci] == -1 || contains[parents[ci]][outer] {
				parents[ci] = outer
			}
		}
		if parents[ci] == -1 {
			return nil, fmt.Errorf(`%w: a void shell has no containing shell`, decaderr.ErrBooleanFailed)
		}
	}
	return parents, nil
}

// FaceIndices maps facet faces to the built body's face order.
func FaceIndices[F comparable](ctx context.Context, faces, facetFace []F) ([]int, error) {
	budget := proofbound.NewWorkBudget(ctx)
	flat := make(map[F]int, len(faces))
	for i, f := range faces {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		flat[f] = i
	}
	out := make([]int, len(facetFace))
	for i, f := range facetFace {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		idx, ok := flat[f]
		if !ok {
			return nil, fmt.Errorf(`%w: a facet maps to a face absent from the built body`, decaderr.ErrBooleanFailed)
		}
		out[i] = idx
	}
	return out, budget.Err()
}

// RestateSources validates a held faceted payload and maps each facet's source
// index back to its live face, retaining facet order.
func RestateSources[F any](ctx context.Context, faces []F, faceOf []int,
	vertexCount, boundCount, triangleCount int) ([]F, error) {
	src := make([]F, triangleCount)
	budget := proofbound.NewWorkBudget(ctx)
	if boundCount != vertexCount || len(faceOf) != triangleCount {
		return nil, fmt.Errorf(`%w: a faceted payload's per-vertex or per-facet record does not match its mesh`, decaderr.ErrBooleanFailed)
	}
	for i, fi := range faceOf {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if fi < 0 || fi >= len(faces) {
			return nil, fmt.Errorf(`%w: a facet maps to no face`, decaderr.ErrBooleanFailed)
		}
		src[i] = faces[fi]
	}
	return src, nil
}

// FaceProofs accumulates each face's largest facet bound and summed facet area
// allowance in facet order.
func FaceProofs[F comparable](budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	faces []F, deltas []float64) (map[F]float64, map[F]float64, error) {
	faceDelta := map[F]float64{}
	faceFacetArea := map[F]float64{}
	for i, f := range faces {
		if err := budget.Step(); err != nil {
			return nil, nil, err
		}
		faceDelta[f] = max(faceDelta[f], deltas[i])
		t := tris[i]
		faceFacetArea[f] = proofbound.AbsSumUpper(faceFacetArea[f],
			proofbound.PerturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], deltas[i]))
	}
	return faceDelta, faceFacetArea, nil
}

// VertexFacetDeltas gives every vertex its largest incident facet bound.
func VertexFacetDeltas(n int, tris [][3]int, deltas []float64) []float64 {
	out := make([]float64, n)
	for i, t := range tris {
		for _, v := range t {
			out[v] = max(out[v], deltas[i])
		}
	}
	return out
}

// Volume integrates a stitched, oriented mesh in exact rational arithmetic,
// then composes its symmetric-difference and final float rounding bounds.
func Volume(ctx context.Context, xverts []proof.Xpt, tris [][3]int,
	volSymDiff float64) (float64, float64, *big.Rat, error) {
	total := new(big.Rat)
	for i, t := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, nil, err
			}
		}
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		total.Add(total, proof.XdotRat(a, proof.Xcross(b, c)))
	}
	volRat := new(big.Rat).Mul(total, big.NewRat(1, 6))
	if volRat.Sign() <= 0 {
		return 0, 0, nil, fmt.Errorf(`%w: the boolean result encloses no volume`, decaderr.ErrBooleanFailed)
	}
	volF, _ := volRat.Float64()
	bound := proofbound.AbsSumUpper(volSymDiff, proofbound.RatAbsDiff(volRat, volF))
	return volF, bound, volRat, nil
}

// FirstMoments integrates the held mesh's three exact first moments in facet
// order. The caller divides by 24 and the exact volume.
func FirstMoments(budget *proofbound.WorkBudget, xverts []proof.Xpt, tris [][3]int) ([3]*big.Rat, error) {
	var moments = [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, t := range tris {
		if err := budget.Step(); err != nil {
			return [3]*big.Rat{}, err
		}
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		det := proof.XdotRat(a, proof.Xcross(b, c))
		ax, ay, az := proof.XhpRat(proof.Xhp(a))
		bx, by, bz := proof.XhpRat(proof.Xhp(b))
		cx, cy, cz := proof.XhpRat(proof.Xhp(c))
		moments[0].Add(moments[0], new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(ax, bx), cx)))
		moments[1].Add(moments[1], new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(ay, by), cy)))
		moments[2].Add(moments[2], new(big.Rat).Mul(det, new(big.Rat).Add(new(big.Rat).Add(az, bz), cz)))
	}
	return moments, nil
}
