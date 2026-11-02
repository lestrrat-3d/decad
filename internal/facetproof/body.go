package facetproof

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
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
	if budget.Replay {
		return sourcePatchesReplay(budget, facetCount, source, adj)
	}
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

// VolumeFromAudit is Volume's result read from the audit's exact
// component volumes. The facets partition into the components, and an exact
// rational sum is independent of order and grouping, so the sum of the
// component volumes is the rational Volume integrates over every facet, bit
// for bit; the float value and bound follow from that same rational.
func VolumeFromAudit(audit *MeshAudit, volSymDiff float64) (float64, float64, *big.Rat, error) {
	volRat := new(big.Rat)
	for _, v := range audit.CompVol {
		volRat.Add(volRat, v)
	}
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
	// Power-of-two weights let every facet term share one growing dyadic
	// denominator. Normalize once per axis rather than once per facet.
	dyadic := true
	for _, vertex := range xverts {
		if vertex.W.Sign() <= 0 || vertex.W.BitLen() != int(vertex.W.TrailingZeroBits())+1 {
			dyadic = false
			break
		}
	}
	var numerators [3]big.Int
	scaleBits := 0
	for _, t := range tris {
		if err := budget.Step(); err != nil {
			return [3]*big.Rat{}, err
		}
		a, b, c := xverts[t[0]], xverts[t[1]], xverts[t[2]]
		cross := proof.Xcross(b, c)
		detNum := proof.XdotNum(a, cross)
		if detNum.Sign() == 0 {
			continue
		}
		// det uses a.W*cross.W; each coordinate sum uses a.W*b.W*c.W.
		// Multiply integer numerators before normalizing one rational per axis.
		bc := new(big.Int).Mul(b.W, c.W)
		ac := new(big.Int).Mul(a.W, c.W)
		ab := new(big.Int).Mul(a.W, b.W)
		den := new(big.Int).Mul(a.W, cross.W)
		den.Mul(den, new(big.Int).Mul(a.W, bc))
		denBits := den.BitLen() - 1
		if dyadic && denBits > scaleBits {
			shift := uint(denBits - scaleBits)
			for axis := range numerators {
				numerators[axis].Lsh(&numerators[axis], shift)
			}
			scaleBits = denBits
		}
		add := func(axis int, ax, bx, cx *big.Int) {
			var sum, term, numerator big.Int
			sum.Mul(ax, bc)
			sum.Add(&sum, term.Mul(bx, ac))
			sum.Add(&sum, term.Mul(cx, ab))
			numerator.Mul(detNum, &sum)
			if dyadic {
				numerator.Lsh(&numerator, uint(scaleBits-denBits))
				numerators[axis].Add(&numerators[axis], &numerator)
				return
			}
			moments[axis].Add(moments[axis], new(big.Rat).SetFrac(&numerator, den))
		}
		add(0, a.X, b.X, c.X)
		add(1, a.Y, b.Y, c.Y)
		add(2, a.Z, b.Z, c.Z)
	}
	if dyadic {
		den := new(big.Int).Lsh(big.NewInt(1), uint(scaleBits))
		for axis := range moments {
			moments[axis].SetFrac(&numerators[axis], den)
		}
	}
	return moments, nil
}

// momentsParallelMin is the facet count from which FirstMomentsFromAudit sums
// its blocks on several goroutines, and momentsBlock the facets per block.
const (
	momentsParallelMin = 2048
	momentsBlock       = 512
)

// FirstMomentsFromAudit is FirstMoments over audit.XVerts that reuses the
// audit's per-facet determinants: each facet adds det·(a+b+c) per axis, summed
// as Dyadics and converted once. It returns FirstMoments' rational exactly. It
// falls back to FirstMoments when the audit holds no determinants or a vertex
// has a non-dyadic denominator. Large meshes sum blocks of facets on several
// goroutines; the exact sum does not depend on the grouping, and the budget is
// charged one step per facet on the calling goroutine afterwards.
// Nonempty work first polls the shared budget. An exhausted wall budget reports
// the first attempted facet through Step, as the serial loop does, before any
// vertex lifting or parallel block computation starts.
func FirstMomentsFromAudit(budget *proofbound.WorkBudget, audit *MeshAudit, tris [][3]int) ([3]*big.Rat, error) {
	if budget.Replay {
		return [3]*big.Rat{}, fmt.Errorf("%w: replay moments require an explicit context", decaderr.ErrUnsupported)
	}
	return firstMomentsFromAudit(budget, audit, tris, orderedwork.EachWorkers(len(tris), momentsParallelMin))
}

func firstMomentsFromAudit(budget *proofbound.WorkBudget, audit *MeshAudit, tris [][3]int,
	workers int) ([3]*big.Rat, error) {
	if len(tris) > 0 {
		if err := budget.Err(); err != nil {
			if errors.Is(err, proofbound.ErrWallWorkBudget) {
				if stepErr := budget.Step(); stepErr != nil {
					return [3]*big.Rat{}, stepErr
				}
			}
			return [3]*big.Rat{}, err
		}
	}
	if audit.Det == nil {
		return FirstMoments(budget, audit.XVerts, tris)
	}
	lifted := make([]proof.DyV3, len(audit.XVerts))
	for i, p := range audit.XVerts {
		v, ok := proof.XptDyV3(p)
		if !ok {
			return FirstMoments(budget, audit.XVerts, tris)
		}
		lifted[i] = v
	}
	if workers >= 2 {
		return firstMomentsParallel(budget, audit.Det, lifted, tris, workers)
	}
	var acc [3]proof.DyAcc
	for fi, t := range tris {
		if err := budget.Step(); err != nil {
			return [3]*big.Rat{}, err
		}
		addFacetMoment(&acc, audit.Det[fi], lifted, t)
	}
	return [3]*big.Rat{acc[0].Rat(), acc[1].Rat(), acc[2].Rat()}, nil
}

// addFacetMoment adds det·(a+b+c) to each axis of acc; a zero det adds nothing.
func addFacetMoment(acc *[3]proof.DyAcc, d proof.Dyadic, lifted []proof.DyV3, t [3]int) {
	if d.IsZero() {
		return
	}
	a, b, c := lifted[t[0]], lifted[t[1]], lifted[t[2]]
	for axis := range 3 {
		s := proof.DyAdd(proof.DyAdd(a[axis], b[axis]), c[axis])
		acc[axis].AddProduct(d, s)
	}
}

// firstMomentsParallel sums momentsBlock facets per block into one exact
// accumulator per axis, then adds the block sums in block order.
func firstMomentsParallel(budget *proofbound.WorkBudget, det []proof.Dyadic, lifted []proof.DyV3,
	tris [][3]int, workers int) ([3]*big.Rat, error) {
	blocks := (len(tris) + momentsBlock - 1) / momentsBlock
	parts := make([][3]proof.DyAcc, blocks)
	if _, err := orderedwork.Each(blocks, workers, 1, budget.Err, func(b int) error {
		for fi := b * momentsBlock; fi < min((b+1)*momentsBlock, len(tris)); fi++ {
			addFacetMoment(&parts[b], det[fi], lifted, tris[fi])
		}
		return nil
	}); err != nil {
		return [3]*big.Rat{}, err
	}
	for range tris {
		if err := budget.Step(); err != nil {
			return [3]*big.Rat{}, err
		}
	}
	var acc [3]proof.DyAcc
	for b := range parts {
		for axis := range acc {
			acc[axis].Add(parts[b][axis].Dyadic())
		}
	}
	return [3]*big.Rat{acc[0].Rat(), acc[1].Rat(), acc[2].Rat()}, nil
}
