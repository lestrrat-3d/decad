package facetproof

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SimilarMesh is a positive uniform image of one certified faceted mesh.
type SimilarMesh struct {
	PlacedMesh
	AreaSlack float64
	Round     float64
}

// ScaleMesh maps every held vertex about anchor. The exact similarity
// preserves the source mesh's embedding and scales its boundary, area and
// occupied-volume allowances by factor, factor² and factor³. The rounded
// image receives a separate embedding audit and swept boundary allowance.
func ScaleMesh(ctx context.Context, budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int, vertexBound []float64,
	volSymDiff, areaSlack float64, anchor r3.Vec, factor float64) (SimilarMesh, error) {
	if len(verts) != len(vertexBound) || factor <= 0 || proofbound.IsNonFinite(factor) ||
		!proofbound.FiniteVec(anchor) || volSymDiff < 0 || areaSlack < 0 ||
		proofbound.IsNonFinite(volSymDiff) || proofbound.IsNonFinite(areaSlack) {
		return SimilarMesh{}, fmt.Errorf("%w: the similarity input has no finite proof", decaderr.ErrUnsupported)
	}
	next := SimilarMesh{PlacedMesh: PlacedMesh{
		Verts: make([]r3.Vec, len(verts)), Tris: tris,
		VertexBound: make([]float64, len(vertexBound)),
	}}
	h := meshbool.HeldRounding{
		Verts: next.Verts, Tris: tris, Exact: make([]proof.Xpt, len(verts)),
		Moved: make([]bool, len(verts)), Movable: make([]bool, len(verts)),
		Reach: gaptol.Reach(ctx),
	}
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return SimilarMesh{}, err
		}
		h.Exact[i] = meshbool.ExactSimilarityImage(anchor, v, factor)
		next.Verts[i] = anchor.Add(v.Sub(anchor).Scale(factor))
		if !proofbound.FiniteVec(next.Verts[i]) {
			return SimilarMesh{}, fmt.Errorf("%w: a scaled vertex is not representable", decaderr.ErrUnsupported)
		}
		if meshbool.CoordDistance(h.Exact[i], next.Verts[i]).Sign() != 0 {
			h.Moved[i], h.Movable[i] = true, true
		}
	}
	if _, err := meshbool.EnforceHeldEmbedding(ctx, h); err != nil {
		return SimilarMesh{}, err
	}
	for i, beta := range vertexBound {
		if err := budget.Step(); err != nil {
			return SimilarMesh{}, err
		}
		gap := meshbool.CoordDistance(h.Exact[i], next.Verts[i])
		w, _ := gap.Float64()
		round := 0.0
		if gap.Sign() > 0 {
			round = proofbound.Radius3D(proofbound.ProvenUpRound(w))
		}
		next.Round = math.Max(next.Round, round)
		next.VertexBound[i] = proofbound.AbsSumUpper(proofbound.ProductUpper(factor, beta), round)
		if proofbound.IsNonFinite(next.VertexBound[i]) {
			return SimilarMesh{}, fmt.Errorf("%w: a scaled vertex has no finite bound", decaderr.ErrUnsupported)
		}
	}
	next.MeshBound = FacetBoundMax(tris, next.VertexBound)
	areaUpper, err := proofbound.PerturbedAreaUpperWithBudget(budget, next.Verts, tris, next.Round)
	if err != nil {
		return SimilarMesh{}, err
	}
	square := proofbound.ProductUpper(factor, factor)
	cube := proofbound.ProductUpper(square, factor)
	next.VolSymDiff = proofbound.AbsSumUpper(
		proofbound.ProductUpper(cube, volSymDiff),
		proofbound.SweptVolumeAllow(next.Round, areaUpper),
	)
	areaRound := 0.0
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return SimilarMesh{}, err
		}
		a, b, c := next.Verts[tri[0]], next.Verts[tri[1]], next.Verts[tri[2]]
		u, v := proofbound.HeldDelta(b, a), proofbound.HeldDelta(c, a)
		term := proofbound.AbsSumUpper(
			proofbound.ProductUpper(next.Round,
				proofbound.AbsSumUpper(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v))),
			proofbound.ProductUpper(2, proofbound.ProductUpper(next.Round, next.Round)),
		)
		areaRound = proofbound.AbsSumUpper(areaRound, term)
	}
	next.AreaSlack = proofbound.AbsSumUpper(proofbound.ProductUpper(square, areaSlack), areaRound)
	if proofbound.IsNonFinite(next.MeshBound) || proofbound.IsNonFinite(next.VolSymDiff) ||
		proofbound.IsNonFinite(next.AreaSlack) {
		return SimilarMesh{}, fmt.Errorf("%w: the scaled mesh has no finite proof", decaderr.ErrUnsupported)
	}
	return next, nil
}
