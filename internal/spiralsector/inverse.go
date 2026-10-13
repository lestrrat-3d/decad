package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// InverseHeldResult maps a copy's stitched held result back to the first
// tooth's frame. Inverse placement is used only for comparing output chains
// with their unplaced source curves. The certified output β is stretched and
// charged for inverse float rounding, the residual tooth transform, and the
// blank's slight radial scaling under the stored axis-rotation matrix.
func InverseHeldResult(ctx context.Context, blankHeld *meshbool.BoolMesh,
	firstVerts []r3.Vec, firstMeshBound float64, held *meshbool.StitchedMesh, move r3.Transform) (
	[]r3.Vec, []float64, float64, float64, bool,
) {
	inv, err := move.Inverse()
	if err != nil {
		return nil, nil, 0, 0, false
	}
	forward, err := massmoment.PlacementRotation(move)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	back, err := massmoment.PlacementRotation(inv)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	// The admitted near-axis motion has no X/YZ cross terms. Keep the local
	// blank ring's held X coordinate exact, and charge the omitted one-ULP
	// axial inverse through the canonical back*forward residual below.
	back[0][0] = ratOne()
	back[0][1], back[0][2] = ratZero(), ratZero()
	charge, err := massmoment.MapChargeOf(back)
	if err != nil {
		return nil, nil, 0, 0, false
	}
	if len(held.VertexBound) != len(held.Verts) || len(firstVerts) == 0 {
		return nil, nil, 0, 0, false
	}
	maxTooth, maxBlank := 0.0, 0.0
	for _, v := range firstVerts {
		maxTooth = max(maxTooth, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	for _, v := range blankHeld.Verts {
		maxBlank = max(maxBlank, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	toothRadius := proofbound.AbsSumUpper(proofbound.Radius3D(maxTooth), firstMeshBound)
	blankRadius := proofbound.AbsSumUpper(proofbound.Radius3D(maxBlank),
		facetproof.FacetBoundMax(blankHeld.Tris, blankHeld.VertexBound))
	deviation := new(big.Rat)
	for row := range 3 {
		for column := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, ratMul(back[row][k], forward[k][column]))
			}
			if row == column {
				entry.Sub(entry, ratOne())
			}
			deviation.Add(deviation, ratAbs(entry))
		}
	}
	toothExtra := proofbound.ProductUpper(proofbound.RatFloatUp(deviation), toothRadius)
	blankExtra := proofbound.ProductUpper(charge.Stretch, blankRadius)
	radialExtra := blankExtra
	commonExtra := proofbound.AbsSumUpper(toothExtra, blankExtra)
	verts := make([]r3.Vec, len(held.Verts))
	beta := make([]float64, len(held.VertexBound))
	maxExtra := 0.0
	for i, v := range held.Verts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, nil, 0, 0, false
			}
		}
		if !proofbound.FiniteVec(v) || proofbound.IsNonFinite(held.VertexBound[i]) {
			return nil, nil, 0, 0, false
		}
		verts[i] = inv.Apply(v)
		verts[i].X = v.X
		if !proofbound.FiniteVec(verts[i]) {
			return nil, nil, 0, 0, false
		}
		round := proofbound.RigidRoundAllow(max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z)), 0)
		extra := proofbound.AbsSumUpper(round, commonExtra)
		beta[i] = proofbound.AbsSumUpper(
			proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), held.VertexBound[i]), extra)
		maxExtra = max(maxExtra, extra)
	}
	return verts, beta, proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), held.Round), maxExtra), radialExtra, true
}
