package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// InverseHeldResult maps a copy's stitched held result back to the first
// tooth's frame. Inverse placement is used only for comparing output chains
// with their unplaced source curves.
//
// rec is the exact rotation the copy's true solid is the image under. It must
// fix the X axis through the origin, and move must have zero translation, so the
// inverse is the exact orthogonal map Q^T with no translation. The certified
// output β is charged for the float inverse's entrywise gap to Q^T, applied once
// per copy, and for the inverse's rounding. Q^T is an isometry, so β takes no
// stretch factor, and Q^T fixes the blank's radius, so the radial allowance is
// the gap alone. The result is refused when any charge is not finite.
func InverseHeldResult(ctx context.Context, blankHeld *meshbool.BoolMesh,
	firstVerts []r3.Vec, firstMeshBound float64, held *meshbool.StitchedMesh, move r3.Transform,
	rec *exactrotation.Rotation) (
	[]r3.Vec, []float64, float64, float64, bool,
) {
	q, ok := AxisCopyMap(rec)
	if !ok || move.Translation() != (r3.Vec{}) {
		return nil, nil, 0, 0, false
	}
	inv, err := move.Inverse()
	if err != nil || inv.Translation() != (r3.Vec{}) {
		return nil, nil, 0, 0, false
	}
	if len(held.VertexBound) != len(held.Verts) || len(firstVerts) == 0 ||
		proofbound.IsNonFinite(firstMeshBound) || firstMeshBound < 0 {
		return nil, nil, 0, 0, false
	}
	// Entry (i,j) of the float inverse against entry (j,i) of Q.
	basis := inv.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	gapSum := new(big.Rat)
	for j, col := range cols {
		for i, f := range [3]float64{col.X, col.Y, col.Z} {
			entry := new(big.Rat).SetFloat64(f)
			if entry == nil {
				return nil, nil, 0, 0, false
			}
			gapSum.Add(gapSum, ratAbs(new(big.Rat).Sub(q[j][i], entry)))
		}
	}
	gap := proofbound.RatFloatUp(gapSum)
	maxBlank := 0.0
	for _, v := range blankHeld.Verts {
		maxBlank = max(maxBlank, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	blankRadius := proofbound.AbsSumUpper(proofbound.Radius3D(maxBlank),
		facetproof.FacetBoundMax(blankHeld.Tris, blankHeld.VertexBound))
	radialExtra := proofbound.ProductUpper(gap, blankRadius)
	if proofbound.IsNonFinite(gap) || proofbound.IsNonFinite(blankRadius) || proofbound.IsNonFinite(radialExtra) {
		return nil, nil, 0, 0, false
	}
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
		// Q^T fixes X exactly, so the held X coordinate is the exact image's.
		verts[i] = inv.Apply(v)
		verts[i].X = v.X
		reach := max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
		extra := proofbound.AbsSumUpper(proofbound.RigidRoundAllow(reach, 0), proofbound.ProductUpper(gap, reach))
		beta[i] = proofbound.AbsSumUpper(held.VertexBound[i], extra)
		if !proofbound.FiniteVec(verts[i]) || proofbound.IsNonFinite(extra) || proofbound.IsNonFinite(beta[i]) {
			return nil, nil, 0, 0, false
		}
		maxExtra = max(maxExtra, extra)
	}
	round := proofbound.AbsSumUpper(held.Round, maxExtra)
	if proofbound.IsNonFinite(round) {
		return nil, nil, 0, 0, false
	}
	return verts, beta, round, radialExtra, true
}
