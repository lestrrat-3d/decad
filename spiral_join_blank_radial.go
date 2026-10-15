package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// spiralHeldBlankRadialAllowance bounds the actual held face's radius at
// each X against its exact source meridian. The caller separately proves
// this face is a full radial graph with spiralHeldBlankCellGraph.
func spiralHeldBlankRadialAllowance(ctx context.Context, blank *Body,
	held *meshbool.BoolMesh, source *spiralBlankSource, edge int) (float64, bool, error) {
	if ctx == nil || blank == nil || held == nil || source == nil ||
		edge < 0 || edge >= len(source.EdgeIndex) || len(held.Src) != len(held.Tris) {
		return 0, false, nil
	}
	group := spiralBlankFaceGroup(blank, source, edge)
	if group < 0 {
		return 0, false, nil
	}
	cycle := [6]planePoint{source.ToeAxis, source.HeelAxis, source.HeelOuter,
		source.Ded, source.ToeRoot, source.ToeOuter}
	a, z := cycle[edge], cycle[(edge+1)%len(cycle)]
	if a.U == z.U {
		return 0, false, nil
	}
	m := lfDiv(lfSub(lfF(z.V), lfF(a.V)), lfSub(lfF(z.U), lfF(a.U)))
	b := lfSub(lfF(a.V), lfMul(m, lfF(a.U)))
	limit := new(big.Rat)
	seen := false
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		if held.Src[i] != group {
			continue
		}
		seen = true
		var points [3]r3.Vec
		for k, index := range tri {
			if index < 0 || index >= len(held.Verts) ||
				!proofbound.FiniteVec(held.Verts[index]) {
				return 0, false, nil
			}
			points[k] = held.Verts[index]
		}
		triangleLimit, ok := spiralsector.TriangleMeridianError(points, m, b)
		if !ok {
			return 0, false, nil
		}
		if triangleLimit.Cmp(limit) > 0 {
			limit = triangleLimit
		}
	}
	if !seen {
		return 0, false, nil
	}
	allowance := proofbound.RatFloatUp(limit)
	return allowance, !proofbound.IsNonFinite(allowance), nil
}
