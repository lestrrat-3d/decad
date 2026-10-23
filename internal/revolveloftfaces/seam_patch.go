package revolveloftfaces

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ArcSeamPatch is an open source-face construction piece. Its vertex bounds
// exclude the complete face's curve and twist proofs.
type ArcSeamPatch struct {
	Vertices     []r3.Vec
	Triangles    [][3]int
	VertexBounds []float64
	Origins      []provenance.Ref
	Seam         []int
}

// meridianPoint keeps the endpoint on the exact radial ray of the seam.
// The radial residual and float movement are charged separately; angular
// chording is excluded from this vertex bound.
func meridianPoint(seam loftface.RatVec, end sectionrecord.Point2) (r3.Vec, float64, bool) {
	if seam[0] == nil || seam[1] == nil || seam[2] == nil || end.V <= 0 {
		return r3.Vec{}, 0, false
	}
	radialSquared := lfAdd(lfMul(seam[1], seam[1]), lfMul(seam[2], seam[2]))
	if radialSquared.Sign() <= 0 {
		return r3.Vec{}, 0, false
	}
	squared, _ := radialSquared.Float64()
	if proofbound.IsNonFinite(squared) || squared <= 0 {
		return r3.Vec{}, 0, false
	}
	seamRadius := math.Sqrt(squared)
	if proofbound.IsNonFinite(seamRadius) || seamRadius <= 0 {
		return r3.Vec{}, 0, false
	}
	// The stored square root chooses a radial scale. Its exact residual is
	// charged below, so rounding cannot admit contact.
	scale := seamDiv(lfF(end.V), lfF(seamRadius))
	exact := loftface.RatVec{lfF(end.U), lfMul(seam[1], scale), lfMul(seam[2], scale)}
	point, round, ok := HeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	radius := lfF(end.V)
	difference := seamAbs(lfSub(lfAdd(lfMul(exact[1], exact[1]),
		lfMul(exact[2], exact[2])), lfMul(radius, radius)))
	residual := proofbound.RatFloatUp(seamDiv(difference, radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}
