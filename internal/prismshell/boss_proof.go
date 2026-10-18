package prismshell

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// PrismUnionBossShellSimilarity checks the boss transform and brackets its
// horizontal similarity scale before the local offset is mapped to the boss.
func PrismUnionBossShellSimilarity(xform r3.Transform) (float64, float64, bool) {
	basis := xform.Basis()
	c, s := basis.EX.X, basis.EX.Y
	if basis.EX.Z != 0 || basis.EY != (r3.Vec{X: -s, Y: c}) ||
		basis.EZ != (r3.Vec{Z: 1}) || !proofbound.FiniteVec(basis.EX) ||
		!proofbound.FiniteVec(xform.Translation()) {
		return 0, 0, false
	}
	length2 := new(big.Rat).Add(
		new(big.Rat).Mul(proofarith.FloatRat(c), proofarith.FloatRat(c)),
		new(big.Rat).Mul(proofarith.FloatRat(s), proofarith.FloatRat(s)))
	lo, hi := proofbound.RatSqrtDown(length2), proofbound.RatSqrtUp(length2)
	return lo, hi, lo > 0.99 && hi < 1.01
}

// PrismUnionBossShellLevels checks the exact order and root thickness of the
// plate and boss levels.
func PrismUnionBossShellLevels(plateBottom, bossBottom, interfaceZ, bossTop, t float64) bool {
	pt := func(v float64) *big.Rat { return proofarith.FloatRat(v) }
	return new(big.Rat).Add(pt(plateBottom), pt(t)).Cmp(pt(bossBottom)) < 0 &&
		pt(bossBottom).Cmp(pt(interfaceZ)) < 0 &&
		pt(interfaceZ).Cmp(pt(bossTop)) < 0 &&
		new(big.Rat).Sub(pt(interfaceZ), pt(bossBottom)).Cmp(pt(t)) == 0
}

// PrismUnionBossShellCurveBound encloses the root's quarter-circle chord gap.
func PrismUnionBossShellCurveBound(t float64, stations int) float64 {
	// The planar quarter circle (q(w), s(w)) has second-derivative norm at
	// most 4t on 0 <= w <= 1. Linear interpolation at matching w across a
	// cell of width 1/N differs by at most 4t/(8N²) = t/(2N²). Insetting
	// both rectangle coordinates stretches this gap by at most sqrt(2),
	// rounded safely to 3/2.
	n := float64(stations)
	return proofbound.DivUpper(proofbound.ProductUpper(1.5, t), 2*n*n)
}

// PrismUnionBossShellTrueVolume encloses the outer solid less its three
// cavity regions, including the rounded root band.
func PrismUnionBossShellTrueVolume(plateRect, bossRect BossRect,
	plateBottom, interfaceZ, bossBottom, bossTop, unitLo, unitHi, t float64) proofbound.RatInterval {
	point := func(v float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(v)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := proofbound.Interval(proofarith.FloatRat(unitLo), proofarith.FloatRat(unitHi))
	w := mul(point(bossRect.Width()), scale)
	h := mul(point(bossRect.Height()), scale)
	tv := point(t)
	t2, t3 := mul(tv, tv), mul(mul(tv, tv), tv)
	plateArea := mul(point(plateRect.Width()), point(plateRect.Height()))
	lowerRect := plateRect.Inset(t)
	lowerArea := mul(point(lowerRect.Width()), point(lowerRect.Height()))
	outer := add(mul(plateArea, point(interfaceZ-plateBottom)),
		mul(mul(w, h), point(bossTop-interfaceZ)))
	lower := mul(lowerArea, point(bossBottom-plateBottom-t))
	upper := mul(mul(sub(w, mul(point(2), tv)), sub(h, mul(point(2), tv))),
		point(bossTop-interfaceZ))
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	widths := add(w, h)
	band := add(sub(mul(mul(w, h), tv), mul(mul(point(2), widths), t2)),
		add(mul(mul(mul(widths, pi), t2), point(0.5)),
			sub(mul(t3, proofbound.Interval(big.NewRat(20, 3), big.NewRat(20, 3))),
				mul(mul(point(2), pi), t3))))
	return sub(outer, add(add(lower, upper), band))
}

// PrismUnionBossShellPayload builds and proves the authenticated inward cup
// mesh. The caller retains the source identity and commits the result body.
func PrismUnionBossShellPayload(ctx context.Context, ref provenance.Producer, input BossMeshInput,
	sourceVolSymDiff, sourceAreaSlack, t float64) (facetedrecord.Payload, *facetproof.MeshAudit, error) {
	mesh, err := PrismUnionBossShellMeshOf(ctx, ref, input, t)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Verts, mesh.Tris, 0)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	if err := revolvemesh.RevolveContactAudit(budget, data, mesh.Tris, 0); err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	trueVolume := PrismUnionBossShellTrueVolume(input.PlateRect, input.BossRect,
		input.PlateBottom, input.InterfaceZ, input.BossBottom, input.BossTop,
		input.UnitLo, input.UnitHi, t)
	if trueVolume.Lo == nil || trueVolume.Lo.Sign() <= 0 ||
		len(audit.CompVol) != 1 || audit.CompVol[0].Sign() <= 0 {
		return facetedrecord.Payload{}, nil,
			fmt.Errorf("%w: boss shell has no proven positive occupied volume", decaderr.ErrUnsupported)
	}
	curve := PrismUnionBossShellCurveBound(t, input.Stations)
	// All source-preserving exterior facets keep their Boolean proof. The new
	// cavity is a family of nested convex rectangles. The root's true
	// quarter-circle surface stays within curve plus the station lift of a
	// chorded surface matched at the same rational curve parameter. This
	// matching lets Z move near the horizontal tangent;
	// a same-Z radial gap would be larger. Every homotopy section remains a
	// strictly nested rectangle. Its swept root area is below 4 P t, where P
	// is the boss perimeter; multiplying by the displacement bounds the
	// occupied-volume change without using a same-Z horizontal gap, which
	// becomes large near the quarter-circle's horizontal tangent. The lower
	// cavity is exact; the upper cavity uses a fixed inset.
	width := proofbound.ProductUpper(input.BossRect.Width(), input.UnitHi)
	height := proofbound.ProductUpper(input.BossRect.Height(), input.UnitHi)
	perimeter := proofbound.ProductUpper(2, proofbound.AbsSumUpper(width, height))
	station := mesh.MaxVertexError
	rootGap := proofbound.AbsSumUpper(curve, proofbound.ProductUpper(4, station))
	upperGap := proofbound.ProductUpper(4, station)
	if !(rootGap < t/16) {
		return facetedrecord.Payload{}, nil,
			fmt.Errorf("%w: boss shell root stations are too coarse for a bounded cavity", decaderr.ErrUnsupported)
	}
	rootVol := proofbound.ProductUpper(
		proofbound.ProductUpper(4, proofbound.ProductUpper(perimeter, t)), rootGap)
	upperVol := proofbound.ProductUpper(input.BossTop-input.InterfaceZ,
		proofbound.AbsSumUpper(proofbound.ProductUpper(perimeter, upperGap),
			proofbound.ProductUpper(4, proofbound.ProductUpper(upperGap, upperGap))))
	capArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(width, height),
		proofbound.ProductUpper(input.PlateRect.Inset(t).Width(), input.PlateRect.Inset(t).Height()))
	levelVol := proofbound.ProductUpper(proofbound.ProductUpper(4, capArea), station)
	volumeBound := proofbound.AbsSumUpper(sourceVolSymDiff, rootVol, upperVol, levelVol)
	heldVolume := audit.CompVol[0]
	if proofbound.IntervalAbsUpper(proofbound.IntervalSub(trueVolume,
		proofbound.PointInterval(heldVolume))).Cmp(proofarith.FloatRat(volumeBound)) > 0 {
		return facetedrecord.Payload{}, nil,
			fmt.Errorf("%w: boss shell held volume exceeds its occupied-volume allowance", decaderr.ErrUnsupported)
	}
	for i := mesh.RootStart; i < mesh.BossPointEnd; i++ {
		mesh.VertexBound[i] = proofbound.AbsSumUpper(mesh.VertexBound[i], curve)
	}
	bound := facetproof.FacetBoundMax(mesh.Tris, mesh.VertexBound)
	// The root patch's curved area is no larger than the boss perimeter times
	// sqrt(2) times a quarter-circle arc. Bounding both true and held root
	// patches by twice that value covers their area difference independently
	// of any cancellation across the four sides.
	rootAreaSlack := proofbound.ProductUpper(
		proofbound.ProductUpper(4, perimeter), proofbound.ProductUpper(t, proofbound.RatFloatUp(proofbound.PiUpper)))
	payload := facetedrecord.Payload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: mesh.VertexBound, MeshBound: bound, VolSymDiff: volumeBound,
		AreaSlack: proofbound.AbsSumUpper(sourceAreaSlack, rootAreaSlack),
		DPair:     facetedrecord.SourceDiameterUpper(mesh.Verts, bound),
		Xform:     r3.Identity(), ContactAudited: true,
	}
	return payload, audit, nil
}
