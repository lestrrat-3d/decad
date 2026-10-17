package facetedrecord

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// PointPlaneOrder returns -1 when a lies strictly nearer the apex than b,
// +1 for the reverse, and zero if the source boundary bound cannot decide.
func PointPlaneOrder(base Payload, a, b *PointPlaneLimit) int {
	// b.D*a.g - a.D*b.g is affine on the source cap. Every cap
	// triangle is convex, so its vertex minima decide the whole held cap.
	cx := pointPlaneSub(pointPlaneMul(b.D, a.Nx), pointPlaneMul(a.D, b.Nx))
	cy := pointPlaneSub(pointPlaneMul(b.D, a.Ny), pointPlaneMul(a.D, b.Ny))
	margin := pointPlaneMul(pointPlaneAdd(pointPlaneAbs(cx), pointPlaneAbs(cy)),
		ratOfPointPlane(base.MeshBound))
	allPositive, allNegative := true, true
	for _, far := range base.Verts[1:] {
		x := pointPlaneSub(ratOfPointPlane(far.X), ratOfPointPlane(base.PointSection.Apex.X))
		y := pointPlaneSub(ratOfPointPlane(far.Y), ratOfPointPlane(base.PointSection.Apex.Y))
		v := pointPlaneAdd(pointPlaneMul(cx, x), pointPlaneMul(cy, y))
		allPositive = allPositive && v.Cmp(margin) > 0
		allNegative = allNegative && v.Cmp(new(big.Rat).Neg(margin)) < 0
	}
	if allPositive {
		return -1
	}
	if allNegative {
		return 1
	}
	return 0
}

// ValidatePointPlane proves every source ray meets the complete finite
// ribbon strictly inside both its length and its sweep span. The source
// point-loft boundary allowance covers every curved point between stations.
func ValidatePointPlane(base Payload, plane *PointPlaneLimit) error {
	if base.PointSection == nil || len(base.Verts) < 4 || base.MeshBound < 0 ||
		proofbound.IsNonFinite(base.MeshBound) || proofbound.IsNonFinite(base.VolSymDiff) ||
		proofbound.IsNonFinite(base.AreaSlack) {
		return fmt.Errorf("%w: a planar point-loft split needs a proved source fan", decaderr.ErrUnsupported)
	}
	apex := base.PointSection.Apex
	normalAllow := pointPlaneMul(plane.NormalL1(), ratOfPointPlane(base.MeshBound))
	var denMin *big.Rat
	reach := 0.0
	for _, far := range base.Verts[1:] {
		den := plane.Denominator(apex, far)
		lo := pointPlaneSub(den, normalAllow)
		if lo.Sign() <= 0 || lo.Cmp(plane.D) <= 0 {
			return fmt.Errorf("%w: the cut plane does not cross every source ray strictly", decaderr.ErrUnsupported)
		}
		if denMin == nil || lo.Cmp(denMin) < 0 {
			denMin = lo
		}
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(far, apex)))
	}
	normUp := proofbound.RatFloatUp(plane.NormalL1())
	dUp := proofbound.RatFloatUp(plane.D)
	denLo := proofbound.RatFloatDown(denMin)
	reach = proofbound.AbsSumUpper(reach, base.MeshBound)
	alphaUp := proofbound.DivUpper(dUp, denLo)
	stretch := proofbound.ProductUpper(alphaUp, proofbound.AbsSumUpper(1,
		proofbound.DivUpper(proofbound.ProductUpper(reach, normUp), denLo)))
	if proofbound.IsNonFinite(stretch) {
		return fmt.Errorf("%w: the cut plane has no finite source map", decaderr.ErrUnsupported)
	}
	plane.MapStretch = stretch
	// A projective map sends each source cap triangle to a triangle. A
	// convex finite sheet contains that image when it contains every
	// mapped vertex with room for the true source displacement.
	ex := pointPlaneSub(ratOfPointPlane(plane.B.U), ratOfPointPlane(plane.A.U))
	ey := pointPlaneSub(ratOfPointPlane(plane.B.V), ratOfPointPlane(plane.A.V))
	e2 := pointPlaneAdd(pointPlaneMul(ex, ex), pointPlaneMul(ey, ey))
	cutAllow := proofbound.ProductUpper(stretch, base.MeshBound)
	spanMargin := pointPlaneMul(pointPlaneAdd(pointPlaneAbs(ex), pointPlaneAbs(ey)),
		ratOfPointPlane(cutAllow))
	zMargin := ratOfPointPlane(cutAllow)
	z0 := pointPlaneAdd(ratOfPointPlane(plane.Z0), zMargin)
	z1 := pointPlaneSub(ratOfPointPlane(plane.Z1), zMargin)
	for _, far := range base.Verts[1:] {
		q, err := PointPlaneMapExact(apex, far, plane)
		if err != nil {
			return err
		}
		along := pointPlaneAdd(
			pointPlaneMul(pointPlaneSub(q[0], ratOfPointPlane(plane.A.U)), ex),
			pointPlaneMul(pointPlaneSub(q[1], ratOfPointPlane(plane.A.V)), ey))
		if along.Cmp(spanMargin) <= 0 ||
			along.Cmp(pointPlaneSub(e2, spanMargin)) >= 0 ||
			q[2].Cmp(z0) <= 0 || q[2].Cmp(z1) >= 0 {
			return fmt.Errorf("%w: the finite split sheet ends inside the source section", decaderr.ErrUnsupported)
		}
	}
	return nil
}

// PointPlaneCapGroup carries the source or cut-plane cap record into a slab.
func PointPlaneCapGroup(ref provenance.Producer, role string, plane *PointPlaneLimit,
	base Payload, far, reverse bool) Group {
	if far {
		frame := base.PointSection.Source.Frame
		profilePlane := base.PointSection.Plane
		apexSide := proofarith.XdotRat(
			proofarith.Xsub(proofarith.XptOf(base.PointSection.Apex),
				proofarith.XptOf(profilePlane.Origin)),
			proofarith.Xcross(proofarith.XptOf(profilePlane.U),
				proofarith.XptOf(profilePlane.V)),
		)
		origins := append([]provenance.Ref(nil), base.Groups[len(base.Groups)-1].Origins...)
		origins = append(origins, provenance.Ref{Producer: ref, Role: role})
		sourceCap := base.Groups[len(base.Groups)-1]
		return Group{Origins: origins, Surface: surfacegeom.Plane{Frame: frame}, Planar: true,
			Reversed: apexSide.Sign() > 0,
			Centroid: sourceCap.Centroid, HasCentroid: sourceCap.HasCentroid}
	}
	origins := append([]provenance.Ref(nil), plane.Origins...)
	origins = append(origins, provenance.Ref{Producer: ref, Role: role})
	return Group{Origins: origins, Surface: plane.Surface, Planar: true,
		NormalBound: plane.NormalBound, Reversed: reverse,
		Centroid: plane.Centroid, HasCentroid: plane.HasCentroid}
}

// PointPlaneRing maps the original source ring and accumulates rounding.
func PointPlaneRing(ctx context.Context, base Payload, plane *PointPlaneLimit) (
	[]r3.Vec, float64, error) {
	vertices := make([]r3.Vec, len(base.Verts)-1)
	if plane == nil {
		copy(vertices, base.Verts[1:])
		return vertices, 0, nil
	}
	round := 0.0
	for i, far := range base.Verts[1:] {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, 0, err
			}
		}
		q, err := PointPlaneMapExact(base.PointSection.Apex, far, plane)
		if err != nil {
			return nil, 0, err
		}
		vertices[i], round, err = pointPlaneRoundInto(q, round)
		if err != nil {
			return nil, 0, err
		}
	}
	return vertices, round, nil
}

func pointPlaneRoundInto(q [3]*big.Rat, prior float64) (r3.Vec, float64, error) {
	v, gap, err := PointPlaneRound(q)
	return v, max(prior, gap), err
}
