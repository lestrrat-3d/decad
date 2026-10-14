package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ExtremeFacesByCentroid returns the faces with the least and greatest
// area-centroid projections onto axis. It currently certifies complete
// point-loft Split slabs, including rigidly transformed copies. A side face need
// not have an individual Centroid reading: the source cone proves that every
// side centroid lies strictly between the two returned cap centroids.
func (b *Body) ExtremeFacesByCentroid(ctx context.Context, axis r3.Vec) (*Face, *Face, error) {
	if ctx == nil || b == nil || !proofbound.FiniteVec(axis) || axis == (r3.Vec{}) {
		return nil, nil, fmt.Errorf("%w: centroid ranking needs a body, context and finite nonzero axis", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	fp, ok := b.n.Payload.(facetedPayload)
	if !ok || fp.pointPlane == nil || fp.pointPlane.lower == nil ||
		fp.pointPlane.base.pointSection == nil ||
		proofbound.IsNonFinite(fp.pointPlaneCapDisplacement) || fp.pointPlaneCapDisplacement < 0 {
		return nil, nil, fmt.Errorf("%w: this body has no complete point-loft Split source", ErrUnsupported)
	}
	n := len(fp.pointPlane.base.verts) - 1
	if n < 3 || len(fp.verts) != 2*n || len(fp.src) != len(fp.faceOf) {
		return nil, nil, fmt.Errorf("%w: the point-loft Split rings are incomplete", ErrUnsupported)
	}
	capStartGroup := len(fp.pointPlane.base.groups) - 1
	capEndGroup := capStartGroup + 1
	start, ok := pointCapGroupFace(b, fp, capStartGroup)
	if !ok {
		return nil, nil, fmt.Errorf("%w: the Split has no unique start cap", ErrUnsupported)
	}
	end, ok := pointCapGroupFace(b, fp, capEndGroup)
	if !ok || start == end {
		return nil, nil, fmt.Errorf("%w: the Split has no unique end cap", ErrUnsupported)
	}
	for _, group := range fp.src {
		if group < 0 || group > capEndGroup {
			return nil, nil, fmt.Errorf("%w: the Split has an unrelated face group", ErrUnsupported)
		}
	}
	axisLen := proofbound.DvLenUpper(proofbound.HeldDelta(axis, r3.Vec{}))
	if proofbound.IsNonFinite(axisLen) {
		return nil, nil, fmt.Errorf("%w: the projection axis has no finite bound", ErrUnsupported)
	}
	startLo, startHi, ok := pointCapCenterInterval(start, axis, axisLen)
	if !ok {
		return nil, nil, fmt.Errorf("%w: the start cap has no bounded centroid", ErrUnsupported)
	}
	endLo, endHi, ok := pointCapCenterInterval(end, axis, axisLen)
	if !ok {
		return nil, nil, fmt.Errorf("%w: the end cap has no bounded centroid", ErrUnsupported)
	}
	lowerLo, lowerHi, ok, err := pointCapRingInterval(ctx, fp.verts[:n], axis, axisLen,
		fp.pointPlaneCapDisplacement)
	if err != nil {
		return nil, nil, err
	}
	if !ok {
		return nil, nil, fmt.Errorf("%w: the lower cap boundary has no finite projection", ErrUnsupported)
	}
	upperLo, upperHi, ok, err := pointCapRingInterval(ctx, fp.verts[n:], axis, axisLen,
		fp.pointPlaneCapDisplacement)
	if err != nil {
		return nil, nil, err
	}
	if !ok {
		return nil, nil, fmt.Errorf("%w: the upper cap boundary has no finite projection", ErrUnsupported)
	}
	// On each true source ray, the area density of a conical strip is
	// t*|(p-a)×p'|. Between ray fractions l and u, the centroid fraction
	// from the lower cap is (2u+l)/(3(u+l)), always in [1/2, 2/3]. A side
	// face's centroid is a positive-weight average over any subset of rays.
	// Evaluate both endpoint fractions so a reversed projection also works.
	sideLo := min(pointCapAffineDown(lowerLo, upperLo, 1, 1, 2),
		pointCapAffineDown(lowerLo, upperLo, 1, 2, 3))
	sideHi := max(pointCapAffineUp(lowerHi, upperHi, 1, 1, 2),
		pointCapAffineUp(lowerHi, upperHi, 1, 2, 3))
	if startHi < sideLo && sideHi < endLo {
		return start, end, nil
	}
	if endHi < sideLo && sideHi < startLo {
		return end, start, nil
	}
	return nil, nil, fmt.Errorf("%w: cap centroid intervals do not separate every side centroid", ErrUnsupported)
}

func pointCapGroupFace(b *Body, fp facetedPayload, group int) (*Face, bool) {
	faces := b.Faces()
	index, ok := facetedtopology.GroupFace(fp.src, fp.faceOf, len(faces), group)
	if !ok {
		return nil, false
	}
	return faces[index], true
}

func pointCapCenterInterval(face *Face, axis r3.Vec, axisLen float64) (float64, float64, bool) {
	center, err := face.Centroid()
	if err != nil {
		return 0, 0, false
	}
	return proofbound.CenterAxisInterval(center.Value, center.Bound.Base(), axis, axisLen)
}

func pointCapRingInterval(ctx context.Context, ring []r3.Vec, axis r3.Vec,
	axisLen, displacement float64) (float64, float64, bool, error) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for i, vertex := range ring {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, false, err
			}
		}
		if !proofbound.FiniteVec(vertex) {
			return 0, 0, false, nil
		}
		dot := proofbound.AxisDot(vertex, axis)
		lo = min(lo, proofbound.RatFloatDown(dot))
		hi = max(hi, proofbound.RatFloatUp(dot))
	}
	delta := proofbound.ProductUpper(displacement, axisLen)
	lo = math.Nextafter(lo-delta, math.Inf(-1))
	hi = math.Nextafter(hi+delta, math.Inf(1))
	return lo, hi, !proofbound.IsNonFinite(lo) && !proofbound.IsNonFinite(hi), nil
}

func pointCapAffineDown(lower, upper float64, lowerWeight, upperWeight, divisor int64) float64 {
	q := pointPlaneAdd(pointPlaneMul(big.NewRat(lowerWeight, 1), ratOfPointPlane(lower)),
		pointPlaneMul(big.NewRat(upperWeight, 1), ratOfPointPlane(upper)))
	return proofbound.RatFloatDown(new(big.Rat).Quo(q, big.NewRat(divisor, 1)))
}

func pointCapAffineUp(lower, upper float64, lowerWeight, upperWeight, divisor int64) float64 {
	q := pointPlaneAdd(pointPlaneMul(big.NewRat(lowerWeight, 1), ratOfPointPlane(lower)),
		pointPlaneMul(big.NewRat(upperWeight, 1), ratOfPointPlane(upper)))
	return proofbound.RatFloatUp(new(big.Rat).Quo(q, big.NewRat(divisor, 1)))
}
