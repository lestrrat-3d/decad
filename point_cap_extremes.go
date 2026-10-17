package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
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
	if !ok || fp.PointPlane == nil || fp.PointPlane.Lower == nil ||
		fp.PointPlane.Base.PointSection == nil ||
		proofbound.IsNonFinite(fp.PointPlaneCapDisplacement) || fp.PointPlaneCapDisplacement < 0 {
		return nil, nil, fmt.Errorf("%w: this body has no complete point-loft Split source", ErrUnsupported)
	}
	n := len(fp.PointPlane.Base.Verts) - 1
	if n < 3 || len(fp.Verts) != 2*n || len(fp.Src) != len(fp.FaceOf) {
		return nil, nil, fmt.Errorf("%w: the point-loft Split rings are incomplete", ErrUnsupported)
	}
	capStartGroup := len(fp.PointPlane.Base.Groups) - 1
	capEndGroup := capStartGroup + 1
	start, ok := pointCapGroupFace(b, fp, capStartGroup)
	if !ok {
		return nil, nil, fmt.Errorf("%w: the Split has no unique start cap", ErrUnsupported)
	}
	end, ok := pointCapGroupFace(b, fp, capEndGroup)
	if !ok || start == end {
		return nil, nil, fmt.Errorf("%w: the Split has no unique end cap", ErrUnsupported)
	}
	for _, group := range fp.Src {
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
	lowerLo, lowerHi, ok, err := facetedrecord.PointCapRingInterval(ctx, fp.Verts[:n], axis, axisLen,
		fp.PointPlaneCapDisplacement)
	if err != nil {
		return nil, nil, err
	}
	if !ok {
		return nil, nil, fmt.Errorf("%w: the lower cap boundary has no finite projection", ErrUnsupported)
	}
	upperLo, upperHi, ok, err := facetedrecord.PointCapRingInterval(ctx, fp.Verts[n:], axis, axisLen,
		fp.PointPlaneCapDisplacement)
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
	sideLo := min(facetedrecord.PointCapAffineDown(lowerLo, upperLo, 1, 1, 2),
		facetedrecord.PointCapAffineDown(lowerLo, upperLo, 1, 2, 3))
	sideHi := max(facetedrecord.PointCapAffineUp(lowerHi, upperHi, 1, 1, 2),
		facetedrecord.PointCapAffineUp(lowerHi, upperHi, 1, 2, 3))
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
	index, ok := facetedtopology.GroupFace(fp.Src, fp.FaceOf, len(faces), group)
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
