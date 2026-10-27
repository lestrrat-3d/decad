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
// point-loft Split slabs, including rigidly transformed copies, and a faceted body every face of which has its own
// Centroid reading, such as a SolidFromMesh body. On a point-loft slab a side face need not have an individual
// Centroid reading: the source cone proves that every side centroid lies strictly between the two returned cap
// centroids. On the other bodies the least face is strictly below every other face's centroid and the greatest
// strictly above, or the answer is ErrUnsupported.
func (b *Body) ExtremeFacesByCentroid(ctx context.Context, axis r3.Vec) (*Face, *Face, error) {
	if ctx == nil || b == nil || !proofbound.FiniteVec(axis) || axis == (r3.Vec{}) {
		return nil, nil, fmt.Errorf("%w: centroid ranking needs a body, context and finite nonzero axis", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	fp, ok := b.n.Payload.(facetedPayload)
	if ok && fp.PointPlane == nil {
		return extremeFacesByHeldCentroids(b, axis)
	}
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
	order, err := facetedrecord.PointCapExtremeOrder(ctx, fp, axis, axisLen,
		startLo, startHi, endLo, endHi)
	if err != nil {
		return nil, nil, err
	}
	if order > 0 {
		return start, end, nil
	}
	return end, start, nil
}

// extremeFacesByHeldCentroids ranks a body every face of which has its own Centroid reading, as the planar faces of a
// SolidFromMesh body do: the least face is the one whose centroid interval lies strictly below every other face's, the
// greatest the one strictly above every other's, and a tie or an overlap of intervals is ErrUnsupported because the
// readings do not decide the order. A face without a reading refuses the whole body.
func extremeFacesByHeldCentroids(b *Body, axis r3.Vec) (*Face, *Face, error) {
	faces := b.Faces()
	if len(faces) < 2 {
		return nil, nil, fmt.Errorf("%w: a body of fewer than two faces has no pair of extreme faces", ErrUnsupported)
	}
	axisLen := proofbound.DvLenUpper(proofbound.HeldDelta(axis, r3.Vec{}))
	if proofbound.IsNonFinite(axisLen) {
		return nil, nil, fmt.Errorf("%w: the projection axis has no finite bound", ErrUnsupported)
	}
	lo := make([]float64, len(faces))
	hi := make([]float64, len(faces))
	for i, f := range faces {
		l, h, ok := pointCapCenterInterval(f, axis, axisLen)
		if !ok {
			return nil, nil, fmt.Errorf("%w: face %d has no bounded centroid", ErrUnsupported, i)
		}
		lo[i], hi[i] = l, h
	}
	least, greatest := -1, -1
	for i := range faces {
		below, above := true, true
		for j := range faces {
			if i == j {
				continue
			}
			below = below && hi[i] < lo[j]
			above = above && lo[i] > hi[j]
		}
		if below {
			least = i
		}
		if above {
			greatest = i
		}
	}
	if least < 0 || greatest < 0 {
		return nil, nil, fmt.Errorf("%w: the face centroids do not decide a unique least and greatest along the axis", ErrUnsupported)
	}
	return faces[least], faces[greatest], nil
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
