package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

func ratOfPointPlane(v float64) *big.Rat { return proofarith.FloatRat(v) }

func pointPlaneAbs(q *big.Rat) *big.Rat { return new(big.Rat).Abs(q) }

// pointPlaneAdd, pointPlaneSub and pointPlaneMul return a fresh Rat and read
// their operands only. Operands must be in lowest terms: the result always
// equals the exact value, but is in lowest terms only for reduced operands (a
// 3/9 operand gives 3/9 where big.Rat gives 1/3). Every producer in this
// package reduces: FloatRat through ratOfPointPlane and lfF, big.NewRat
// literals, lfOne, lfZero, and results of these helpers and of raw Quo, Abs and
// Neg. The guard TestNoRatWritesThroughNumDenom (internal/proof) bars in-place
// Num and Denom writes outside internal/proof.
func pointPlaneAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }

func pointPlaneSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }

func pointPlaneMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }

// exactPointPlaneSheet recognizes the actual step-E sheet class. A rectangle
// drawn as Patch does not pass this gate because it has a different source.
func exactPointPlaneSheet(tool *Body, apex r3.Vec) (*pointPlaneLimit, bool) {
	cp, ok := tool.n.Payload.(chainPayload)
	if !ok || tool.Kind() != BodySheet || cp.Xform != r3.Identity() ||
		cp.Frame.Origin() != (r3.Vec{}) || cp.Frame.U() != (r3.Vec{X: 1}) ||
		cp.Frame.V() != (r3.Vec{Y: 1}) || cp.SectionDelta != 0 ||
		cp.Z0Delta != 0 || cp.Z1Delta != 0 || len(cp.Chains) != 1 ||
		len(cp.Chains[0].Segments) != 1 || len(tool.Faces()) != 1 {
		return nil, false
	}
	line, ok := cp.Chains[0].Segments[0].(lineSeg)
	if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
		(line.TStart == 1 && line.TEnd == 0)) {
		return nil, false
	}
	for _, vertex := range tool.Vertices() {
		if vertex.Position().Bound.Mag() != 0 {
			return nil, false
		}
	}
	face := tool.Faces()[0]
	surface, ok := face.n.Surface.(surfacegeom.Plane)
	if !ok || !proofbound.FiniteVec(apex) || !proofbound.FiniteVec(r3.Vec{
		X: line.Start.U, Y: line.Start.V, Z: line.End.U}) ||
		!proofbound.FiniteVec(r3.Vec{X: line.End.V, Y: cp.Z0, Z: cp.Z1}) {
		return nil, false
	}
	ax, ay := ratOfPointPlane(line.Start.U), ratOfPointPlane(line.Start.V)
	bx, by := ratOfPointPlane(line.End.U), ratOfPointPlane(line.End.V)
	nx := pointPlaneSub(ay, by)
	ny := pointPlaneSub(bx, ax)
	if nx.Sign() == 0 && ny.Sign() == 0 {
		return nil, false
	}
	d := pointPlaneAdd(
		pointPlaneMul(nx, pointPlaneSub(ax, ratOfPointPlane(apex.X))),
		pointPlaneMul(ny, pointPlaneSub(ay, ratOfPointPlane(apex.Y))),
	)
	if d.Sign() == 0 {
		return nil, false
	}
	if d.Sign() < 0 {
		nx.Neg(nx)
		ny.Neg(ny)
		d.Neg(d)
	}
	// Sketch may canonicalize a one-line chain independently of CreateLine's
	// endpoint order. Compare the actual face frame to the geometric plane
	// after orienting that plane away from the apex.
	frameNormal := surface.Frame.N()
	if !proofbound.FiniteVec(frameNormal) || proofbound.IsNonFinite(face.n.NormalBound) ||
		face.n.NormalBound < 0 {
		return nil, false
	}
	dot := pointPlaneAdd(pointPlaneMul(nx, ratOfPointPlane(frameNormal.X)),
		pointPlaneMul(ny, ratOfPointPlane(frameNormal.Y)))
	allow := pointPlaneMul(pointPlaneAdd(pointPlaneAbs(nx), pointPlaneAbs(ny)),
		ratOfPointPlane(face.n.NormalBound))
	if pointPlaneAbs(dot).Cmp(allow) <= 0 {
		return nil, false
	}
	return &pointPlaneLimit{Nx: nx, Ny: ny, D: d,
		A: line.Start, B: line.End, Z0: cp.Z0, Z1: cp.Z1,
		Surface: surface, Origins: face.originRefs(), NormalBound: face.n.NormalBound,
		FrameTowardFar: dot.Sign() > 0}, true
}

func pointPlaneSource(target *Body) (pointPlaneSliceRecord, bool) {
	fp, ok := target.n.Payload.(facetedPayload)
	if !ok || target.Kind() != BodySolid || !target.IsSolid() ||
		!fp.PointPlaneSourceEligible || fp.Xform != r3.Identity() {
		return pointPlaneSliceRecord{}, false
	}
	if fp.PointPlane != nil {
		return *fp.PointPlane, true
	}
	if fp.PointSection == nil {
		return pointPlaneSliceRecord{}, false
	}
	return pointPlaneSliceRecord{Base: fp}, true
}

// splitPointPlane is a separate Split arm. It handles only authenticated
// point-loft fans and one exact chain-sheet class; other pairs continue to
// the shared-generator Split dispatcher.
func (d *Document) splitPointPlane(ctx context.Context, budget *proofbound.WorkBudget,
	target, tool *Body) ([]*Body, bool, error) {
	record, ok := pointPlaneSource(target)
	if !ok {
		return nil, false, nil
	}
	plane, ok := exactPointPlaneSheet(tool, record.Base.PointSection.Apex)
	if !ok {
		return nil, false, nil
	}
	if err := facetedrecord.ValidatePointPlane(record.Base, plane); err != nil {
		return nil, true, err
	}
	if center, proved, err := facetedrecord.PointPlaneCapCentroid(ctx, record.Base, plane); err != nil {
		return nil, true, err
	} else {
		plane.Centroid, plane.HasCentroid = center, proved
	}
	if record.Lower != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, record.Lower, plane) {
		case -1:
			// The new plane is strictly beyond the existing lower cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's lower plane", ErrUnsupported)
		}
	}
	if record.Upper != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, plane, record.Upper) {
		case -1:
			// The new plane is strictly before the existing upper cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's upper plane", ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return nil, true, err
	}
	ref := d.nextProducerID()
	inner := record
	inner.Upper = plane
	outer := record
	outer.Lower = plane
	first, err := bodyResult(facetedrecord.BuildPointPlaneSlab(ctx,
		envOf(documentNode(d), ref), inner))
	if err != nil {
		return nil, true, fmt.Errorf("point-loft inner slab: %w", err)
	}
	second, err := bodyResult(facetedrecord.BuildPointPlaneSlab(ctx,
		envOf(documentNode(d), ref+1), outer))
	if err != nil {
		return nil, true, fmt.Errorf("point-loft outer slab: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	pieces := []*Body{first, second}
	d.commitMany(pieces, target, tool)
	return pieces, true, nil
}
