package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
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
	first, err := buildPointPlaneSlab(ctx, d, ref, inner)
	if err != nil {
		return nil, true, fmt.Errorf("point-loft inner slab: %w", err)
	}
	second, err := buildPointPlaneSlab(ctx, d, ref+1, outer)
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

func buildPointPlaneSlab(ctx context.Context, d *Document, ref producerID,
	record pointPlaneSliceRecord) (*Body, error) {
	base := record.Base
	if record.Lower != nil && record.Upper != nil &&
		facetedrecord.PointPlaneOrder(base, record.Lower, record.Upper) != -1 {
		return nil, fmt.Errorf("%w: point-loft slab planes are not ordered", ErrUnsupported)
	}
	n := len(base.Verts) - 1
	if n < 3 || len(base.Tris) <= n || len(base.Src) != len(base.Tris) {
		return nil, fmt.Errorf("%w: point-loft source has no complete fan", ErrUnsupported)
	}
	lower, lowerRound, err := facetedrecord.PointPlaneRing(ctx, base, record.Lower)
	if err != nil {
		return nil, err
	}
	upper, upperRound, err := facetedrecord.PointPlaneRing(ctx, base, record.Upper)
	if err != nil {
		return nil, err
	}
	if record.Upper == nil {
		copy(upper, base.Verts[1:])
	}
	round := max(lowerRound, upperRound)
	hasApex := record.Lower == nil
	vertices := make([]r3.Vec, 0, 2*n+1)
	if hasApex {
		vertices = append(vertices, base.PointSection.Apex)
	} else {
		vertices = append(vertices, lower...)
	}
	upperOffset := len(vertices)
	vertices = append(vertices, upper...)
	groups := make([]facetGroup, 0, len(base.Groups)+2)
	groups = append(groups, base.Groups[:len(base.Groups)-1]...)
	lowerGroup := len(groups)
	if !hasApex {
		groups = append(groups, facetedrecord.PointPlaneCapGroup(ref, provenance.RoleCapStart,
			record.Lower, base, false, record.Lower.FrameTowardFar))
	}
	upperGroup := len(groups)
	upperReversed := false
	if record.Upper != nil {
		upperReversed = !record.Upper.FrameTowardFar
	}
	groups = append(groups, facetedrecord.PointPlaneCapGroup(ref, provenance.RoleCapEnd,
		record.Upper, base, record.Upper == nil, upperReversed))
	triangles := make([][3]int, 0, 2*n+2*(len(base.Tris)-n))
	sources := make([]int, 0, cap(triangles))
	for i := range n {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		side := base.Tris[i]
		if side[0] != 0 || side[1] <= 0 || side[2] <= 0 {
			return nil, fmt.Errorf("%w: point-loft side has no apex fan", ErrUnsupported)
		}
		b, c := side[1]-1, side[2]-1
		source := base.Src[i]
		if source < 0 || source >= len(base.Groups)-1 {
			return nil, fmt.Errorf("%w: point-loft side has no source role", ErrUnsupported)
		}
		if hasApex {
			triangles = append(triangles, [3]int{0, upperOffset + b, upperOffset + c})
			sources = append(sources, source)
			continue
		}
		triangles = append(triangles,
			[3]int{b, upperOffset + b, upperOffset + c},
			[3]int{b, upperOffset + c, c})
		sources = append(sources, source, source)
	}
	for _, capTri := range base.Tris[n:] {
		if capTri[0] == 0 || capTri[1] == 0 || capTri[2] == 0 {
			return nil, fmt.Errorf("%w: point-loft far cap includes its apex", ErrUnsupported)
		}
		mapIndex := func(v int, offset int) int { return offset + v - 1 }
		triangles = append(triangles, [3]int{
			mapIndex(capTri[0], upperOffset), mapIndex(capTri[1], upperOffset),
			mapIndex(capTri[2], upperOffset)})
		sources = append(sources, upperGroup)
		if !hasApex {
			triangles = append(triangles, [3]int{
				mapIndex(capTri[0], 0), mapIndex(capTri[2], 0), mapIndex(capTri[1], 0)})
			sources = append(sources, lowerGroup)
		}
	}
	if layout, ok := loftmesh.PointPlaneSlabCapLayout(base.Tris[:n], len(triangles), upperOffset, hasApex); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, vertices, triangles, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), vertices, triangles)
	}
	if err != nil {
		return nil, err
	}
	stretch := 1.0
	if record.Lower != nil {
		stretch = max(stretch, record.Lower.MapStretch)
	}
	if record.Upper != nil {
		stretch = max(stretch, record.Upper.MapStretch)
	}
	meshBound := proofbound.AbsSumUpper(proofbound.ProductUpper(stretch, base.MeshBound), round)
	if proofbound.IsNonFinite(meshBound) {
		return nil, fmt.Errorf("%w: planar split has no finite boundary bound", ErrUnsupported)
	}
	areaRound, err := proofbound.PerturbedAreaUpperContext(ctx, vertices, triangles, round)
	if err != nil {
		return nil, err
	}
	volumeGap := proofbound.AbsSumUpper(base.VolSymDiff,
		proofbound.SweptVolumeAllow(round, areaRound))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, 0)
	if err != nil {
		return nil, err
	}
	trueArea := proofbound.AbsSumUpper(baseArea, base.AreaSlack)
	// Each side patch is a subset of its original fan. The new caps are
	// projective images of the far cap; their area grows by at most L².
	capStretch := proofbound.ProductUpper(stretch, stretch)
	trueUpper := proofbound.ProductUpper(trueArea,
		proofbound.AbsSumUpper(1, proofbound.ProductUpper(2, capStretch)))
	areaSlack := proofbound.AbsSumUpper(trueUpper, areaRound)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: planar split has no finite occupied-volume or area proof", ErrUnsupported)
	}
	vertexBounds := make([]float64, len(vertices))
	for i := range vertexBounds {
		vertexBounds[i] = meshBound
	}
	recordCopy := record
	return bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), facetedPayload{
		Verts: vertices, Tris: triangles, Src: sources, Groups: groups,
		VertexBound: vertexBounds, MeshBound: meshBound,
		VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity(), PointPlane: &recordCopy,
		PointPlaneCapDisplacement: meshBound,
		PointPlaneSourceEligible:  true,
	}))
}
