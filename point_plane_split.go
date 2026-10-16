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

// pointPlaneOrder returns -1 when a lies strictly nearer the apex than b,
// +1 for the reverse, and zero if the source boundary bound cannot decide.
func pointPlaneOrder(base facetedPayload, a, b *pointPlaneLimit) int {
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

// validatePointPlane proves every source ray meets the complete finite
// ribbon strictly inside both its length and its sweep span. The source
// point-loft boundary allowance covers every curved point between stations.
func validatePointPlane(base facetedPayload, plane *pointPlaneLimit) error {
	if base.PointSection == nil || len(base.Verts) < 4 || base.MeshBound < 0 ||
		proofbound.IsNonFinite(base.MeshBound) || proofbound.IsNonFinite(base.VolSymDiff) ||
		proofbound.IsNonFinite(base.AreaSlack) {
		return fmt.Errorf("%w: a planar point-loft split needs a proved source fan", ErrUnsupported)
	}
	apex := base.PointSection.Apex
	normalAllow := pointPlaneMul(plane.NormalL1(), ratOfPointPlane(base.MeshBound))
	var denMin *big.Rat
	reach := 0.0
	for _, far := range base.Verts[1:] {
		den := plane.Denominator(apex, far)
		lo := pointPlaneSub(den, normalAllow)
		if lo.Sign() <= 0 || lo.Cmp(plane.D) <= 0 {
			return fmt.Errorf("%w: the cut plane does not cross every source ray strictly", ErrUnsupported)
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
		return fmt.Errorf("%w: the cut plane has no finite source map", ErrUnsupported)
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
		q, err := pointPlaneMapExact(apex, far, plane)
		if err != nil {
			return err
		}
		along := pointPlaneAdd(
			pointPlaneMul(pointPlaneSub(q[0], ratOfPointPlane(plane.A.U)), ex),
			pointPlaneMul(pointPlaneSub(q[1], ratOfPointPlane(plane.A.V)), ey))
		if along.Cmp(spanMargin) <= 0 ||
			along.Cmp(pointPlaneSub(e2, spanMargin)) >= 0 ||
			q[2].Cmp(z0) <= 0 || q[2].Cmp(z1) >= 0 {
			return fmt.Errorf("%w: the finite split sheet ends inside the source section", ErrUnsupported)
		}
	}
	return nil
}

func pointPlaneMapExact(apex, far r3.Vec, plane *pointPlaneLimit) ([3]*big.Rat, error) {
	den := plane.Denominator(apex, far)
	if den.Sign() <= 0 {
		return [3]*big.Rat{}, fmt.Errorf("%w: a source ray does not meet the split plane", ErrUnsupported)
	}
	alpha := new(big.Rat).Quo(plane.D, den)
	a := [3]float64{apex.X, apex.Y, apex.Z}
	p := [3]float64{far.X, far.Y, far.Z}
	var q [3]*big.Rat
	for i := range q {
		q[i] = pointPlaneAdd(ratOfPointPlane(a[i]),
			pointPlaneMul(alpha, pointPlaneSub(ratOfPointPlane(p[i]), ratOfPointPlane(a[i]))))
	}
	return q, nil
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
	if err := validatePointPlane(record.Base, plane); err != nil {
		return nil, true, err
	}
	if center, proved, err := pointPlaneCapCentroid(ctx, record.Base, plane); err != nil {
		return nil, true, err
	} else {
		plane.Centroid, plane.HasCentroid = vecMeasurementToInternal(center), proved
	}
	if record.Lower != nil {
		switch pointPlaneOrder(record.Base, record.Lower, plane) {
		case -1:
			// The new plane is strictly beyond the existing lower cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's lower plane", ErrUnsupported)
		}
	}
	if record.Upper != nil {
		switch pointPlaneOrder(record.Base, plane, record.Upper) {
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

func pointPlaneCapGroup(ref producerID, role string, plane *pointPlaneLimit,
	base facetedPayload, far, reverse bool) facetGroup {
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
		return facetGroup{Origins: origins, Surface: surfacegeom.Plane{Frame: frame}, Planar: true,
			Reversed: apexSide.Sign() > 0,
			Centroid: sourceCap.Centroid, HasCentroid: sourceCap.HasCentroid}
	}
	origins := append([]provenance.Ref(nil), plane.Origins...)
	origins = append(origins, provenance.Ref{Producer: ref, Role: role})
	return facetGroup{Origins: origins, Surface: plane.Surface, Planar: true,
		NormalBound: plane.NormalBound, Reversed: reverse,
		Centroid: plane.Centroid, HasCentroid: plane.HasCentroid}
}

func pointPlaneRing(ctx context.Context, base facetedPayload, plane *pointPlaneLimit) (
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
		q, err := pointPlaneMapExact(base.PointSection.Apex, far, plane)
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
	v, gap, err := facetedrecord.PointPlaneRound(q)
	return v, max(prior, gap), err
}

// pointPlaneSlabCapLayout is a slab's wall and cap structure as
// buildPointPlaneSlab emits it: n side walls (apex slab) or 2n (two planes),
// then the far-cap triangles, interleaved upper/lower for a two-plane slab.
// sides are the source fan's side triangles.
func pointPlaneSlabCapLayout(sides [][3]int, triCount, upperOffset int, hasApex bool) (loftmesh.LoftCapLayout, bool) {
	ring, ok := loftmesh.FanRing(sides)
	if !ok {
		return loftmesh.LoftCapLayout{}, false
	}
	n := len(sides)
	shift := func(offset int) []int {
		out := make([]int, len(ring))
		for i, r := range ring {
			out[i] = offset + r
		}
		return out
	}
	if hasApex {
		if triCount < n {
			return loftmesh.LoftCapLayout{}, false
		}
		layout := loftmesh.LoftCapLayout{Walls: n, Loops0: [][]int{shift(upperOffset)}, Loops1: [][]int{{0}}}
		for t := n; t < triCount; t++ {
			layout.Cap0 = append(layout.Cap0, t)
		}
		return layout, true
	}
	if triCount < 2*n || (triCount-2*n)%2 != 0 {
		return loftmesh.LoftCapLayout{}, false
	}
	layout := loftmesh.LoftCapLayout{Walls: 2 * n, Loops0: [][]int{shift(upperOffset)}, Loops1: [][]int{shift(0)}}
	for t := 2 * n; t < triCount; t += 2 {
		layout.Cap0 = append(layout.Cap0, t)
		layout.Cap1 = append(layout.Cap1, t+1)
	}
	return layout, true
}

func buildPointPlaneSlab(ctx context.Context, d *Document, ref producerID,
	record pointPlaneSliceRecord) (*Body, error) {
	base := record.Base
	if record.Lower != nil && record.Upper != nil &&
		pointPlaneOrder(base, record.Lower, record.Upper) != -1 {
		return nil, fmt.Errorf("%w: point-loft slab planes are not ordered", ErrUnsupported)
	}
	n := len(base.Verts) - 1
	if n < 3 || len(base.Tris) <= n || len(base.Src) != len(base.Tris) {
		return nil, fmt.Errorf("%w: point-loft source has no complete fan", ErrUnsupported)
	}
	lower, lowerRound, err := pointPlaneRing(ctx, base, record.Lower)
	if err != nil {
		return nil, err
	}
	upper, upperRound, err := pointPlaneRing(ctx, base, record.Upper)
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
		groups = append(groups, pointPlaneCapGroup(ref, provenance.RoleCapStart,
			record.Lower, base, false, record.Lower.FrameTowardFar))
	}
	upperGroup := len(groups)
	upperReversed := false
	if record.Upper != nil {
		upperReversed = !record.Upper.FrameTowardFar
	}
	groups = append(groups, pointPlaneCapGroup(ref, provenance.RoleCapEnd,
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
	if layout, ok := pointPlaneSlabCapLayout(base.Tris[:n], len(triangles), upperOffset, hasApex); ok {
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
