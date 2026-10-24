package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// BuildPointConeTrim assembles a certified cone-trimmed point loft from its source record.
func BuildPointConeTrim(ctx context.Context, env topo.Env,
	record PointConeTrimRecord) (*topo.Body, error) {
	if record.Upper != nil {
		return buildPointConeInsideBand(ctx, env, record)
	}
	if record.Lower == nil {
		return nil, fmt.Errorf("%w: a point-cone trim has no radial limit", decaderr.ErrUnsupported)
	}
	return buildPointConeOuterBand(ctx, env, record)
}

// buildPointConeInsideBand keeps the material inside the upper cone. A prior
// toe Cut supplies the lower cone; otherwise the common loft apex is its lower
// boundary. The upper cap is split where the cone reaches the far profile.
func buildPointConeInsideBand(ctx context.Context, env topo.Env,
	record PointConeTrimRecord) (*topo.Body, error) {
	base, upper, lower := record.Base, record.Upper, record.Lower
	far, caps, sides, farRound, err := PointConeSubdivide(ctx, base, 4)
	if err != nil {
		return nil, err
	}
	capArea, reach := pointcone.CapArea(far, caps), pointcone.ReachUpper(far)
	upperNodeRound, upperQError := 0.0, 0.0
	upperSamples, upperErrs := PointConeFactors(far, upper,
		orderedwork.EachWorkers(len(far), PointConeFactorParallelMin))
	for i := range far {
		if upperErrs[i] != nil {
			return nil, upperErrs[i]
		}
		upperNodeRound = max(upperNodeRound, upperSamples[i].PointError)
		upperQError = max(upperQError, upperSamples[i].QError)
	}
	// A lower limit shares the upper limit's axis (revolveplan.PointConeBoolean).
	capGeom := pointcone.CapGeometry(far, caps, upper.Axis)
	upperCapBound, upperLip, err := pointcone.CapBound(capGeom, upper.BoundConstants())
	if err != nil {
		return nil, err
	}
	upperCapBound = proofbound.AbsSumUpper(upperCapBound,
		proofbound.ProductUpper(reach, proofbound.ProductUpper(upperQError,
			proofbound.ProductUpper(upperLip, upperLip))))
	lowerCapBound, lowerLip, lowerNodeRound := 0.0, 0.0, 0.0
	if lower != nil {
		lowerCapBound, lowerLip, err = pointcone.CapBound(capGeom, lower.BoundConstants())
		if err != nil {
			return nil, err
		}
	}
	outside, inside, _, allSides, far, upperSamples, contactRound, err :=
		pointcone.SplitAtContact(far, caps, sides, upperSamples)
	if err != nil {
		return nil, err
	}
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	allCaps := make([][3]int, 0, len(outside)+len(inside))
	allCaps = append(allCaps, outside...)
	allCaps = append(allCaps, inside...)
	lowerSamples := make([]pointcone.Sample, len(far))
	if lower != nil {
		lowerQError := 0.0
		var lowerErrs []error
		lowerSamples, lowerErrs = PointConeFactors(far, lower,
			orderedwork.EachWorkers(len(far), PointConeFactorParallelMin))
		for i := range far {
			if lowerErrs[i] != nil {
				return nil, lowerErrs[i]
			}
			if lowerSamples[i].Q-lowerSamples[i].QError <= 1 {
				return nil, fmt.Errorf("%w: the lower cone touches the point-loft far cap", decaderr.ErrUnsupported)
			}
			lowerNodeRound = max(lowerNodeRound, lowerSamples[i].PointError)
			lowerQError = max(lowerQError, lowerSamples[i].QError)
		}
		lowerCapBound = proofbound.AbsSumUpper(lowerCapBound,
			proofbound.ProductUpper(reach, proofbound.ProductUpper(lowerQError,
				proofbound.ProductUpper(lowerLip, lowerLip))))
	}
	n := len(far)
	verts := make([]r3.Vec, 0, 2*n+1)
	for i, p := range far {
		if upperSamples[i].Q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(upperSamples[i].Alpha))
		}
	}
	if lower != nil {
		for i, p := range far {
			verts = append(verts, p.Scale(lowerSamples[i].Alpha))
		}
	} else {
		verts = append(verts, r3.Vec{})
	}
	groups := append([]Group(nil), base.Groups...)
	farGroup := len(groups) - 1
	upperGroup := len(groups)
	groups = append(groups, PointConeFacetGroup(upper, false))
	lowerGroup := -1
	if lower != nil {
		lowerGroup = len(groups)
		groups = append(groups, PointConeFacetGroup(lower, true))
	}
	for i := range farGroup {
		groups[i].Surface = surfacegeom.NURBSSurface{}
	}
	tris := make([][3]int, 0, 2*len(allCaps)+2*len(allSides))
	src := make([]int, 0, cap(tris))
	add := func(t [3]int, group int) {
		tris = append(tris, t)
		src = append(src, group)
	}
	for _, t := range outside {
		add(t, upperGroup)
	}
	for _, t := range inside {
		add(t, farGroup)
	}
	if lower != nil {
		for _, t := range allCaps {
			add([3]int{t[2] + n, t[1] + n, t[0] + n}, lowerGroup)
		}
		for _, side := range allSides {
			u, v := side.U, side.V
			add([3]int{u, u + n, v + n}, side.Group)
			add([3]int{u, v + n, v}, side.Group)
		}
	} else {
		apex := len(verts) - 1
		for _, side := range allSides {
			add([3]int{apex, side.V, side.U}, side.Group)
		}
	}
	verts, tris = pointcone.Compact(verts, tris)
	if tessellation.OrientationSign(verts, tris, r3.Vec{}) < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	lip := proofbound.AbsSumUpper(1, upperLip, lowerLip)
	capBound := proofbound.AbsSumUpper(upperCapBound, lowerCapBound)
	roundEnvelope := proofbound.AbsSumUpper(proofbound.ProductUpper(farRound, lip),
		upperNodeRound, lowerNodeRound)
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(base.MeshBound, lip),
		capBound, roundEnvelope)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	// Moving an interpolated cone cap to its radial image can enlarge its
	// swept area by the square of the crossing map's Lipschitz bound.
	capAreaFactor := max(1, proofbound.ProductUpper(upperLip, upperLip),
		proofbound.ProductUpper(lowerLip, lowerLip))
	volumeGap := proofbound.AbsSumUpper(base.VolSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea, capBound), capAreaFactor),
		proofbound.SweptVolumeAllow(roundEnvelope, areaUpper))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, base.MeshBound)
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(base.AreaSlack, baseArea, areaUpper,
		proofbound.ProductUpper(proofbound.AbsSumUpper(1,
			proofbound.ProductUpper(upperLip, upperLip),
			proofbound.ProductUpper(lowerLip, lowerLip)),
			proofbound.AbsSumUpper(capArea, base.AreaSlack)))
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: the point-cone intersection has no finite mesh proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := Payload{
		Verts: verts, Tris: tris, Src: src, Groups: groups, VertexBound: vertexBound,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity(), PointCone: &record,
	}
	return Build(ctx, env, pp)
}

// buildPointConeOuterBand constructs a near cone cap and the original far cap
// over the same triangulated radial domain. The cone must cross every ray
// strictly before the far cap; contact curves use a separate clipping arm.
func buildPointConeOuterBand(ctx context.Context, env topo.Env,
	record PointConeTrimRecord) (*topo.Body, error) {
	base, cone := record.Base, record.Lower
	if !PointConeHasOutsideWitness(base, cone) {
		return nil, fmt.Errorf("%w: the point loft has no certified material outside the cone", decaderr.ErrUnsupported)
	}
	far, caps, sides, farRound, err := PointConeSubdivide(ctx, base, 3)
	if err != nil {
		return nil, err
	}
	if len(far) == 0 || len(caps) == 0 {
		return nil, fmt.Errorf("%w: the point loft has no far cap", decaderr.ErrDegenerate)
	}
	fullCapArea := pointcone.CapArea(far, caps)
	fullReach := pointcone.ReachUpper(far)
	maxNodeRound, maxQError := 0.0, 0.0
	samples, sampleErrs := PointConeFactors(far, cone,
		orderedwork.EachWorkers(len(far), PointConeFactorParallelMin))
	for i := range far {
		if sampleErrs[i] != nil {
			return nil, sampleErrs[i]
		}
		sample := samples[i]
		maxNodeRound = max(maxNodeRound, sample.PointError)
		maxQError = max(maxQError, sample.QError)
	}
	capBound, lip, err := pointcone.CapBound(pointcone.CapGeometry(far, caps, cone.Axis), cone.BoundConstants())
	if err != nil {
		return nil, err
	}
	capBound = proofbound.AbsSumUpper(capBound,
		proofbound.ProductUpper(fullReach, proofbound.ProductUpper(maxQError,
			proofbound.ProductUpper(lip, lip))))
	clippedCaps, _, clippedSides, _, clippedFar, clippedSamples, contactRound, err :=
		pointcone.SplitAtContact(far, caps, sides, samples)
	if err != nil {
		return nil, err
	}
	far, caps, sides, samples = clippedFar, clippedCaps, clippedSides, clippedSamples
	farRound = proofbound.AbsSumUpper(farRound, contactRound)
	verts := append([]r3.Vec(nil), far...)
	for i, p := range far {
		if samples[i].Q <= 1 {
			verts = append(verts, p)
		} else {
			verts = append(verts, p.Scale(samples[i].Alpha))
		}
	}
	groups := append([]Group(nil), base.Groups...)
	capGroup := len(groups) - 1
	coneGroup := len(groups)
	groups = append(groups, PointConeFacetGroup(cone, true))
	for i := range capGroup {
		groups[i].Surface = surfacegeom.NURBSSurface{}
	}
	tris := make([][3]int, 0, len(caps)*2+len(sides)*2)
	src := make([]int, 0, cap(tris))
	add := func(t [3]int, group int) {
		tris = append(tris, t)
		src = append(src, group)
	}
	n := len(far)
	near := func(i int) int {
		if samples[i].Q <= 1 {
			return i
		}
		return i + n
	}
	for _, t := range caps {
		add(t, capGroup)
		add([3]int{near(t[2]), near(t[1]), near(t[0])}, coneGroup)
	}
	for _, side := range sides {
		u, v := side.U, side.V
		if near(u) != u {
			add([3]int{u, near(u), near(v)}, side.Group)
		}
		if near(v) != v {
			add([3]int{u, near(v), v}, side.Group)
		}
	}
	verts, tris = pointcone.Compact(verts, tris)
	if tessellation.OrientationSign(verts, tris, r3.Vec{}) < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	roundEnvelope := proofbound.AbsSumUpper(
		proofbound.ProductUpper(farRound, proofbound.AbsSumUpper(1, lip)), maxNodeRound)
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(base.MeshBound, proofbound.AbsSumUpper(1, lip)),
		capBound, roundEnvelope)
	capArea := fullCapArea
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	capAreaFactor := max(1, proofbound.ProductUpper(lip, lip))
	volumeGap := proofbound.AbsSumUpper(base.VolSymDiff,
		proofbound.ProductUpper(proofbound.ProductUpper(capArea, capBound), capAreaFactor),
		proofbound.SweptVolumeAllow(roundEnvelope, areaUpper))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, base.MeshBound)
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(base.AreaSlack, baseArea,
		proofbound.ProductUpper(proofbound.ProductUpper(lip, lip),
			proofbound.AbsSumUpper(capArea, base.AreaSlack)), areaUpper)
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: the point-cone trim has no finite mesh proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := Payload{
		Verts: verts, Tris: tris, Src: src, Groups: groups, VertexBound: vertexBound,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity(), PointCone: &record,
	}
	return Build(ctx, env, pp)
}
