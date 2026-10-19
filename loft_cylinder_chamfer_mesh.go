package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderChamferSide struct {
	groups    [][2]int
	clipBound float64
	rings     [2][]int
	exact     [2][]proofarith.Xpt
}

type loftCylinderOneCapData struct {
	capGroups [2]int
	outer     [2][]int
	inner     [2][]int
	boreGroup int
	ideal     []proofarith.Xpt
	band      [2]int
}

func buildLoftCylinderCapChamfer(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback float64) (*Body, error) {
	return buildLoftCylinderCapChamferCaps(ctx, d, ref, fp, source, setback,
		[2]bool{true, true}, 0.1, nil, nil)
}

// The inactive top cap is kept verbatim, including its bore rim and middle
// annulus. The active bottom band uses the same fixed-cutter proof as the
// public two-cap route. This private result is sewn to its reflection.
func buildLoftCylinderOneCapChamfer(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback, allowance float64,
	reflectedShift []*big.Rat, data *loftCylinderOneCapData) (*Body, error) {
	return buildLoftCylinderCapChamferCaps(ctx, d, ref, fp, source, setback,
		[2]bool{true, false}, allowance, reflectedShift, data)
}

func buildLoftCylinderCapChamferCaps(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback float64,
	active [2]bool, allowance float64, reflectedShift []*big.Rat,
	oneCap *loftCylinderOneCapData) (*Body, error) {
	// This rewrite has its own fixed work envelope. The source ring and its
	// chorded inset bound the constructed mesh; the crossing audit still
	// performs every exact contact check it would under the default ceiling.
	const (
		maxRimVertices   = 4_096
		maxInsetVertices = 4_096
		maxTriangles     = 32_768
	)
	if len(source.outer[0]) > maxRimVertices || len(source.outer[1]) > maxRimVertices {
		return nil, fmt.Errorf("%w: loft/cylinder cap outline exceeds the chamfer work ceiling", ErrUnsupported)
	}
	if len(fp.VertexBound) != len(fp.Verts) || len(fp.Src) != len(fp.Tris) {
		return nil, fmt.Errorf("%w: loft/cylinder source has incomplete mesh proof", ErrUnsupported)
	}
	old := &joinedMesh{Verts: fp.Verts, Tris: fp.Tris, Src: fp.Src, Groups: fp.Groups}
	for cap := range 2 {
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		valid := false
		if len(source.inner[cap]) > 0 {
			valid = loftCylinderAuditAnnulus(ctx, old, source.outer[cap], source.inner[cap],
				source.capGroups[cap], level, cap == 0)
		} else {
			valid = loftCylinderAuditCap(ctx, old, source.outer[cap], source.capGroups[cap], level, cap == 0)
		}
		if !valid {
			return nil, fmt.Errorf("%w: loft/cylinder source cap no longer tiles its ring", ErrUnsupported)
		}
	}
	sideSource := loftcylinder.ChamferSideSource{
		Outer: source.outer, CapGroups: source.capGroups, BoreGroup: source.boreGroup,
		Z0: source.z0, Z1: source.z1,
	}
	groups, err := loftcylinder.ChamferSideGroups(fp, sideSource)
	if err != nil {
		return nil, err
	}
	mesh := &joinedMesh{Verts: append([]r3.Vec(nil), fp.Verts...),
		Groups: append([]facetGroup(nil), fp.Groups...)}
	bounds := append([]float64(nil), fp.VertexBound...)
	for i, tri := range fp.Tris {
		if fp.Src[i] == source.boreGroup {
			mesh.Add(tri, fp.Src[i])
		}
	}
	middleFirst := len(mesh.Tris)
	clipResult, err := loftcylinder.ChamferClipSide(ctx, mesh, &bounds, fp, sideSource, groups, setback)
	if err != nil {
		return nil, err
	}
	clipped := loftCylinderChamferSide{
		groups: clipResult.Groups, clipBound: clipResult.ClipBound,
		rings: clipResult.Rings, exact: clipResult.Exact,
	}
	middle := [2]int{middleFirst, len(mesh.Tris)}
	if !active[1] {
		if err := loftcylinder.ChamferRestoreTop(mesh, fp, sideSource, groups, clipped.rings[1]); err != nil {
			return nil, err
		}
	}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, clipped.rings[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder side level is not one simple outer ring", ErrUnsupported)
		}
	}
	var inset [2][]int
	var bands [2][2]int
	var radialReach [2]float64
	maxInset := 0.0
	for cap := range 2 {
		if !active[cap] {
			inset[cap] = source.outer[cap]
			bands[cap] = [2]int{len(mesh.Tris), len(mesh.Tris)}
			continue
		}
		samples, err := loftcylinder.ChamferInset(ctx, fp.Verts, source.outer[cap], setback)
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d inset: %w", cap, err)
		}
		if len(samples.Points) > maxInsetVertices {
			return nil, fmt.Errorf("%w: loft/cylinder inset exceeds the chamfer work ceiling", ErrUnsupported)
		}
		maxInset = max(maxInset, samples.Bound)
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		inset[cap] = make([]int, len(samples.Points))
		for i, p := range samples.Points {
			p.Z = level
			inset[cap][i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, p)
			bounds = append(bounds, proofbound.AbsSumUpper(fp.MeshBound, samples.Bound))
		}
		if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, inset[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder inset is not one simple outer ring", ErrUnsupported)
		}
		inSource, err := loftcylinder.ChamferRingNested(ctx, mesh.Verts, source.outer[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if !inSource {
			return nil, fmt.Errorf("%w: loft/cylinder inset crosses its source cap", ErrUnsupported)
		}
		if active[1] {
			inSide, err := loftcylinder.ChamferRingNested(ctx, mesh.Verts, clipped.rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !inSide {
				return nil, fmt.Errorf("%w: loft/cylinder inset crosses its side-level ring", ErrUnsupported)
			}
		}
		if err := loftcylinder.ChamferHeldOutlineGap(ctx, mesh.Verts,
			source.outer[cap], inset[cap], fp.MeshBound); err != nil {
			return nil, err
		}
		if err := loftCylinderChamferExteriorLineage(fp, source, cap); err != nil {
			return nil, err
		}
		outerRadius := proofbound.AbsSumUpper(source.blankRadius, 0.01)
		radialReach[cap], err = loftCylinderChamferCapRadialReach(source, mesh.Verts,
			inset[cap], cap, outerRadius)
		if err != nil {
			return nil, err
		}
		if err := loftCylinderChamferExteriorRimInside(ctx, source, mesh.Verts,
			inset[cap], cap, outerRadius); err != nil {
			return nil, err
		}
		if len(source.inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.boreRadius, fp.MeshBound, samples.Bound,
				clipped.clipBound)
			for i, index := range inset[cap] {
				next := inset[cap][(i+1)%len(inset[cap])]
				if loftcylinder.TriangleMinRadius(mesh.Verts, [3]int{index, next, next}) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder inset edge reaches the exact bore", ErrUnsupported)
				}
			}
		}
		capGroup := source.capGroups[cap]
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		mesh.Groups[capGroup].Origins = append(append([]provenance.Ref(nil), mesh.Groups[capGroup].Origins...),
			provenance.Ref{Producer: ref, Role: role})
		capTris, err := loftcylinder.ChamferCapTriangles(ctx, mesh.Verts, inset[cap], source.inner[cap], cap == 0)
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d: %w", cap, err)
		}
		for _, t := range capTris {
			mesh.Add(t, capGroup)
		}
		bandGroup := len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetGroup{Origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("chamferCap(%d,0,0)", cap)}}})
		bandFirst := len(mesh.Tris)
		bandTris, err := loftcylinder.ChamferBandTriangles(clipped.rings[cap], inset[cap],
			samples.LineStart, samples.ArcStart, len(samples.Points))
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder band %d: %w", cap, err)
		}
		for _, t := range bandTris {
			mesh.Add(t, bandGroup)
		}
		bands[cap] = [2]int{bandFirst, len(mesh.Tris)}
		if len(source.inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.boreRadius, fp.MeshBound, samples.Bound,
				clipped.clipBound)
			for _, tri := range mesh.Tris[bandFirst:] {
				if loftcylinder.TriangleMinRadius(mesh.Verts, tri) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder cap band reaches the exact bore", ErrUnsupported)
				}
			}
		}
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, err
	}
	ideal := loftCylinderChamferIdealVertices(mesh.Verts, clipped)
	var bandSideEdges [2]map[[2]int]struct{}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		capLevel := proofarith.FloatRat(source.z0)
		sideLevel := new(big.Rat).Add(capLevel, proofarith.FloatRat(setback))
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.z1)
			sideLevel = new(big.Rat).Sub(capLevel, proofarith.FloatRat(setback))
		}
		if !loftcylinder.ChamferExactRingSimple(ctx, ideal, clipped.rings[cap]) {
			return nil, fmt.Errorf("%w: ideal loft/cylinder side section is not simple", ErrUnsupported)
		}
		if active[1] {
			nested, err := loftcylinder.ChamferExactNested(ctx, ideal, clipped.rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !nested {
				return nil, fmt.Errorf("%w: ideal loft/cylinder inset crosses its side section", ErrUnsupported)
			}
		}
		bandSideEdges[cap], err = loftcylinder.ChamferBandBoundary(mesh.Tris, bands[cap], clipped.rings[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if err := loftcylinder.ChamferIdealBandLevels(mesh.Tris, ideal, bands[cap], inset[cap],
			capLevel, sideLevel); err != nil {
			return nil, err
		}
	}
	if err := loftcylinder.ChamferContactAudit(ctx, fp, mesh, sideSource, clipResult,
		bands, bandSideEdges, ideal, setback); err != nil {
		return nil, err
	}
	if err := loftcylinder.ChamferIdealBandAudit(ctx, len(mesh.Verts), mesh.Tris, ideal, bands, clipped.clipBound); err != nil {
		return nil, err
	}
	bridgeReach := 0.0
	if active[1] {
		bridgeReach, err = loftcylinder.ChamferBridgeCap(ctx, fp, mesh, ideal,
			bands[1], inset[1], setback, source.z1, source.bridgeBound,
			source.topSourceLevel, source.outer[1])
		if err != nil {
			return nil, err
		}
	}
	needed, err := loftmesh.CylinderChamferBound(fp.MeshBound, maxInset, clipped.clipBound, bridgeReach)
	if err != nil {
		return nil, err
	}
	if needed > allowance || allowance > 0.1 || allowance <= 0 || proofbound.IsNonFinite(allowance) {
		return nil, fmt.Errorf("%w: cap chamfer has no finite boundary allowance", ErrUnsupported)
	}
	for cap, reach := range radialReach {
		if !active[cap] {
			continue
		}
		if cap == 1 {
			reach = proofbound.AbsSumUpper(reach, bridgeReach)
		}
		if proofbound.AbsSumUpper(reach, clipped.clipBound) >= allowance {
			return nil, fmt.Errorf("%w: cap %d radial reach exceeds the published boundary bound",
				ErrUnsupported, cap)
		}
	}
	// The near-band and middle-side proofs read only the finished mesh and
	// source records, so they run side by side. When both fail, the near-band
	// error is returned, as when they ran in sequence.
	if err := loftCylinderChamferReach(ctx, &loftCylinderChamferReachInput{
		fp: fp, mesh: mesh, source: source, ideal: ideal, bands: bands, middle: middle,
		clipBound: clipped.clipBound, allowance: allowance, multipleRings: !active[1],
		reflectedShift: reflectedShift,
	}, orderedwork.EachWorkers(middle[1]-middle[0], 256)); err != nil {
		return nil, err
	}
	// The exact band-to-source witness consumes the full certified tolerance.
	// Publish that allowance so VerifyAll does not understate the new boundary.
	bound := allowance
	if err := loftcylinder.ChamferIdealOrientation(ctx, mesh.Tris, ideal); err != nil {
		return nil, err
	}
	if err := loftCylinderChamferSolidWitness(ctx, fp, source, setback); err != nil {
		return nil, err
	}
	if len(mesh.Tris) > maxTriangles {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer exceeds the triangle work ceiling", ErrUnsupported)
	}
	if err := loftCylinderChamferCrossingAudit(ctx, mesh, source, inset, maxTriangles); err != nil {
		return nil, err
	}
	for cap := range 2 {
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		valid := false
		if len(source.inner[cap]) > 0 {
			valid = loftCylinderAuditAnnulus(ctx, mesh, inset[cap], source.inner[cap],
				source.capGroups[cap], level, cap == 0)
		} else {
			valid = loftCylinderAuditCap(ctx, mesh, inset[cap], source.capGroups[cap], level, cap == 0)
		}
		if !valid {
			return nil, fmt.Errorf("%w: loft/cylinder inset cap does not tile its ring", ErrUnsupported)
		}
	}
	for i := range bounds {
		bounds[i] = max(bounds[i], bound)
	}
	var usedIdeal []bool
	if oneCap != nil {
		usedIdeal = make([]bool, len(ideal))
		for _, tri := range mesh.Tris {
			for _, index := range tri {
				usedIdeal[index] = true
			}
		}
	}
	var remap []int
	mesh.Verts, bounds, remap = loftcylinder.ChamferCompact(mesh.Verts, mesh.Tris, bounds)
	if oneCap != nil {
		oneCap.ideal = make([]proofarith.Xpt, len(mesh.Verts))
		for i, x := range ideal {
			if usedIdeal[i] {
				oneCap.ideal[remap[i]] = x
			}
		}
		oneCap.band = bands[0]
	}
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris,
		clipped.clipBound)
	if err != nil {
		return nil, err
	}
	// The ideal rational clip applies one fixed cutter to both the source and
	// held solids. Vertex motion from that ideal shell to the stored mesh is
	// charged separately; its two embedded closed surfaces share indices.
	volumeGap := proofbound.AbsSumUpper(fp.VolSymDiff,
		proofbound.SweptVolumeAllow(clipped.clipBound, roundArea))
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	result := fp
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = bounds, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, proofbound.AbsSumUpper(fp.AreaSlack, area)
	result.LoftCylinderJoin, result.LoftCylinderBore = nil, nil
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), result))
	if err != nil {
		return nil, err
	}
	if oneCap != nil {
		oneCap.capGroups = source.capGroups
		oneCap.boreGroup = source.boreGroup
		for cap := range 2 {
			oneCap.outer[cap] = make([]int, len(inset[cap]))
			for i, index := range inset[cap] {
				oneCap.outer[cap][i] = remap[index]
			}
			oneCap.inner[cap] = make([]int, len(source.inner[cap]))
			for i, index := range source.inner[cap] {
				oneCap.inner[cap][i] = remap[index]
			}
		}
	}
	return body, nil
}
