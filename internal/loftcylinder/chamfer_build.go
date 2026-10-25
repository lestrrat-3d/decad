package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChamferHalf is what the one-cap build hands its mirrored assembly, in the
// compacted result's vertex indices: the cap and bore groups, each cap's
// outer loop (the inset loop on the chamfered cap) and inner loop, the ideal
// rational point of every kept vertex, and the bottom band's triangle range.
type ChamferHalf struct {
	CapGroups    [2]int
	Outer, Inner [2][]int
	BoreGroup    int
	Ideal        []proofarith.Xpt
	Band         [2]int
}

// ChamferBothCaps builds the equal-setback chamfer of both outer cap loops of
// an admitted source (BindCapSetback) under the 0.1 mm boundary allowance.
func ChamferBothCaps(ctx context.Context, env topo.Env, fp facetedrecord.Payload,
	source ChamferSource, setback float64) (*topo.Body, error) {
	return chamferCaps(ctx, env, fp, source, setback, [2]bool{true, true}, 0.1, nil, nil)
}

// ChamferBottomCap chamfers only the bottom cap and fills half for the
// mirrored assembly. The inactive top cap is kept verbatim, including its
// bore rim and middle annulus. The active bottom band uses the same
// fixed-cutter proof as ChamferBothCaps. reflectedShift holds each source
// tooth's exact axial shift for the reflected split-ring checks
// (ChamferReachInput.ReflectedShift), or nil when no tooth is bridged.
func ChamferBottomCap(ctx context.Context, env topo.Env, fp facetedrecord.Payload,
	source ChamferSource, setback, allowance float64, reflectedShift []*big.Rat,
	half *ChamferHalf) (*topo.Body, error) {
	return chamferCaps(ctx, env, fp, source, setback, [2]bool{true, false}, allowance,
		reflectedShift, half)
}

// chamferCapTiles reports whether the group of cap end (0 start, 1 end)
// tiles outer, around the source's bore ring when it has one, at that cap's
// level.
func chamferCapTiles(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	source ChamferSource, end int, outer []int) bool {
	level := source.Z0
	if end == 1 {
		level = source.Z1
	}
	if len(source.Inner[end]) > 0 {
		return AuditAnnulus(ctx, verts, tris, src, outer, source.Inner[end],
			source.CapGroups[end], level, end == 0)
	}
	return AuditCap(ctx, verts, tris, src, outer, source.CapGroups[end], level, end == 0)
}

func chamferCaps(ctx context.Context, env topo.Env, fp facetedrecord.Payload,
	source ChamferSource, setback float64, active [2]bool, allowance float64,
	reflectedShift []*big.Rat, oneCap *ChamferHalf) (*topo.Body, error) {
	// This rewrite has its own fixed work envelope. The source ring and its
	// chorded inset bound the constructed mesh; the crossing audit still
	// performs every exact contact check it would under the default ceiling.
	const (
		maxRimVertices   = 4_096
		maxInsetVertices = 4_096
		maxTriangles     = 32_768
	)
	if len(source.Outer[0]) > maxRimVertices || len(source.Outer[1]) > maxRimVertices {
		return nil, fmt.Errorf("%w: loft/cylinder cap outline exceeds the chamfer work ceiling", decaderr.ErrUnsupported)
	}
	if len(fp.VertexBound) != len(fp.Verts) || len(fp.Src) != len(fp.Tris) {
		return nil, fmt.Errorf("%w: loft/cylinder source has incomplete mesh proof", decaderr.ErrUnsupported)
	}
	for cap := range 2 {
		if !chamferCapTiles(ctx, fp.Verts, fp.Tris, fp.Src, source, cap, source.Outer[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder source cap no longer tiles its ring", decaderr.ErrUnsupported)
		}
	}
	sideSource := ChamferSideSource{
		Outer: source.Outer, CapGroups: source.CapGroups, BoreGroup: source.BoreGroup,
		Z0: source.Z0, Z1: source.Z1,
	}
	groups, err := ChamferSideGroups(fp, sideSource)
	if err != nil {
		return nil, err
	}
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), fp.Verts...),
		Groups: append([]facetedrecord.Group(nil), fp.Groups...)}
	bounds := append([]float64(nil), fp.VertexBound...)
	for i, tri := range fp.Tris {
		if fp.Src[i] == source.BoreGroup {
			mesh.Add(tri, fp.Src[i])
		}
	}
	middleFirst := len(mesh.Tris)
	clipped, err := ChamferClipSide(ctx, mesh, &bounds, fp, sideSource, groups, setback)
	if err != nil {
		return nil, err
	}
	middle := [2]int{middleFirst, len(mesh.Tris)}
	if !active[1] {
		if err := ChamferRestoreTop(mesh, fp, sideSource, groups, clipped.Rings[1]); err != nil {
			return nil, err
		}
	}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		if !BoreRingSimple(ctx, mesh.Verts, clipped.Rings[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder side level is not one simple outer ring", decaderr.ErrUnsupported)
		}
	}
	var inset [2][]int
	var bands [2][2]int
	var radialReach [2]float64
	maxInset := 0.0
	for cap := range 2 {
		if !active[cap] {
			inset[cap] = source.Outer[cap]
			bands[cap] = [2]int{len(mesh.Tris), len(mesh.Tris)}
			continue
		}
		samples, err := ChamferInset(ctx, fp.Verts, source.Outer[cap], setback)
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d inset: %w", cap, err)
		}
		if len(samples.Points) > maxInsetVertices {
			return nil, fmt.Errorf("%w: loft/cylinder inset exceeds the chamfer work ceiling", decaderr.ErrUnsupported)
		}
		maxInset = max(maxInset, samples.Bound)
		level := source.Z0
		if cap == 1 {
			level = source.Z1
		}
		inset[cap] = make([]int, len(samples.Points))
		for i, p := range samples.Points {
			p.Z = level
			inset[cap][i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, p)
			bounds = append(bounds, proofbound.AbsSumUpper(fp.MeshBound, samples.Bound))
		}
		if !BoreRingSimple(ctx, mesh.Verts, inset[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder inset is not one simple outer ring", decaderr.ErrUnsupported)
		}
		inSource, err := ChamferRingNested(ctx, mesh.Verts, source.Outer[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if !inSource {
			return nil, fmt.Errorf("%w: loft/cylinder inset crosses its source cap", decaderr.ErrUnsupported)
		}
		if active[1] {
			inSide, err := ChamferRingNested(ctx, mesh.Verts, clipped.Rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !inSide {
				return nil, fmt.Errorf("%w: loft/cylinder inset crosses its side-level ring", decaderr.ErrUnsupported)
			}
		}
		if err := ChamferHeldOutlineGap(ctx, mesh.Verts,
			source.Outer[cap], inset[cap], fp.MeshBound); err != nil {
			return nil, err
		}
		if err := ChamferExteriorLineage(fp, source, cap); err != nil {
			return nil, err
		}
		outerRadius := proofbound.AbsSumUpper(source.BlankRadius, 0.01)
		radialReach[cap], err = ChamferCapRadialReach(source, mesh.Verts,
			inset[cap], cap, outerRadius)
		if err != nil {
			return nil, err
		}
		if err := ChamferExteriorRimInside(ctx, source, mesh.Verts,
			inset[cap], cap, outerRadius); err != nil {
			return nil, err
		}
		if len(source.Inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.BoreRadius, fp.MeshBound, samples.Bound,
				clipped.ClipBound)
			for i, index := range inset[cap] {
				next := inset[cap][(i+1)%len(inset[cap])]
				if TriangleMinRadius(mesh.Verts, [3]int{index, next, next}) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder inset edge reaches the exact bore", decaderr.ErrUnsupported)
				}
			}
		}
		capGroup := source.CapGroups[cap]
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		mesh.Groups[capGroup].Origins = append(append([]provenance.Ref(nil), mesh.Groups[capGroup].Origins...),
			provenance.Ref{Producer: env.Ref, Role: role})
		capTris, err := ChamferCapTriangles(ctx, mesh.Verts, inset[cap], source.Inner[cap], cap == 0)
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d: %w", cap, err)
		}
		for _, t := range capTris {
			mesh.Add(t, capGroup)
		}
		bandGroup := len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: env.Ref,
			Role: fmt.Sprintf("chamferCap(%d,0,0)", cap)}}})
		bandFirst := len(mesh.Tris)
		bandTris, err := ChamferBandTriangles(clipped.Rings[cap], inset[cap],
			samples.LineStart, samples.ArcStart, len(samples.Points))
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder band %d: %w", cap, err)
		}
		for _, t := range bandTris {
			mesh.Add(t, bandGroup)
		}
		bands[cap] = [2]int{bandFirst, len(mesh.Tris)}
		if len(source.Inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.BoreRadius, fp.MeshBound, samples.Bound,
				clipped.ClipBound)
			for _, tri := range mesh.Tris[bandFirst:] {
				if TriangleMinRadius(mesh.Verts, tri) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder cap band reaches the exact bore", decaderr.ErrUnsupported)
				}
			}
		}
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, err
	}
	ideal := chamferIdealVertices(mesh.Verts, clipped)
	var bandSideEdges [2]map[[2]int]struct{}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		capLevel := proofarith.FloatRat(source.Z0)
		sideLevel := new(big.Rat).Add(capLevel, proofarith.FloatRat(setback))
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.Z1)
			sideLevel = new(big.Rat).Sub(capLevel, proofarith.FloatRat(setback))
		}
		if !ChamferExactRingSimple(ctx, ideal, clipped.Rings[cap]) {
			return nil, fmt.Errorf("%w: ideal loft/cylinder side section is not simple", decaderr.ErrUnsupported)
		}
		if active[1] {
			nested, err := ChamferExactNested(ctx, ideal, clipped.Rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !nested {
				return nil, fmt.Errorf("%w: ideal loft/cylinder inset crosses its side section", decaderr.ErrUnsupported)
			}
		}
		bandSideEdges[cap], err = ChamferBandBoundary(mesh.Tris, bands[cap], clipped.Rings[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if err := ChamferIdealBandLevels(mesh.Tris, ideal, bands[cap], inset[cap],
			capLevel, sideLevel); err != nil {
			return nil, err
		}
	}
	if err := ChamferContactAudit(ctx, fp, mesh, sideSource, clipped,
		bands, bandSideEdges, ideal, setback); err != nil {
		return nil, err
	}
	if err := ChamferIdealBandAudit(ctx, len(mesh.Verts), mesh.Tris, ideal, bands, clipped.ClipBound); err != nil {
		return nil, err
	}
	bridgeReach := 0.0
	if active[1] {
		bridgeReach, err = ChamferBridgeCap(ctx, fp, mesh, ideal,
			bands[1], inset[1], setback, source.Z1, source.BridgeBound,
			source.TopSourceLevel, source.Outer[1])
		if err != nil {
			return nil, err
		}
	}
	needed, err := loftmesh.CylinderChamferBound(fp.MeshBound, maxInset, clipped.ClipBound, bridgeReach)
	if err != nil {
		return nil, err
	}
	if needed > allowance || allowance > 0.1 || allowance <= 0 || proofbound.IsNonFinite(allowance) {
		return nil, fmt.Errorf("%w: cap chamfer has no finite boundary allowance", decaderr.ErrUnsupported)
	}
	for cap, reach := range radialReach {
		if !active[cap] {
			continue
		}
		if cap == 1 {
			reach = proofbound.AbsSumUpper(reach, bridgeReach)
		}
		if proofbound.AbsSumUpper(reach, clipped.ClipBound) >= allowance {
			return nil, fmt.Errorf("%w: cap %d radial reach exceeds the published boundary bound",
				decaderr.ErrUnsupported, cap)
		}
	}
	// The near-band and middle-side proofs read only the finished mesh and
	// source records, so they run side by side. When both fail, the near-band
	// error is returned, as when they ran in sequence.
	if err := ChamferReach(ctx, &ChamferReachInput{
		FP: fp, Mesh: mesh, Source: source, Ideal: ideal, Bands: bands, Middle: middle,
		ClipBound: clipped.ClipBound, Allowance: allowance, MultipleRings: !active[1],
		ReflectedShift: reflectedShift,
	}, orderedwork.EachWorkers(middle[1]-middle[0], 256)); err != nil {
		return nil, err
	}
	// The exact band-to-source witness consumes the full certified tolerance.
	// Publish that allowance so VerifyAll does not understate the new boundary.
	bound := allowance
	if err := ChamferIdealOrientation(ctx, mesh.Tris, ideal); err != nil {
		return nil, err
	}
	if err := chamferSolidWitness(ctx, fp, source, setback); err != nil {
		return nil, err
	}
	if len(mesh.Tris) > maxTriangles {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer exceeds the triangle work ceiling", decaderr.ErrUnsupported)
	}
	if err := ChamferCrossingAudit(ctx, mesh, source, inset, maxTriangles); err != nil {
		return nil, err
	}
	for cap := range 2 {
		if !chamferCapTiles(ctx, mesh.Verts, mesh.Tris, mesh.Src, source, cap, inset[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder inset cap does not tile its ring", decaderr.ErrUnsupported)
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
	mesh.Verts, bounds, remap = ChamferCompact(mesh.Verts, mesh.Tris, bounds)
	if oneCap != nil {
		oneCap.Ideal = make([]proofarith.Xpt, len(mesh.Verts))
		for i, x := range ideal {
			if usedIdeal[i] {
				oneCap.Ideal[remap[i]] = x
			}
		}
		oneCap.Band = bands[0]
	}
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris,
		clipped.ClipBound)
	if err != nil {
		return nil, err
	}
	// The ideal rational clip applies one fixed cutter to both the source and
	// held solids. Vertex motion from that ideal shell to the stored mesh is
	// charged separately; its two embedded closed surfaces share indices.
	volumeGap := proofbound.AbsSumUpper(fp.VolSymDiff,
		proofbound.SweptVolumeAllow(clipped.ClipBound, roundArea))
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	result := fp
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = bounds, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, proofbound.AbsSumUpper(fp.AreaSlack, area)
	result.ExactCopyMoments = nil
	result.LoftCylinderJoin, result.LoftCylinderBore = nil, nil
	body, err := facetedrecord.Build(ctx, env, result)
	if err != nil {
		return nil, err
	}
	if oneCap != nil {
		oneCap.CapGroups = source.CapGroups
		oneCap.BoreGroup = source.BoreGroup
		for cap := range 2 {
			oneCap.Outer[cap] = make([]int, len(inset[cap]))
			for i, index := range inset[cap] {
				oneCap.Outer[cap][i] = remap[index]
			}
			oneCap.Inner[cap] = make([]int, len(source.Inner[cap]))
			for i, index := range source.Inner[cap] {
				oneCap.Inner[cap][i] = remap[index]
			}
		}
	}
	return body, nil
}

// The ideal side stations are exact rational points on the old wall. Their
// rounded output images are charged separately after this ideal audit.
func chamferIdealVertices(verts []r3.Vec, side ChamferSideResult) []proofarith.Xpt {
	xverts := make([]proofarith.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proofarith.XptOf(v)
	}
	for cap, ring := range side.Rings {
		for i, vertex := range ring {
			xverts[vertex] = side.Exact[cap][i]
		}
	}
	return xverts
}

// A mid-height point well inside the exact blank, and outside the bore when
// present, fixes the occupied side. The ideal middle side is an exact subset
// of the old wall's two-triangle strips, so it has the same classification
// there; the cap bands cannot reach this level.
func chamferSolidWitness(ctx context.Context, fp facetedrecord.Payload,
	source ChamferSource, setback float64) error {
	mid := new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(source.Z1))
	mid.Quo(mid, big.NewRat(2, 1))
	start := new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(setback))
	if mid.Cmp(start) <= 0 || mid.Cmp(end) >= 0 {
		return fmt.Errorf("%w: cap bands leave no common interior slice", decaderr.ErrUnsupported)
	}
	r := 0.0
	if source.BoreRadius > 0 {
		r = proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Add(
			proofarith.FloatRat(source.BoreRadius), proofarith.FloatRat(source.BlankRadius)),
			big.NewRat(2, 1)))
	}
	if proofbound.AbsSumUpper(r, source.InnerInset) >= source.BlankRadius ||
		source.BoreRadius > 0 && r <= proofbound.AbsSumUpper(source.BoreRadius, fp.MeshBound) {
		return fmt.Errorf("%w: cap chamfer lacks an interior blank witness", decaderr.ErrUnsupported)
	}
	witness := proofarith.Xlerp(
		proofarith.XptOf(r3.Vec{X: r, Z: source.Z0}),
		proofarith.XptOf(r3.Vec{X: r, Z: source.Z1}),
		big.NewInt(1), big.NewInt(2))
	indices := make([]int, len(fp.Tris))
	for i := range indices {
		indices[i] = i
	}
	inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness,
		meshbool.NewParityMesh(fp.Verts, fp.Tris), indices)
	if err != nil {
		return err
	}
	if !inside || on {
		return fmt.Errorf("%w: cap chamfer shell excludes its blank witness", decaderr.ErrUnsupported)
	}
	return nil
}
