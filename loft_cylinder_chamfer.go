package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
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

// loftCylinderChamferSource is the already audited cap and side correspondence
// of an axial loft joined to an exact circular blank. The cap loops are CCW
// when viewed from +Z, including the start cap whose public coedges are CW.
type loftCylinderChamferSource struct {
	blank          *Body
	teeth          []*Body
	originalTeeth  []*Body
	axialBridge    []float64
	bridgeBound    float64
	topSourceLevel *big.Rat
	// Optional physical-height correction for a reflected, axially bridged
	// source tooth. The ring's shared held level stays unchanged.
	ringLevelShift []*big.Rat
	outer, inner   [2][]int
	capGroups      [2]int
	boreGroup      int
	blankRadius    float64
	boreRadius     float64
	boreBound      float64
	innerInset     float64
	outerExcess    float64
	rootCollar     float64
	z0, z1         float64
}

func loftCylinderChamferSourceOf(fp facetedPayload) (loftCylinderChamferSource, bool) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return loftCylinderChamferSource{}, false
	}
	if r := fp.LoftCylinderBore; r != nil {
		return loftCylinderChamferSource{
			blank: bodyHandle(r.Blank), teeth: bodyHandles(r.ProofTeeth),
			originalTeeth: bodyHandles(r.Teeth), axialBridge: r.AxialBridge,
			outer: r.Outer, inner: r.Inner, capGroups: r.CapGroups,
			boreGroup: r.WallGroup, blankRadius: r.BlankRadius,
			boreRadius: r.BoreRadius, boreBound: r.BoreBound, innerInset: r.InnerInset,
			outerExcess: r.OuterExcess, rootCollar: r.RootCollar,
			z0: r.Z0, z1: r.Z1,
		}, true
	}
	if r := fp.LoftCylinderJoin; r != nil {
		return loftCylinderChamferSource{
			blank: bodyHandle(r.Blank), teeth: bodyHandles(r.ProofTeeth),
			originalTeeth: bodyHandles(r.Teeth), axialBridge: r.AxialBridge,
			outer: r.CapOutlines, capGroups: r.CapGroups,
			boreGroup: -1, blankRadius: r.Radius,
			innerInset: r.InnerInset, outerExcess: r.OuterExcess,
			rootCollar: r.RootCollar, z0: r.Z0, z1: r.Z1,
		}, true
	}
	return loftCylinderChamferSource{}, false
}

// tryLoftCylinderCapChamfer accepts exactly the two full outer cap loops of
// an authenticated, unplaced loft/cylinder join or its centered bore result.
// It never interprets an arbitrary faceted edge as an offset carrier.
func tryLoftCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return nil, false, nil
	}
	if asym != nil {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for this loft/cylinder cap", ErrUnsupported)
	}
	if dDelta != 0 || d <= 0 || !proofbound.FiniteVec(r3.Vec{X: d, Y: source.z0, Z: source.z1}) {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap setback has no exact millimeter magnitude", ErrUnsupported)
	}
	startExact := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(d))
	endExact := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(d))
	if startExact.Cmp(endExact) >= 0 || source.z0+d == source.z0 || source.z1-d == source.z1 {
		return nil, true, fmt.Errorf("%w: loft/cylinder cap bands reach each other or round onto a cap", ErrUnsupported)
	}
	bridgeBound, topSourceLevel, err := loftcylinder.ChamferBridgeSources(&loftcylinder.ChamferBridgeSourcesInput{
		ProofTeeth: bodyNodes(source.teeth), OriginalTeeth: bodyNodes(source.originalTeeth),
		AxialBridge: source.axialBridge, Z0: source.z0, Z1: source.z1, BridgeBound: source.bridgeBound,
	})
	if err != nil {
		return nil, true, err
	}
	source.bridgeBound, source.topSourceLevel = bridgeBound, topSourceLevel
	if source.topSourceLevel.Cmp(endExact) <= 0 {
		return nil, true, fmt.Errorf("%w: cap chamfer bridge reaches its side level", ErrUnsupported)
	}
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				if caps[0] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder start cap is split", ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				if caps[1] != nil {
					return nil, true, fmt.Errorf("%w: loft/cylinder end cap is split", ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, true, fmt.Errorf("%w: loft/cylinder result has no current cap faces", ErrUnsupported)
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	if len(selected) != len(edges) {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer repeats an edge", ErrUnsupported)
	}
	for cap, face := range caps {
		if len(face.n.Loops) == 0 || !loftcylinder.ChamferCapWalk(
			face.n.Loops[0], fp.Verts, source.outer[cap], cap == 0) {
			return nil, true, fmt.Errorf("%w: loft/cylinder cap %d has no outer loop", ErrUnsupported, cap)
		}
		for _, ce := range face.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return nil, true, fmt.Errorf("%w: loft/cylinder chamfer needs both complete outer cap loops", ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, true, fmt.Errorf("%w: loft/cylinder chamfer selection includes a bore or side edge", ErrUnsupported)
	}
	body, err := buildLoftCylinderCapChamfer(ctx, documentHandle(b.n.Doc), b.n.Doc.NextProducer, fp, source, d)
	return body, true, err
}

// Cut's authenticated bore wall is kept verbatim. Its same-height radial
// source correspondence remains valid because the new outer cap bands stay
// clear of the entire source bore shell and both cap annuli keep their inner
// loops. The caller audits the rebuilt annuli before publishing the result.
func loftCylinderChamferBoreInheritance(fp facetedPayload, mesh *joinedMesh,
	source loftCylinderChamferSource) error {
	if source.boreGroup < 0 {
		if fp.LoftCylinderBore != nil || len(source.inner[0]) != 0 || len(source.inner[1]) != 0 {
			return fmt.Errorf("%w: chamfer has inconsistent no-bore cap records", ErrUnsupported)
		}
		return nil
	}
	record := fp.LoftCylinderBore
	if record == nil || source.boreGroup != record.WallGroup ||
		source.boreRadius != record.BoreRadius || source.boreBound != record.BoreBound ||
		source.boreBound < 0 || proofbound.IsNonFinite(source.boreBound) ||
		source.boreBound > fp.MeshBound ||
		proofbound.AbsSumUpper(source.boreRadius, source.boreBound, source.innerInset) >= source.blankRadius {
		return fmt.Errorf("%w: chamfer bore lost its source radial bound", ErrUnsupported)
	}
	if len(mesh.Verts) < len(fp.Verts) || len(fp.Tris) != len(fp.Src) ||
		len(mesh.Tris) != len(mesh.Src) {
		return fmt.Errorf("%w: chamfer bore has incomplete held triangles", ErrUnsupported)
	}
	for cap, ring := range source.inner {
		if len(ring) < 3 || len(ring) != len(record.Inner[cap]) {
			return fmt.Errorf("%w: chamfer bore lost its inner cap loop", ErrUnsupported)
		}
		for i, index := range ring {
			if index != record.Inner[cap][i] || index < 0 || index >= len(fp.Verts) ||
				mesh.Verts[index] != fp.Verts[index] {
				return fmt.Errorf("%w: chamfer bore changed an inner cap station", ErrUnsupported)
			}
		}
	}
	old := make([][3]int, 0)
	for i, group := range fp.Src {
		if group == source.boreGroup {
			old = append(old, fp.Tris[i])
		}
	}
	if len(old) == 0 {
		return fmt.Errorf("%w: chamfer bore has no retained wall", ErrUnsupported)
	}
	kept := 0
	for i, group := range mesh.Src {
		if group != source.boreGroup {
			continue
		}
		if kept >= len(old) || mesh.Tris[i] != old[kept] {
			return fmt.Errorf("%w: chamfer changed a held bore-wall facet", ErrUnsupported)
		}
		for _, index := range mesh.Tris[i] {
			if index < 0 || index >= len(fp.Verts) || mesh.Verts[index] != fp.Verts[index] {
				return fmt.Errorf("%w: chamfer changed a held bore-wall vertex", ErrUnsupported)
			}
		}
		kept++
	}
	if kept != len(old) {
		return fmt.Errorf("%w: chamfer dropped a held bore-wall facet", ErrUnsupported)
	}
	return nil
}

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

// The ideal side stations are exact rational points on the old wall. Their
// rounded output images are charged separately after this ideal audit.
func loftCylinderChamferIdealVertices(verts []r3.Vec, side loftCylinderChamferSide) []proofarith.Xpt {
	xverts := make([]proofarith.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proofarith.XptOf(v)
	}
	for cap, ring := range side.rings {
		for i, vertex := range ring {
			xverts[vertex] = side.exact[cap][i]
		}
	}
	return xverts
}

// A mid-height point well inside the exact blank, and outside the bore when
// present, fixes the occupied side. The ideal middle side is an exact subset
// of the old wall's two-triangle strips, so it has the same classification
// there; the cap bands cannot reach this level.
func loftCylinderChamferSolidWitness(ctx context.Context, fp facetedPayload,
	source loftCylinderChamferSource, setback float64) error {
	mid := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(source.z1))
	mid.Quo(mid, big.NewRat(2, 1))
	start := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(setback))
	if mid.Cmp(start) <= 0 || mid.Cmp(end) >= 0 {
		return fmt.Errorf("%w: cap bands leave no common interior slice", ErrUnsupported)
	}
	r := 0.0
	if source.boreRadius > 0 {
		r = proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Add(
			proofarith.FloatRat(source.boreRadius), proofarith.FloatRat(source.blankRadius)),
			big.NewRat(2, 1)))
	}
	if proofbound.AbsSumUpper(r, source.innerInset) >= source.blankRadius ||
		source.boreRadius > 0 && r <= proofbound.AbsSumUpper(source.boreRadius, fp.MeshBound) {
		return fmt.Errorf("%w: cap chamfer lacks an interior blank witness", ErrUnsupported)
	}
	witness := proofarith.Xlerp(
		proofarith.XptOf(r3.Vec{X: r, Z: source.z0}),
		proofarith.XptOf(r3.Vec{X: r, Z: source.z1}),
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
		return fmt.Errorf("%w: cap chamfer shell excludes its blank witness", ErrUnsupported)
	}
	return nil
}

// Union copied these precise Loft face triangles, and Cut retained their
// groups. Match by authenticated origin instead of interpreting a role string
// on a faceted result that may coalesce several public edges.
func loftCylinderChamferToothFaces(source loftCylinderChamferSource) (map[provenance.Ref][]loftcylinder.ChamferToothFace, error) {
	if source.blank == nil || len(source.teeth) == 0 ||
		len(source.originalTeeth) != len(source.teeth) || source.blankRadius <= 0 {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer lost its source operands", ErrUnsupported)
	}
	faces := make(map[provenance.Ref][]loftcylinder.ChamferToothFace)
	for toothIndex, tooth := range source.teeth {
		if tooth == nil || source.originalTeeth[toothIndex] == nil {
			return nil, fmt.Errorf("%w: loft/cylinder tooth source is absent", ErrUnsupported)
		}
		lp, ok := tooth.n.Payload.(loftPayload)
		original, originalOK := source.originalTeeth[toothIndex].n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil ||
			!originalOK || original.SourceIdentity != lp.SourceIdentity ||
			lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) ||
			len(lp.Segment) != 1 {
			return nil, fmt.Errorf("%w: loft/cylinder tooth lost its certified surface", ErrUnsupported)
		}
		root := -1
		for i, segment := range lp.Segment[0] {
			if segment == 5 {
				root = i
				break
			}
		}
		toothFaces := tooth.Faces()
		if root < 3 || len(toothFaces) != lp.Walls+2 || lp.Walls < 2*root {
			return nil, fmt.Errorf("%w: loft/cylinder tooth has no paired exterior faces", ErrUnsupported)
		}
		for j := range root {
			for side := range 2 {
				f := toothFaces[2+2*j+side]
				if len(f.n.Origins) == 0 {
					return nil, fmt.Errorf("%w: loft/cylinder tooth face lost its origin", ErrUnsupported)
				}
				origin := f.n.Origins[0]
				faces[origin] = append(faces[origin], loftcylinder.ChamferToothFace{
					Body: &tooth.n, Loft: lp, Root: root, Cell: j, Bound: lp.Proof.FacetDeparture,
					Scale: proofarith.FloatRat(original.Xform.Basis().EZ.Z),
				})
			}
		}
	}
	return faces, nil
}
