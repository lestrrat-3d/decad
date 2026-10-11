package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// A second tooth is admitted only as a rotation of the same authenticated
// trimmed source. Both bodies are rebuilt against one immutable blank; a
// general faceted Union still takes the ordinary mesh path.
func tryBevelTwoToothUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	joined, placed := a, b
	jp, joinedOK := joined.payload.(facetedPayload)
	if !joinedOK || jp.bevelJoin == nil {
		joined, placed = b, a
		jp, joinedOK = joined.payload.(facetedPayload)
	}
	tooth, toothOK := placed.payload.(facetedPayload)
	if !joinedOK || !toothOK || jp.bevelJoin == nil ||
		jp.xform != r3.Identity() || tooth.bevelTooth == nil ||
		tooth.bevelTooth.source == nil ||
		jp.bevelJoin.first.pointCone != tooth.bevelTooth.source ||
		tooth.xform != tooth.bevelTooth.motion ||
		!bevelAxisRotation(tooth.bevelTooth.motion) {
		return nil, false, nil
	}
	blankBody := jp.bevelJoin.blank
	blank, ok := exactBevelBlank(blankBody)
	if !ok || !bevelTrimmedSourcesSeparateZ(jp.bevelJoin.first, tooth) {
		return nil, false, nil
	}
	record := tooth.bevelTooth.source
	if record.lower == nil || record.upper == nil || record.base.pointSection == nil ||
		record.base.pointSection.apex != (r3.Vec{}) || record.base.xform != r3.Identity() {
		return nil, false, nil
	}
	boundary, ok := certifyBevelToothBoundary(record.base.pointSection, blank, record.base)
	if !ok {
		return nil, false, nil
	}
	body, err := buildBevelTwoToothJoin(ctx, d, ref, blankBody, blank,
		record, boundary, jp.bevelJoin.first, tooth)
	return body, true, err
}

func bevelAxisRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t == r3.Identity() ||
		t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EX == (r3.Vec{X: 1}) && b.EY.X == 0 && b.EZ.X == 0 &&
		b.EY.Y == b.EZ.Z && b.EY.Z == -b.EZ.Y
}

// Every point of a closed solid lies between its boundary's extreme Z
// coordinates. The held triangle extrema plus each body's certified spatial
// bound therefore prove the two complete source teeth disjoint.
func bevelTrimmedSourcesSeparateZ(first, second facetedPayload) bool {
	interval := func(fp facetedPayload) (float64, float64, bool) {
		if len(fp.verts) == 0 || fp.meshBound < 0 || proofbound.IsNonFinite(fp.meshBound) {
			return 0, 0, false
		}
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, p := range fp.verts {
			if !proofbound.FiniteVec(p) {
				return 0, 0, false
			}
			lo, hi = min(lo, p.Z), max(hi, p.Z)
		}
		return math.Nextafter(lo-fp.meshBound, math.Inf(-1)),
			proofbound.AbsSumUpper(hi, fp.meshBound), true
	}
	aLo, aHi, aOK := interval(first)
	bLo, bHi, bOK := interval(second)
	return aOK && bOK && (aHi < bLo || bHi < aLo)
}

type bevelTwoPlan struct {
	original, placed []bevelPathNode
	caps             [][3]int
	edges            []bevelCapEdge
	lo, hi           float64
	crossingBound    float64
	farRound         float64
	placedRound      float64
	rootNode         []int
	upperBound       float64
	lowerBound       float64
	lip              float64
	capArea          float64
	toolGap          float64
	nodeRound        float64
	snap             float64
}

func makeBevelTwoPlan(ctx context.Context, blank *bevelBlankSource,
	record *pointConeTrimRecord, boundary *bevelToothBoundary,
	motion r3.Transform, charge massmoment.MapCharge) (bevelTwoPlan, error) {
	base := record.base
	path, crossingBound, err := bevelExteriorPath(base, boundary,
		base.pointSection.source, blank)
	if err != nil {
		return bevelTwoPlan{}, err
	}
	lo, hi, ok := bevelRootAngles(path)
	if !ok {
		return bevelTwoPlan{}, fmt.Errorf("%w: root contacts do not bound one angular sector", ErrUnsupported)
	}
	regular := bevelUniformAngles(lo, hi)
	nodes, err := bevelCapBoundary(path, base.pointSection.source.frame,
		blank.coneSection().Slope, regular, lo, hi)
	if err != nil {
		return bevelTwoPlan{}, err
	}
	caps, err := bevelCapTriangulate(ctx, nodes, base.pointSection.source.frame)
	if err != nil {
		return bevelTwoPlan{}, triangulation.WrapLoftError(err)
	}
	nodes, caps, edges, farRound, err := bevelSubdivideCap(ctx, nodes, caps, 3)
	if err != nil {
		return bevelTwoPlan{}, err
	}
	plan := bevelTwoPlan{original: nodes, placed: nodes, caps: caps,
		edges: edges, lo: lo, hi: hi, crossingBound: crossingBound,
		farRound: farRound}
	if motion == r3.Identity() {
		return plan, nil
	}
	plan.placed = append([]bevelPathNode(nil), nodes...)
	for i, node := range nodes {
		p := motion.Apply(node.p)
		if !proofbound.FiniteVec(p) {
			return bevelTwoPlan{}, fmt.Errorf("%w: placed tooth cap has a non-finite station", ErrUnsupported)
		}
		plan.placed[i].p = p
		if node.rootBoundary {
			plan.placed[i].angle = math.Atan2(p.Z, p.Y)
		}
		maxAbs := max(math.Abs(node.p.X), math.Abs(node.p.Y), math.Abs(node.p.Z))
		// The held basis is only approximately orthonormal. Its exact rational
		// map can move the root off the nominal circular blank as well as round
		// Apply's arithmetic, so both distances enter the sewn-boundary proof.
		plan.placedRound = max(plan.placedRound, proofbound.AbsSumUpper(
			proofbound.RigidRoundAllow(maxAbs, 0),
			proofbound.ProductUpper(charge.Stretch, proofbound.Radius3D(maxAbs))))
	}
	plan.lo, plan.hi, ok = bevelRootAngles(plan.placed[:len(path)])
	if !ok {
		return bevelTwoPlan{}, fmt.Errorf("%w: placed root contacts cross the angular seam", ErrUnsupported)
	}
	plan.crossingBound = proofbound.AbsSumUpper(plan.crossingBound, plan.placedRound)
	return plan, nil
}

func buildBevelTwoToothJoin(ctx context.Context, d *Document, ref producerID,
	blankBody *Body, blank *bevelBlankSource, record *pointConeTrimRecord,
	boundary *bevelToothBoundary, first, second facetedPayload) (*Body, error) {
	motions := [2]r3.Transform{r3.Identity(), second.bevelTooth.motion}
	rotation, err := massmoment.PlacementRotation(motions[1])
	if err != nil {
		return nil, err
	}
	placedCharge, err := massmoment.MapChargeOf(rotation)
	if err != nil {
		return nil, err
	}
	var plans [2]bevelTwoPlan
	for i := range plans {
		charge := massmoment.MapCharge{}
		if i == 1 {
			charge = placedCharge
		}
		plan, err := makeBevelTwoPlan(ctx, blank, record, boundary, motions[i], charge)
		if err != nil {
			return nil, err
		}
		plans[i] = plan
	}
	gap := math.Inf(1)
	if plans[0].hi < plans[1].lo {
		gap = plans[1].lo - plans[0].hi
	} else if plans[1].hi < plans[0].lo {
		gap = plans[0].lo - plans[1].hi
	}
	minRadius := min(blank.toeRoot.V, blank.ded.V)
	seamCost := proofbound.AbsSumUpper(
		plans[0].crossingBound, plans[1].crossingBound,
		first.meshBound, second.meshBound)
	if !(minRadius > 0 && seamCost < minRadius/2) {
		return nil, fmt.Errorf("%w: the two root sectors have no angular error bound", ErrUnsupported)
	}
	// For d/r < 1/2, 2d/r exceeds asin(d/r), the largest angle a point
	// can move under the admitted spatial error at this root radius.
	angleCost := proofbound.DivUpper(proofbound.ProductUpper(2, seamCost), minRadius)
	if !(math.Nextafter(gap, math.Inf(-1)) > angleCost) {
		return nil, fmt.Errorf("%w: the two certified root sectors are not separate", ErrUnsupported)
	}
	regular := bevelUniformAngles(plans[0].lo, plans[0].hi)
	regular = append(regular, plans[1].lo, plans[1].hi)
	// The placed root stations differ by a few ulps from the regular grid.
	// Inside either covered sector the cap owns every split of the blank ring;
	// adding an independent regular station there would leave a tiny free edge.
	outside := regular[:0]
	for _, angle := range regular {
		if plans[0].lo < angle && angle < plans[0].hi ||
			plans[1].lo < angle && angle < plans[1].hi {
			continue
		}
		outside = append(outside, angle)
	}
	regular = outside
	angles, _, err := bevelMergedAngles(regular, plans[0].placed)
	if err != nil {
		return nil, err
	}
	angles, plans[1].rootNode, err = bevelMergedAngles(angles, plans[1].placed)
	if err != nil {
		return nil, err
	}
	_, plans[0].rootNode, err = bevelMergedAngles(angles, plans[0].placed)
	if err != nil {
		return nil, err
	}
	blankGroups, err := bevelBlankGroups(blankBody, blank)
	if err != nil {
		return nil, err
	}
	mesh := &bevelJoinedMesh{groups: append([]facetGroup(nil), blankGroups[:]...)}
	ring, blankSag, err := bevelBuildBlankMesh(mesh, blank, angles,
		[][2]float64{{plans[0].lo, plans[0].hi}, {plans[1].lo, plans[1].hi}})
	if err != nil {
		return nil, err
	}
	localBound, localRound, sourceVolume, sourceArea := 0.0, 0.0, 0.0, 0.0
	for i := range plans {
		p := &plans[i]
		groups := []facetGroup(nil)
		if i == 1 {
			groups = second.groups
		}
		p.nodeRound, p.snap, err = bevelBuildToothMesh(mesh, ring,
			p.placed, p.original, p.caps, p.edges, p.rootNode,
			record, motions[i], groups)
		if err != nil {
			return nil, err
		}
		var toolOK bool
		p.toolGap, toolOK = bevelToolFaceGap(blank, record)
		if !toolOK {
			return nil, fmt.Errorf("%w: tooth tools have no finite meridian gap", ErrUnsupported)
		}
		far := make([]r3.Vec, len(p.original))
		for j, node := range p.original {
			far[j] = node.p
		}
		var upperLip, lowerLip float64
		p.upperBound, upperLip, err = pointConeCapBound(far, p.caps, record.upper)
		if err != nil {
			return nil, err
		}
		p.lowerBound, lowerLip, err = pointConeCapBound(far, p.caps, record.lower)
		if err != nil {
			return nil, err
		}
		p.lip = proofbound.AbsSumUpper(1, upperLip, lowerLip)
		p.capArea = pointConeCapArea(far, p.caps)
		localBound = max(localBound, proofbound.AbsSumUpper(p.snap, p.nodeRound,
			proofbound.ProductUpper(p.farRound, p.lip),
			proofbound.ProductUpper(proofbound.AbsSumUpper(record.base.meshBound,
				p.crossingBound, p.placedRound), p.lip),
			max(p.upperBound, p.lowerBound), blank.coneGap, p.toolGap))
		localRound = max(localRound, proofbound.AbsSumUpper(p.snap, p.nodeRound,
			proofbound.ProductUpper(p.farRound, p.lip),
			proofbound.ProductUpper(proofbound.AbsSumUpper(p.crossingBound,
				p.placedRound), p.lip), blank.coneGap, p.toolGap))
		capFactor := max(1, proofbound.ProductUpper(upperLip, upperLip),
			proofbound.ProductUpper(lowerLip, lowerLip))
		volumePart := proofbound.AbsSumUpper(record.base.volSymDiff,
			proofbound.ProductUpper(proofbound.ProductUpper(p.capArea,
				proofbound.AbsSumUpper(p.upperBound, p.lowerBound)), capFactor))
		if i == 1 {
			// An exact affine placement scales symmetric-volume error by its
			// exact determinant, even when r3 accepted it as a rigid rotation.
			volumePart = proofbound.ProductUpper(
				proofbound.AbsSumUpper(1, placedCharge.Volume), volumePart)
		}
		sourceVolume = proofbound.AbsSumUpper(sourceVolume, volumePart)
		baseArea, areaErr := proofbound.PerturbedAreaUpperContext(ctx,
			record.base.verts, record.base.tris, record.base.meshBound)
		if areaErr != nil {
			return nil, areaErr
		}
		areaPart := proofbound.AbsSumUpper(baseArea,
			record.base.areaSlack,
			proofbound.ProductUpper(proofbound.AbsSumUpper(1,
				proofbound.ProductUpper(upperLip, upperLip),
				proofbound.ProductUpper(lowerLip, lowerLip)),
				proofbound.AbsSumUpper(p.capArea, record.base.areaSlack)))
		if i == 1 {
			areaPart = proofbound.ProductUpper(
				proofbound.AbsSumUpper(1, placedCharge.Stretch), areaPart)
		}
		sourceArea = proofbound.AbsSumUpper(sourceArea, areaPart)
	}
	bound := proofbound.AbsSumUpper(blankSag, localBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: two teeth cannot meet the 0.1 mm boundary proof", ErrUnsupported)
	}
	if err := bevelOrientMesh(mesh); err != nil {
		return nil, fmt.Errorf("%w; tooth sectors [%g,%g] and [%g,%g]",
			err, plans[0].lo, plans[0].hi, plans[1].lo, plans[1].hi)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBevelTwoJoin(proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris); err != nil {
		return nil, err
	}
	roundEnvelope := proofbound.AbsSumUpper(blankSag, localRound)
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, roundEnvelope)
	if err != nil {
		return nil, err
	}
	volumeGap := proofbound.AbsSumUpper(sourceVolume,
		proofbound.SweptVolumeAllow(roundEnvelope, area))
	areaForArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := blankBody.Area()
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(areaForArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), sourceArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: two teeth have no finite measure proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := facetedPayload{verts: mesh.verts, tris: mesh.tris, src: mesh.src,
		groups: mesh.groups, vertexBound: vertexBound, contactAudited: true,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: record.base.dPair, xform: r3.Identity()}
	body, err := buildFacetedBody(ctx, d, ref, pp)
	if err != nil {
		return nil, err
	}
	blankVolume, err := blankBody.Volume()
	if err != nil {
		return nil, err
	}
	joinedVolume, err := body.Volume()
	if err != nil {
		return nil, err
	}
	if joinedVolume.Value.Base()-joinedVolume.Bound.Base() <=
		blankVolume.Value.Base()+blankVolume.Bound.Base() {
		return nil, fmt.Errorf("%w: two teeth have no certified occupied volume", ErrUnsupported)
	}
	return body, nil
}
