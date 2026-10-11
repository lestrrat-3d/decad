package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// A patterned join rebuilds every certified tooth against the same immutable
// blank. The source list is recorded only for this bounded eight-tooth class.
func tryBevelPatternUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	joined, placed := a, b
	jp, joinedOK := joined.payload.(facetedPayload)
	if !joinedOK || jp.bevelJoin == nil || len(jp.bevelJoin.placed) == 0 {
		joined, placed = b, a
		jp, joinedOK = joined.payload.(facetedPayload)
	}
	tooth, toothOK := placed.payload.(facetedPayload)
	if !joinedOK || !toothOK || jp.bevelJoin == nil ||
		len(jp.bevelJoin.placed) == 0 || len(jp.bevelJoin.placed) >= 7 ||
		jp.xform != r3.Identity() || tooth.bevelTooth == nil ||
		tooth.bevelTooth.source == nil ||
		jp.bevelJoin.first.pointCone != tooth.bevelTooth.source ||
		tooth.xform != tooth.bevelTooth.motion ||
		!bevelAxisRotation(tooth.bevelTooth.motion) {
		return nil, false, nil
	}
	teeth := make([]facetedPayload, 0, len(jp.bevelJoin.placed)+2)
	teeth = append(teeth, jp.bevelJoin.first)
	for _, prior := range jp.bevelJoin.placed {
		if prior.bevelTooth == nil || prior.bevelTooth.source != tooth.bevelTooth.source ||
			prior.xform != prior.bevelTooth.motion || !bevelAxisRotation(prior.bevelTooth.motion) {
			return nil, false, nil
		}
		teeth = append(teeth, prior)
	}
	teeth = append(teeth, tooth)
	if !bevelTrimmedSourcesSeparateBox(teeth) {
		return nil, false, nil
	}
	blankBody := jp.bevelJoin.blank
	blank, ok := exactBevelBlank(blankBody)
	if !ok {
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
	body, err := buildBevelPatternJoin(ctx, d, ref, blankBody, blank, record, boundary, teeth)
	return body, true, err
}

func bevelAngleAt(p r3.Vec, ringStart float64) float64 {
	angle := math.Atan2(p.Z, p.Y)
	if angle < ringStart {
		angle += 2 * math.Pi
	}
	return angle
}

func bevelRootNodeAngles(path []bevelPathNode) (float64, float64, bool) {
	a, b := path[0].angle, path[len(path)-1].angle
	if proofbound.IsNonFinite(a) || proofbound.IsNonFinite(b) ||
		a == b || math.Abs(a-b) >= math.Pi {
		return 0, 0, false
	}
	return min(a, b), max(a, b), true
}

// Both route distances around the circular root are rounded down. A sector
// that crosses the chosen angular seam has a nonpositive distance here and
// cannot enter the shared-ring construction.
func bevelPatternAngularGap(a, b bevelTwoPlan) float64 {
	if b.lo < a.lo {
		a, b = b, a
	}
	if a.hi >= b.lo {
		return math.Inf(-1)
	}
	direct := new(big.Rat).Sub(proofarith.FloatRat(b.lo), proofarith.FloatRat(a.hi))
	wrap := new(big.Rat).Add(proofbound.TwoPiInterval().Lo, proofarith.FloatRat(a.lo))
	wrap.Sub(wrap, proofarith.FloatRat(b.hi))
	return min(proofbound.RatFloatDown(direct), proofbound.RatFloatDown(wrap))
}

func bevelPatternMeshBound(bound float64, charge massmoment.MapCharge) float64 {
	return proofbound.AbsSumUpper(bound,
		proofbound.ProductUpper(charge.Stretch, bound))
}

// A closed source solid lies inside its boundary coordinate ranges. One
// separated coordinate after both mesh bounds are charged proves the two
// complete source teeth disjoint, including their interiors.
func bevelTrimmedSourcesSeparateBox(teeth []facetedPayload) bool {
	type box struct{ lo, hi r3.Vec }
	boxes := make([]box, len(teeth))
	for i, tooth := range teeth {
		if len(tooth.verts) == 0 || tooth.meshBound < 0 || proofbound.IsNonFinite(tooth.meshBound) {
			return false
		}
		charge := massmoment.MapCharge{}
		if i != 0 {
			rotation, err := massmoment.PlacementRotation(tooth.bevelTooth.motion)
			if err != nil {
				return false
			}
			charge, err = massmoment.MapChargeOf(rotation)
			if err != nil {
				return false
			}
		}
		// PlaceMesh carries source displacement and Apply rounding, but does
		// not scale that displacement by the held basis's stretch. Multiplying
		// the whole placed bound is conservative for both parts.
		bound := bevelPatternMeshBound(tooth.meshBound, charge)
		lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
		for _, p := range tooth.verts {
			if !proofbound.FiniteVec(p) {
				return false
			}
			lo.X, lo.Y, lo.Z = min(lo.X, p.X), min(lo.Y, p.Y), min(lo.Z, p.Z)
			hi.X, hi.Y, hi.Z = max(hi.X, p.X), max(hi.Y, p.Y), max(hi.Z, p.Z)
		}
		boxes[i] = box{lo: r3.Vec{
			X: math.Nextafter(lo.X-bound, math.Inf(-1)),
			Y: math.Nextafter(lo.Y-bound, math.Inf(-1)),
			Z: math.Nextafter(lo.Z-bound, math.Inf(-1)),
		}, hi: r3.Vec{
			X: math.Nextafter(hi.X+bound, math.Inf(1)),
			Y: math.Nextafter(hi.Y+bound, math.Inf(1)),
			Z: math.Nextafter(hi.Z+bound, math.Inf(1)),
		}}
	}
	for i := range boxes {
		for j := i + 1; j < len(boxes); j++ {
			a, b := boxes[i], boxes[j]
			if !(a.hi.X < b.lo.X || b.hi.X < a.lo.X ||
				a.hi.Y < b.lo.Y || b.hi.Y < a.lo.Y ||
				a.hi.Z < b.lo.Z || b.hi.Z < a.lo.Z) {
				return false
			}
		}
	}
	return true
}
func buildBevelPatternJoin(ctx context.Context, d *Document, ref producerID,
	blankBody *Body, blank *bevelBlankSource, record *pointConeTrimRecord,
	boundary *bevelToothBoundary, teeth []facetedPayload) (*Body, error) {
	const ringStart = -math.Pi / 8
	motions := make([]r3.Transform, len(teeth))
	charges := make([]massmoment.MapCharge, len(teeth))
	plans := make([]bevelTwoPlan, len(teeth))
	for i := range plans {
		motions[i] = r3.Identity()
		if i != 0 {
			motions[i] = teeth[i].bevelTooth.motion
			rotation, err := massmoment.PlacementRotation(motions[i])
			if err != nil {
				return nil, err
			}
			charges[i], err = massmoment.MapChargeOf(rotation)
			if err != nil {
				return nil, err
			}
		}
		plan, err := makeBevelTwoPlan(ctx, blank, record, boundary,
			motions[i], charges[i], ringStart)
		if err != nil {
			return nil, err
		}
		plans[i] = plan
	}
	minRadius := min(blank.toeRoot.V, blank.ded.V)
	for i := range plans {
		for j := i + 1; j < len(plans); j++ {
			seamCost := proofbound.AbsSumUpper(plans[i].crossingBound,
				plans[j].crossingBound,
				bevelPatternMeshBound(teeth[i].meshBound, charges[i]),
				bevelPatternMeshBound(teeth[j].meshBound, charges[j]))
			if !(minRadius > 0 && seamCost < minRadius/2) {
				return nil, fmt.Errorf("%w: patterned root sectors have no angular error bound", ErrUnsupported)
			}
			// For d/r < 1/2, 2d/r exceeds asin(d/r), the largest
			// angular change allowed by a point's spatial displacement.
			angleCost := proofbound.DivUpper(proofbound.ProductUpper(2, seamCost), minRadius)
			if !(bevelPatternAngularGap(plans[i], plans[j]) > angleCost) {
				return nil, fmt.Errorf("%w: patterned root sectors are not separate", ErrUnsupported)
			}
		}
	}
	regular := make([]float64, 0, 256+2*len(plans))
	for i := range 256 {
		regular = append(regular, ringStart+math.Pi*float64(i)/128)
	}
	for _, plan := range plans {
		regular = append(regular, plan.lo, plan.hi)
	}
	// The placed root stations differ by a few ulps from the regular grid.
	// Inside either covered sector the cap owns every split of the blank ring;
	// adding an independent regular station there would leave a tiny free edge.
	outside := regular[:0]
	for _, angle := range regular {
		covered := false
		for _, plan := range plans {
			covered = covered || plan.lo < angle && angle < plan.hi
		}
		if covered {
			continue
		}
		outside = append(outside, angle)
	}
	regular = outside
	angles := regular
	for i := range plans {
		var err error
		angles, _, err = bevelMergedAngles(angles, plans[i].placed)
		if err != nil {
			return nil, err
		}
	}
	for i := range plans {
		var err error
		_, plans[i].rootNode, err = bevelMergedAngles(angles, plans[i].placed)
		if err != nil {
			return nil, err
		}
	}
	blankGroups, err := bevelBlankGroups(blankBody, blank)
	if err != nil {
		return nil, err
	}
	mesh := &bevelJoinedMesh{groups: append([]facetGroup(nil), blankGroups[:]...)}
	seams := make([][2]float64, len(plans))
	for i, plan := range plans {
		seams[i] = [2]float64{plan.lo, plan.hi}
	}
	ring, blankSag, err := bevelBuildBlankMesh(mesh, blank, angles, seams)
	if err != nil {
		return nil, err
	}
	localBound, localRound, sourceVolume, sourceArea := 0.0, 0.0, 0.0, 0.0
	for i := range plans {
		p := &plans[i]
		groups := []facetGroup(nil)
		if i != 0 {
			groups = teeth[i].groups
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
			proofbound.ProductUpper(proofbound.AbsSumUpper(
				bevelPatternMeshBound(record.base.meshBound, charges[i]),
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
		if i != 0 {
			// An exact affine placement scales symmetric-volume error by its
			// exact determinant, even when r3 accepted it as a rigid rotation.
			volumePart = proofbound.ProductUpper(
				proofbound.AbsSumUpper(1, charges[i].Volume), volumePart)
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
		if i != 0 {
			areaPart = proofbound.ProductUpper(
				proofbound.AbsSumUpper(1, charges[i].Stretch), areaPart)
		}
		sourceArea = proofbound.AbsSumUpper(sourceArea, areaPart)
	}
	bound := proofbound.AbsSumUpper(blankSag, localBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: patterned teeth cannot meet the 0.1 mm boundary proof", ErrUnsupported)
	}
	if err := bevelOrientMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBevelPatternJoin(
		proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris, len(teeth)); err != nil {
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
		return nil, fmt.Errorf("%w: patterned teeth have no finite measure proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	pp := facetedPayload{verts: mesh.verts, tris: mesh.tris, src: mesh.src,
		groups: mesh.groups, vertexBound: vertexBound, contactAudited: true,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: record.base.dPair, xform: r3.Identity(),
		bevelJoin: &bevelJoinRecord{blank: blankBody, first: teeth[0],
			placed: append([]facetedPayload(nil), teeth[1:]...)}}
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
		return nil, fmt.Errorf("%w: patterned teeth have no certified occupied volume", ErrUnsupported)
	}
	return body, nil
}
