package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// A mirrored cylinder Join rebuilds the lower source against one exact half
// of the original blank. The upper source is its authenticated reflection.
// The original Boolean operands remain recorded for each later pattern step.
type loftMirrorCylinderJoinRecord struct {
	Blank        *topo.Body
	Source       *loftMirrorJoinRecord
	Pairs        []*topo.Body
	LowerHalf    *topo.Body
	CapGroups    [2]int
	CapOutlines  [2][]int
	BridgeBound  float64
	BridgeVolume float64
}

func tryLoftMirrorCylinderUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *Body
	var pairs []*Body
	var source *loftMirrorJoinRecord
	if pp, ok := a.n.Payload.(prismPayload); ok && loftMirrorCylinderBlank(pp) {
		blank, pairs = a, []*Body{b}
	} else if pp, ok := b.n.Payload.(prismPayload); ok && loftMirrorCylinderBlank(pp) {
		blank, pairs = b, []*Body{a}
	} else if fp, ok := a.n.Payload.(facetedPayload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftMirrorCylinderJoin.Blank)
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(append([]*Body(nil), bodyHandles(fp.LoftMirrorCylinderJoin.Pairs)...), b)
	} else if fp, ok := b.n.Payload.(facetedPayload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftMirrorCylinderJoin.Blank)
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(append([]*Body(nil), bodyHandles(fp.LoftMirrorCylinderJoin.Pairs)...), a)
	} else {
		return nil, false, nil
	}
	return tryLoftMirrorCylinderPairs(ctx, d, ref, blank, pairs, source)
}

func tryLoftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord) (*Body, bool, error) {
	return loftMirrorCylinderPairs(ctx, d, ref, blank, pairs, source, orderedwork.MaxEachWorkers)
}

// loftMirrorCylinderPairs is tryLoftMirrorCylinderPairs with the cap on the
// goroutines that place the lower teeth. The placements are independent, so
// the cap changes the wall time and nothing else.
func loftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord, placeWorkers int) (*Body, bool, error) {
	pp, blankOK := blank.n.Payload.(prismPayload)
	if !blankOK || !loftMirrorCylinderBlank(pp) || len(pairs) == 0 || len(pairs) > 32 {
		return nil, false, nil
	}
	first, firstOK := pairs[0].n.Payload.(facetedPayload)
	if !firstOK || first.LoftMirrorJoin == nil || first.Xform != r3.Identity() ||
		!first.ContactAudited || source != nil && source != first.LoftMirrorJoin {
		return nil, false, nil
	}
	record := first.LoftMirrorJoin
	if record.Lower == nil || record.Upper == nil || record.StationCount < 3 ||
		record.Lower.Doc != documentNode(d) || record.Upper.Doc != documentNode(d) {
		return nil, false, nil
	}
	lower, lowOK := record.Lower.Payload.(loftPayload)
	upper, upOK := record.Upper.Payload.(loftPayload)
	if !lowOK || !upOK || !lower.AuthenticatedReconstruction ||
		!upper.AuthenticatedReconstruction || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity ||
		lower.Xform != r3.Identity() || !loftMirrorAxialSource(lower) {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: mirrored cylinder source has no reflection: %s", ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	z0, middle, z2 := pp.Z0, lower.Frame1.Origin().Z, pp.Z1
	if lower.Frame0.Origin().Z != z0 || !(z0 < middle && middle < z2) ||
		!loftMirrorCylinderLevels(z0, middle, z2) {
		return nil, false, nil
	}
	// The first-stage source relation proves upper=reflection(lower). The
	// blank has the same circle and exact caps over both half intervals.
	half := pp
	half.Z1 = middle
	halfBlank, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), half, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	checked := loftMirrorCylinderCheckPairs(d, pairs, record, middle)
	workers := min(placeWorkers, orderedwork.EachWorkers(checked.stop, 2))
	lowerTeeth, err := loftMirrorCylinderPlaceTeeth(checked.stop, workers, func(i int) (*Body, error) {
		if checked.motions[i] == r3.Identity() {
			return bodyHandle(record.Lower), nil
		}
		return placePayload(ctx, lower, d, ref, checked.motions[i])
	})
	if err != nil {
		return nil, true, err
	}
	if checked.stop < len(pairs) {
		return nil, checked.admit, checked.err
	}
	lowerJoin, admitted, err := tryLoftCylinderUnionTeethWithUpperBridge(
		ctx, d, ref, halfBlank, lowerTeeth, checked.upperBridges)
	if err != nil {
		return nil, true, err
	}
	if !admitted {
		return nil, true, fmt.Errorf("%w: mirrored cylinder's lower slab lacks a certified circular Join", ErrUnsupported)
	}
	result, err := buildLoftMirrorCylinderUnion(ctx, d, ref, blank, pairs, record,
		lowerJoin, reflection, checked.bridgeBound, checked.bridgeVolume)
	return result, true, err
}

// loftMirrorPairChecks is the serial checking pass over a Union's pairs.
// Pairs [0, stop) passed every check; stop == len(pairs) means all did.
// Otherwise the pair at stop ends the Union with (nil, admit, err) once the
// placements below it have succeeded.
type loftMirrorPairChecks struct {
	motions      []r3.Transform
	upperBridges []float64
	bridgeBound  float64
	bridgeVolume float64
	stop         int
	admit        bool
	err          error
}

// loftMirrorCylinderCheckPairs runs, in pair order, every check the Union makes
// on a pair except its placement, and stops at the first pair that fails one.
func loftMirrorCylinderCheckPairs(d *Document, pairs []*Body, record *loftMirrorJoinRecord,
	middle float64) loftMirrorPairChecks {
	c := loftMirrorPairChecks{
		motions:      make([]r3.Transform, len(pairs)),
		upperBridges: make([]float64, len(pairs)),
		stop:         len(pairs),
	}
	refuse := func(i int, admit bool, err error) loftMirrorPairChecks {
		c.stop, c.admit, c.err = i, admit, err
		return c
	}
	for i, pair := range pairs {
		payload, ok := pair.n.Payload.(facetedPayload)
		if !ok || !payload.ContactAudited || pair.n.Doc != documentNode(d) {
			return refuse(i, false, nil)
		}
		motion, ok := loftMirrorCylinderPairMotion(payload, record)
		if !ok {
			return refuse(i, true, fmt.Errorf(
				"%w: mirrored cylinder pair %d has no authenticated axial placement", ErrUnsupported, i+1))
		}
		if !loftCylinderAxialPlacement(motion) {
			shift, swept, ok, err := loftMirrorCylinderUpperBridge(pair, motion, middle)
			if err != nil {
				return refuse(i, true, err)
			}
			if !ok {
				return refuse(i, true, fmt.Errorf(
					"%w: mirrored cylinder pair %d has no upper axial bridge", ErrUnsupported, i+1))
			}
			c.bridgeBound = max(c.bridgeBound, shift)
			c.bridgeVolume = proofbound.AbsSumUpper(c.bridgeVolume, swept)
			c.upperBridges[i] = shift
		}
		c.motions[i] = motion
	}
	return c
}

// loftMirrorCylinderPlaceTeeth runs place(i) for every i in [0, n) on up to
// workers goroutines and returns the bodies in index order. On failure it
// returns the error of the lowest failing index, the one a loop over [0, n)
// meets first; every lower index has run. place must write nothing another
// index reads.
func loftMirrorCylinderPlaceTeeth(n, workers int, place func(i int) (*Body, error)) ([]*Body, error) {
	teeth := make([]*Body, n)
	if _, err := orderedwork.Each(n, workers, 1, nil, func(i int) error {
		tooth, err := place(i)
		if err != nil {
			return err
		}
		teeth[i] = tooth
		return nil
	}); err != nil {
		return nil, err
	}
	return teeth, nil
}

func loftMirrorCylinderPairMotion(fp facetedPayload, source *loftMirrorJoinRecord) (r3.Transform, bool) {
	if fp.LoftMirrorJoin == source && fp.Xform == r3.Identity() {
		return r3.Identity(), true
	}
	if fp.LoftMirrorCopy == nil || fp.LoftMirrorCopy.Source != source ||
		fp.Xform != fp.LoftMirrorCopy.Motion ||
		!loftCylinderAxialPlacement(fp.LoftMirrorCopy.Motion) &&
			!loftmesh.NearAxialPlacement(fp.LoftMirrorCopy.Motion) {
		return r3.Transform{}, false
	}
	return fp.LoftMirrorCopy.Motion, true
}

// For a lower source L reflected about z=middle, the actual upper copy is
// T(F(L)). The reflected lower-half Join represents F(T(L)); the pair's
// certified area bounds the upper source area along the constant translation
// between them (loftmesh.MirrorUpperBridgeShift). Held placement roundoff
// remains in the lower Join.
func loftMirrorCylinderUpperBridge(pair *Body, motion r3.Transform,
	middle float64) (float64, float64, bool, error) {
	shift, ok := loftmesh.MirrorUpperBridgeShift(motion, middle)
	if !ok {
		return 0, 0, false, nil
	}
	area, err := pair.Area()
	if err != nil {
		return 0, 0, true, err
	}
	swept, ok := loftmesh.MirrorUpperBridgeSwept(shift, area.Value.Base(), area.Bound.Base())
	if !ok {
		return 0, 0, false, nil
	}
	return shift, swept, true, nil
}

func loftMirrorCylinderBlank(pp prismPayload) bool {
	if pp.SurfaceResult || pp.Xform != r3.Identity() || pp.Z0Delta != 0 ||
		pp.Z1Delta != 0 || pp.SectionDelta != 0 || pp.Z1 <= pp.Z0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.Frame.Origin() != (r3.Vec{}) || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) {
		return false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	return ok && circle.CCW && circle.Center == (planePoint{}) &&
		circle.Radius.Base() > 0 && !proofbound.IsNonFinite(circle.Radius.Base())
}

func loftMirrorCylinderLevels(z0, middle, z2 float64) bool {
	if proofbound.IsNonFinite(z0) || proofbound.IsNonFinite(middle) || proofbound.IsNonFinite(z2) {
		return false
	}
	// Exact dyadic comparison ensures the reflected lower cap lands on the
	// original cylinder's upper cap, not merely within a float tolerance.
	twiceMiddle := new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(middle))
	sumCaps := new(big.Rat).Add(proofarith.FloatRat(z0), proofarith.FloatRat(z2))
	return twiceMiddle.Cmp(sumCaps) == 0
}

func buildLoftMirrorCylinderUnion(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord,
	lowerHalf *Body, reflection r3.Transform, bridgeBound, bridgeVolume float64) (*Body, error) {
	lower, ok := lowerHalf.n.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderJoin == nil || !lower.ContactAudited || lower.Xform != r3.Identity() {
		return nil, fmt.Errorf("%w: mirrored cylinder's lower slab has no audited source", ErrUnsupported)
	}
	upperHalf, err := placePayload(ctx, lower, d, ref, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperHalf.n.Payload.(facetedPayload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) {
		return nil, fmt.Errorf("%w: reflected cylinder slab has no matching held mesh", ErrUnsupported)
	}
	lowCaps := lower.LoftCylinderJoin.CapGroups
	if lowCaps[0] < 0 || lowCaps[1] < 0 || lowCaps[0] == lowCaps[1] {
		return nil, fmt.Errorf("%w: lower circular Join lacks separate cap groups", ErrUnsupported)
	}
	// Reflection preserves the upper outline's XY coordinates. Every middle
	// vertex of the reflected cap must match the original held vertex exactly.
	middle := lower.LoftCylinderJoin.Z1
	midVertices := make(map[r3.Vec]int)
	for i, v := range lower.Verts {
		if v.Z == middle {
			if _, exists := midVertices[v]; exists {
				return nil, fmt.Errorf("%w: lower circular Join duplicates a middle vertex", ErrUnsupported)
			}
			midVertices[v] = i
		}
	}
	if len(midVertices) < 3 {
		return nil, fmt.Errorf("%w: circular Join has no complete middle outline", ErrUnsupported)
	}
	vertexMap := make([]int, len(upper.Verts))
	verts := append([]r3.Vec(nil), lower.Verts...)
	vertexBound := append([]float64(nil), lower.VertexBound...)
	for i, v := range upper.Verts {
		if v.Z == middle {
			index, exists := midVertices[v]
			if !exists {
				return nil, fmt.Errorf("%w: reflected cylinder caps do not match exactly", ErrUnsupported)
			}
			vertexMap[i] = index
			vertexBound[index] = max(vertexBound[index],
				proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
			continue
		}
		if v.Z <= middle {
			return nil, fmt.Errorf("%w: reflected cylinder slab crosses its middle plane", ErrUnsupported)
		}
		vertexMap[i] = len(verts)
		verts = append(verts, v)
		vertexBound = append(vertexBound,
			proofbound.AbsSumUpper(upper.VertexBound[i], bridgeBound))
	}
	if !loftMirrorCylinderCapsOppose(lower, upper, vertexMap, lowCaps[1], lowCaps[1]) {
		return nil, fmt.Errorf("%w: reflected cylinder middle caps do not tile equally", ErrUnsupported)
	}
	var tris [][3]int
	var src []int
	var groups []facetGroup
	capGroups := [2]int{-1, -1}
	for half, payload := range []facetedPayload{lower, upper} {
		// Reflection changes cap orientation and height, not its source
		// group ID. Both copies omit the original top cap group.
		omit, keep := lowCaps[1], lowCaps[0]
		groupMap := make(map[int]int)
		for i, tri := range payload.Tris {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			oldGroup := payload.Src[i]
			if oldGroup == omit {
				continue
			}
			group, exists := groupMap[oldGroup]
			if !exists {
				if oldGroup < 0 || oldGroup >= len(payload.Groups) {
					return nil, fmt.Errorf("%w: circular Join facet names no source group", ErrUnsupported)
				}
				group = len(groups)
				groupMap[oldGroup] = group
				groups = append(groups, payload.Groups[oldGroup])
				if oldGroup == keep {
					capGroups[half] = group
				}
			}
			if half == 1 {
				for corner, index := range tri {
					tri[corner] = vertexMap[index]
				}
			}
			tris = append(tris, tri)
			src = append(src, group)
		}
	}
	if capGroups[0] < 0 || capGroups[1] < 0 {
		return nil, fmt.Errorf("%w: mirrored cylinder has no exterior cap groups", ErrUnsupported)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: mirrored cylinder seam has an invalid vertex link: %s", ErrUnsupported, err)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, err
	}
	bound := max(lower.MeshBound,
		proofbound.AbsSumUpper(upper.MeshBound, bridgeBound))
	volumeGap := proofbound.AbsSumUpper(lower.VolSymDiff, upper.VolSymDiff, bridgeVolume)
	areaSlack := proofbound.AbsSumUpper(lower.AreaSlack, upper.AreaSlack,
		proofbound.ProductUpper(2, max(lower.AreaSlack, upper.AreaSlack)))
	diameter := facetedSourceDiameterUpper(verts, bound)
	if bound > 0.1 || proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(diameter) {
		return nil, fmt.Errorf("%w: mirrored cylinder has no finite 0.1 mm proof", ErrUnsupported)
	}
	lowOutlines := lower.LoftCylinderJoin.CapOutlines
	capOutlines := [2][]int{
		append([]int(nil), lowOutlines[0]...),
		make([]int, len(lowOutlines[0])),
	}
	for i, index := range lowOutlines[0] {
		if index < 0 || index >= len(vertexMap) {
			return nil, fmt.Errorf("%w: mirrored cylinder lost a cap outline vertex", ErrUnsupported)
		}
		capOutlines[1][i] = vertexMap[index]
	}
	fullBlank, ok := blank.n.Payload.(prismPayload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cylinder lost its circular blank", ErrUnsupported)
	}
	shell := &joinedMesh{Verts: verts, Tris: tris, Src: src, Groups: groups}
	for cap, outline := range capOutlines {
		z := fullBlank.Z0
		if cap == 1 {
			z = fullBlank.Z1
		}
		if !loftCylinderAuditCap(ctx, shell, outline, capGroups[cap], z, cap == 0) {
			return nil, fmt.Errorf("%w: mirrored cylinder cap does not tile its recorded outline", ErrUnsupported)
		}
	}
	fp := facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: diameter, ContactAudited: true, Xform: r3.Identity(),
		LoftMirrorCylinderJoin: &loftMirrorCylinderJoinRecord{
			Blank: bodyNode(blank), Source: source, Pairs: bodyNodes(pairs),
			LowerHalf: bodyNode(lowerHalf), CapGroups: capGroups, CapOutlines: capOutlines,
			BridgeBound: bridgeBound, BridgeVolume: bridgeVolume,
		},
	}
	if err := loftMirrorCylinderCapOrigins(&fp, blank, pairs, capGroups); err != nil {
		return nil, err
	}
	if err := loftCylinderTagCaps(&fp, blank, capGroups,
		[2]float64{fullBlank.Z0, fullBlank.Z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&fp, capGroups[1], []float64{bridgeBound})
	return buildFacetedBody(ctx, d, ref, fp)
}

func loftMirrorCylinderCapsOppose(lower, upper facetedPayload, upperMap []int,
	lowerGroup, upperGroup int) bool {
	return facetedtopology.CapsOppose(lower.Tris, lower.Src, upper.Tris, upper.Src,
		upperMap, lowerGroup, upperGroup)
}

func loftMirrorCylinderCapOrigins(fp *facetedPayload, blank *Body, pairs []*Body,
	groups [2]int) error {
	for cap, group := range groups {
		role := provenance.RoleCapStart
		if cap == 1 {
			role = provenance.RoleCapEnd
		}
		seen := make(map[provenance.Ref]struct{})
		var origins []provenance.Ref
		add := func(ref provenance.Ref) {
			if _, exists := seen[ref]; !exists {
				seen[ref] = struct{}{}
				origins = append(origins, ref)
			}
		}
		for _, face := range blank.Faces() {
			for _, origin := range face.originRefs() {
				if origin.Producer == blank.n.Origin.Producer && origin.Role == role {
					for _, source := range face.originRefs() {
						add(source)
					}
				}
			}
		}
		for _, pair := range pairs {
			payload, ok := pair.n.Payload.(facetedPayload)
			if !ok {
				return fmt.Errorf("%w: mirrored tooth lacks a source cap", ErrUnsupported)
			}
			record := payload.LoftMirrorJoin
			if record == nil && payload.LoftMirrorCopy != nil {
				record = payload.LoftMirrorCopy.Source
			}
			if record == nil || record.CapGroups[cap] < 0 ||
				record.CapGroups[cap] >= len(payload.Groups) {
				return fmt.Errorf("%w: mirrored tooth lost a source cap role", ErrUnsupported)
			}
			for _, origin := range payload.Groups[record.CapGroups[cap]].Origins {
				add(origin)
			}
		}
		if len(origins) == 0 {
			return fmt.Errorf("%w: mirrored cylinder cap has no source role", ErrUnsupported)
		}
		fp.Groups[group].Origins = origins
	}
	return nil
}
