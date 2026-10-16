package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// pointSourceCurrent checks the source identity held when LoftFromPoint
// recorded the profile. Certificate getters are checked separately at use.
func pointSourceCurrent(record *pointSectionRecord) bool {
	if record == nil || record.Source == nil || record.SourceBox != record.SourceBoxIssued ||
		!slices.Equal(record.SourceStation, record.SourceStationIssued) ||
		proofbound.IsNonFinite(record.SourceStationRound) ||
		record.SourceStationRound < 0 ||
		record.SourceStationRound != record.SourceRoundIssued {
		return false
	}
	src := record.Source
	if src.Sketch == nil || src.Profile == nil || src.Profile.Sketch() != src.Sketch ||
		src.Profile.IsStale() || src.Sketch.Revision() != src.Revision ||
		src.Profile.Revision() != src.Revision ||
		len(src.Entities) != len(record.Profile.Outer.Segments) ||
		len(src.Profile.Outer) != len(src.Entities) {
		return false
	}
	if _, _, _, err := sectioncert.RecordProfile(src.Sketch, src.Profile); err != nil {
		return false
	}
	frame, err := src.Sketch.Plane().Frame()
	if err != nil || frame.Origin() != src.Frame.Origin() ||
		frame.U() != src.Frame.U() || frame.V() != src.Frame.V() {
		return false
	}
	for i, entity := range src.Entities {
		if entity == nil || src.Profile.Outer[i].Entity != entity {
			return false
		}
	}
	return true
}

// LoftFromPoint builds a solid from one exact world point to a closed Sketch
// profile. The apex is in millimetres. This operation accepts one outer loop
// and no holes; every fitted flank remains the recorded Sketch curve. The
// boundary is a certified chorded fan and a cap (docs/loft-point-design.md).
func (d *Document) LoftFromPoint(ctx context.Context, apex r3.Vec,
	s *sketch.Sketch, p *sketch.Profile) (*Body, error) {
	if ctx == nil || d == nil || s == nil || p == nil {
		return nil, fmt.Errorf("%w: point loft needs a context, document, sketch and profile", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if !proofbound.FiniteVec(apex) {
		return nil, fmt.Errorf("%w: the point loft apex must be finite", ErrNotFinite)
	}
	profile, plane, sketchArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	if len(profile.Holes) != 0 {
		return nil, fmt.Errorf("%w: a point loft cannot cone hole loops to one manifold apex", ErrUnsupported)
	}
	// The exact dyadic determinant is the plane-side existence gate. A float
	// dot product can round a shallow but valid point/plane separation to 0.
	separation := proofarith.XdotRat(
		proofarith.Xsub(proofarith.XptOf(apex), proofarith.XptOf(plane.Origin)),
		proofarith.Xcross(proofarith.XptOf(plane.U), proofarith.XptOf(plane.V)),
	)
	if separation.Sign() == 0 {
		return nil, fmt.Errorf("%w: the point loft apex lies in the profile plane", ErrDegenerate)
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf("%w: the profile plane has no frame: %s", ErrDegenerate, err)
	}
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	segmentCount := uint64(len(profile.Outer.Segments))
	limit := loftmesh.StationWorkLimit(0, segmentCount)
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	area, err := falsifyRecordedArea(profile, sketchArea, work0)
	if err != nil {
		return nil, err
	}
	source := &pointSectionSource{
		Sketch: s, Profile: p, Revision: s.Revision(), Frame: frame,
		Entities: make([]sketch.Entity, len(p.Outer)),
	}
	for i, edge := range p.Outer {
		source.Entities[i] = edge.Entity
	}
	record := pointSectionRecord{Apex: apex, Profile: profile, Plane: plane, Source: source}
	ref := d.nextProducerID()
	body, err := evalPointSectionLoft(ctx, d, ref, record, frame, area, work0, work1)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// evalPointSectionLoft reuses the certified two-sided station generator with
// the same authenticated record on both sides. Only its station proof is
// reused: this evaluator builds one apex, one far ring, and one cap.
func evalPointSectionLoft(ctx context.Context, d *Document, ref producerID,
	record pointSectionRecord, frame r3.Frame, area float64,
	work0, work1 *freeform.FreeformWork) (*Body, error) {
	walkSet, err := momentinput.ResolveProfileWalks(record.Profile, work0)
	if err != nil {
		return nil, err
	}
	walks := [][]survey2d.SegmentWalk{walkSet.Outer}
	offsets := []int{0}
	target, err := loftmesh.StationCapGate(record.Profile, record.Profile,
		[2]float64{area, area}, offsets, walks, walks)
	if err != nil {
		return nil, err
	}
	pairs, sectionDelta, matchedDelta, stationRound, err := loftmesh.PairRecords(
		record.Profile, record.Profile, offsets, walks, walks, target, work0, work1)
	if err != nil {
		return nil, err
	}
	if len(pairs) != 1 || len(pairs[0].W) < 3 {
		return nil, fmt.Errorf("%w: a point loft needs one closed ring of at least three stations", ErrDegenerate)
	}
	pair := pairs[0]
	if len(pair.W) > loftmesh.StationCapCeiling {
		return nil, fmt.Errorf("%w: a point loft exceeds its station cap", ErrUnsupported)
	}
	idx := make([]int, len(pair.W))
	for i := range idx {
		idx[i] = i
	}
	clearance := make([]float64, len(pair.MatchedDelta))
	for i, gap := range pair.MatchedDelta {
		clearance[i] = proofbound.AbsSumUpper(gap, stationRound)
	}
	if failure, failed, err := tessellation.SectionWalkClearance(ctx, pair.W,
		[][]int{idx}, [][]float64{clearance}); err != nil {
		return nil, err
	} else if failed {
		return nil, fmt.Errorf("%w: point loft chords %d and %d can meet their source curves",
			ErrUnsupported, failure.ChordA, failure.ChordB)
	}
	capTriangles, err := triangulation.Triangulate(ctx, pair.W, [][]int{idx})
	if err != nil {
		return nil, triangulation.WrapLoftError(err)
	}
	verts := make([]r3.Vec, len(pair.W)+1)
	verts[0] = record.Apex
	maxLocal := 0.0
	for j, pt := range pair.W {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		maxLocal = max(maxLocal, math.Abs(pt.U), math.Abs(pt.V))
		verts[j+1] = frame.ToWorldUV(pt.U, pt.V)
		if !proofbound.FiniteVec(verts[j+1]) {
			return nil, loftmesh.ErrPointUnrepresentable("far section vertex")
		}
	}
	tris := make([][3]int, 0, len(pair.W)+len(capTriangles))
	src := make([]int, 0, len(pair.W)+len(capTriangles))
	groups := make([]facetGroup, len(record.Profile.Outer.Segments)+1)
	for j, seg := range record.Profile.Outer.Segments {
		_, planar := seg.(lineSeg)
		groups[j] = facetGroup{Origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("side(0,%d)", j)}}, Planar: planar}
	}
	capGroup := len(groups) - 1
	groups[capGroup] = facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}}, Planar: true}
	if center, proved := pointFarCapCentroid(record.Profile, record.Plane); proved {
		groups[capGroup].Centroid, groups[capGroup].HasCentroid = vecMeasurementToInternal(center), true
	}
	for j := range pair.W {
		jn := (j + 1) % len(pair.W)
		tris = append(tris, [3]int{0, jn + 1, j + 1})
		src = append(src, pair.Segment[j])
	}
	for _, tri := range capTriangles {
		tris = append(tris, [3]int{tri[0] + 1, tri[1] + 1, tri[2] + 1})
		src = append(src, capGroup)
	}
	sign := tessellation.OrientationSign(verts, tris, record.Apex)
	if sign == 0 {
		return nil, fmt.Errorf("%w: the point loft encloses no volume", ErrDegenerate)
	}
	if sign < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	if layout, ok := pointLoftCapLayout(tris, len(pair.W)); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, verts, tris, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris)
	}
	if err != nil {
		return nil, err
	}
	// Match the far chord to its source curve in the section plane, then
	// charge the frame lift separately. The apex is a recorded world point.
	sectionGap := proofbound.AbsSumUpper(sectionDelta, stationRound, matchedDelta)
	record.SourceStationRound = stationRound
	basisU := proofbound.DvLenUpper(proofbound.HeldDelta(record.Plane.U, r3.Vec{}))
	basisV := proofbound.DvLenUpper(proofbound.HeldDelta(record.Plane.V, r3.Vec{}))
	basisUpper := proofbound.AbsSumUpper(basisU, basisV)
	record.SourceArcUpper = make([]float64, len(pair.ArcUpperW))
	record.SourceMatched = make([]float64, len(pair.MatchedDelta))
	record.SourceStation = make([]planePoint, len(pair.W))
	for j := range pair.ArcUpperW {
		record.SourceStation[j] = planePoint{U: pair.W[j].U, V: pair.W[j].V}
		record.SourceArcUpper[j] = proofbound.ProductUpper(pair.ArcUpperW[j], basisUpper)
		record.SourceMatched[j] = proofbound.ProductUpper(
			proofbound.AbsSumUpper(pair.MatchedDelta[j], stationRound, sectionDelta), basisUpper)
	}
	record.SourceStationIssued = slices.Clone(record.SourceStation)
	record.SourceRoundIssued = record.SourceStationRound
	record.SourceBox = [4]float64{math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)}
	for _, pt := range pair.W {
		record.SourceBox[0] = min(record.SourceBox[0], pt.U)
		record.SourceBox[1] = max(record.SourceBox[1], pt.U)
		record.SourceBox[2] = min(record.SourceBox[2], pt.V)
		record.SourceBox[3] = max(record.SourceBox[3], pt.V)
	}
	for _, i := range []int{0, 2} {
		record.SourceBox[i] = math.Nextafter(record.SourceBox[i]-sectionGap, math.Inf(-1))
		record.SourceBox[i+1] = math.Nextafter(record.SourceBox[i+1]+sectionGap, math.Inf(1))
	}
	record.SourceBoxIssued = record.SourceBox
	liftGap := loftmesh.FrameLiftRoundAllow(frame, maxLocal)
	meshBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(sectionGap, proofbound.AbsSumUpper(basisU, basisV)), liftGap)
	if proofbound.IsNonFinite(meshBound) {
		return nil, fmt.Errorf("%w: the point loft has no finite boundary bound", ErrUnsupported)
	}
	vertexBound := make([]float64, len(verts))
	for j := 1; j < len(vertexBound); j++ {
		vertexBound[j] = meshBound
	}
	perimeter := loftmesh.PerimeterUpper(record.Profile, walks)
	sectionArea := proofbound.SectionDisplacementArea(sectionGap, len(pair.W), perimeter)
	record.SourceAreaDelta = sectionArea
	jacobian := proofbound.CrossProductUpper(record.Plane.U, record.Plane.V)
	capArea := proofbound.ProductUpper(sectionArea, jacobian)
	height := proofbound.DvLenUpper(proofbound.HeldDelta(record.Apex, record.Plane.Origin))
	conedGap := proofbound.ProductUpper(proofbound.ProductUpper(capArea, height),
		math.Nextafter(1.0/3.0, math.Inf(1)))
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, liftGap)
	if err != nil {
		return nil, err
	}
	volumeGap := proofbound.AbsSumUpper(conedGap, proofbound.SweptVolumeAllow(liftGap, areaUpper))
	wallAreaGap := 0.0
	for j := range pair.W {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		jn := (j + 1) % len(pair.W)
		d0 := proofbound.DvLenUpper(proofbound.HeldDelta(verts[j+1], record.Apex))
		d1 := proofbound.DvLenUpper(proofbound.HeldDelta(verts[jn+1], record.Apex))
		reach := proofbound.AbsSumUpper(max(d0, d1), meshBound)
		trueUpper := proofbound.ProductUpper(proofbound.ProductUpper(reach, pair.ArcUpperW[j]), 0.5)
		heldUpper := proofbound.PerturbedTriangleAreaUpper(verts[0], verts[jn+1], verts[j+1], 0)
		wallAreaGap = proofbound.AbsSumUpper(wallAreaGap, trueUpper, heldUpper)
	}
	areaGap := proofbound.AbsSumUpper(wallAreaGap, capArea, areaUpper)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaGap) {
		return nil, fmt.Errorf("%w: the point loft has no finite volume or area proof", ErrUnsupported)
	}
	maxFar := 0.0
	for _, v := range verts[1:] {
		maxFar = max(maxFar, proofbound.DvLenUpper(proofbound.HeldDelta(v, record.Apex)))
	}
	diameter := proofbound.AbsSumUpper(maxFar, proofbound.ProductUpper(perimeter,
		max(basisU, basisV)), meshBound)
	pp := facetedPayload{
		Verts: verts, Tris: tris, Src: src, Groups: groups, VertexBound: vertexBound,
		MeshBound: meshBound, VolSymDiff: volumeGap, AreaSlack: areaGap,
		DPair: diameter, Xform: r3.Identity(),
		PointSection: &record, PointPlaneSourceEligible: true,
	}
	return bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), pp))
}

// pointLoftCapLayout is a point loft's wall and cap structure as
// buildPointLoft assembles it: sides apex fan triangles (0, b, c), then the
// far cap's triangles over the ring vertices 1..sides.
func pointLoftCapLayout(tris [][3]int, sides int) (loftmesh.LoftCapLayout, bool) {
	if sides > len(tris) {
		return loftmesh.LoftCapLayout{}, false
	}
	ring, ok := loftmesh.FanRing(tris[:sides])
	if !ok {
		return loftmesh.LoftCapLayout{}, false
	}
	loop := make([]int, len(ring))
	for i, r := range ring {
		loop[i] = r + 1
	}
	layout := loftmesh.LoftCapLayout{Walls: sides, Loops0: [][]int{loop}, Loops1: [][]int{{0}}}
	for t := sides; t < len(tris); t++ {
		layout.Cap0 = append(layout.Cap0, t)
	}
	return layout, true
}
