package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

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

// pointSectionRecord keeps the authenticated Sketch geometry beside the
// faceted boundary of a point-section loft. The far section is never replaced
// by a fitted or scaled profile.
type pointSectionRecord struct {
	apex                r3.Vec
	profile             profileRecord
	plane               planeRecord
	source              *pointSectionSource
	sourceBox           [4]float64   // U min/max, V min/max; contains the true region and station polygon
	sourceBoxIssued     [4]float64   // original source rectangle at LoftFromPoint publication
	sourceAreaDelta     float64      // area of the symmetric difference between source region and station polygon
	sourceStation       []planePoint // original local station coordinates, in source loop order
	sourceStationIssued []planePoint // station coordinates published with the original point loft
	sourceStationRound  float64      // local error of every held station against its exact curve point
	sourceRoundIssued   float64      // station error published with the original point loft
	sourceArcUpper      []float64    // per-cell source curve speed in world millimetres
	sourceMatched       []float64    // per-cell parameter-matched source departure in world millimetres
}

// pointSectionSource names the live Sketch entities authenticated when the
// point loft was made. A later structural join can request a new certificate
// only from these entities and must refuse if their revision or frame moved.
type pointSectionSource struct {
	sketch   *sketch.Sketch
	profile  *sketch.Profile
	revision uint64
	frame    r3.Frame
	entities []sketch.Entity
}

// pointSourceCurrent checks the source identity held when LoftFromPoint
// recorded the profile. Certificate getters are checked separately at use.
func pointSourceCurrent(record *pointSectionRecord) bool {
	if record == nil || record.source == nil || record.sourceBox != record.sourceBoxIssued ||
		!slices.Equal(record.sourceStation, record.sourceStationIssued) ||
		proofbound.IsNonFinite(record.sourceStationRound) ||
		record.sourceStationRound < 0 ||
		record.sourceStationRound != record.sourceRoundIssued {
		return false
	}
	src := record.source
	if src.sketch == nil || src.profile == nil || src.profile.Sketch() != src.sketch ||
		src.profile.IsStale() || src.sketch.Revision() != src.revision ||
		src.profile.Revision() != src.revision ||
		len(src.entities) != len(record.profile.Outer.Segments) ||
		len(src.profile.Outer) != len(src.entities) {
		return false
	}
	if _, _, _, err := sectioncert.RecordProfile(src.sketch, src.profile); err != nil {
		return false
	}
	frame, err := src.sketch.Plane().Frame()
	if err != nil || frame.Origin() != src.frame.Origin() ||
		frame.U() != src.frame.U() || frame.V() != src.frame.V() {
		return false
	}
	for i, entity := range src.entities {
		if entity == nil || src.profile.Outer[i].Entity != entity {
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
		sketch: s, profile: p, revision: s.Revision(), frame: frame,
		entities: make([]sketch.Entity, len(p.Outer)),
	}
	for i, edge := range p.Outer {
		source.entities[i] = edge.Entity
	}
	record := pointSectionRecord{apex: apex, profile: profile, plane: plane, source: source}
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
	walkSet, err := momentinput.ResolveProfileWalks(record.profile, work0)
	if err != nil {
		return nil, err
	}
	walks := [][]survey2d.SegmentWalk{walkSet.Outer}
	offsets := []int{0}
	target, err := loftmesh.StationCapGate(record.profile, record.profile,
		[2]float64{area, area}, offsets, walks, walks)
	if err != nil {
		return nil, err
	}
	pairs, sectionDelta, matchedDelta, stationRound, err := loftmesh.PairRecords(
		record.profile, record.profile, offsets, walks, walks, target, work0, work1)
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
	verts[0] = record.apex
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
	groups := make([]facetGroup, len(record.profile.Outer.Segments)+1)
	for j, seg := range record.profile.Outer.Segments {
		_, planar := seg.(lineSeg)
		groups[j] = facetGroup{origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("side(0,%d)", j)}}, planar: planar}
	}
	capGroup := len(groups) - 1
	groups[capGroup] = facetGroup{origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}}, planar: true}
	if center, proved := pointFarCapCentroid(record.profile, record.plane); proved {
		groups[capGroup].centroid, groups[capGroup].hasCentroid = center, true
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
	sign := tessellation.OrientationSign(verts, tris, record.apex)
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
	record.sourceStationRound = stationRound
	basisU := proofbound.DvLenUpper(proofbound.HeldDelta(record.plane.U, r3.Vec{}))
	basisV := proofbound.DvLenUpper(proofbound.HeldDelta(record.plane.V, r3.Vec{}))
	basisUpper := proofbound.AbsSumUpper(basisU, basisV)
	record.sourceArcUpper = make([]float64, len(pair.ArcUpperW))
	record.sourceMatched = make([]float64, len(pair.MatchedDelta))
	record.sourceStation = make([]planePoint, len(pair.W))
	for j := range pair.ArcUpperW {
		record.sourceStation[j] = planePoint{U: pair.W[j].U, V: pair.W[j].V}
		record.sourceArcUpper[j] = proofbound.ProductUpper(pair.ArcUpperW[j], basisUpper)
		record.sourceMatched[j] = proofbound.ProductUpper(
			proofbound.AbsSumUpper(pair.MatchedDelta[j], stationRound, sectionDelta), basisUpper)
	}
	record.sourceStationIssued = slices.Clone(record.sourceStation)
	record.sourceRoundIssued = record.sourceStationRound
	record.sourceBox = [4]float64{math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)}
	for _, pt := range pair.W {
		record.sourceBox[0] = min(record.sourceBox[0], pt.U)
		record.sourceBox[1] = max(record.sourceBox[1], pt.U)
		record.sourceBox[2] = min(record.sourceBox[2], pt.V)
		record.sourceBox[3] = max(record.sourceBox[3], pt.V)
	}
	for _, i := range []int{0, 2} {
		record.sourceBox[i] = math.Nextafter(record.sourceBox[i]-sectionGap, math.Inf(-1))
		record.sourceBox[i+1] = math.Nextafter(record.sourceBox[i+1]+sectionGap, math.Inf(1))
	}
	record.sourceBoxIssued = record.sourceBox
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
	perimeter := loftmesh.PerimeterUpper(record.profile, walks)
	sectionArea := proofbound.SectionDisplacementArea(sectionGap, len(pair.W), perimeter)
	record.sourceAreaDelta = sectionArea
	jacobian := proofbound.CrossProductUpper(record.plane.U, record.plane.V)
	capArea := proofbound.ProductUpper(sectionArea, jacobian)
	height := proofbound.DvLenUpper(proofbound.HeldDelta(record.apex, record.plane.Origin))
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
		d0 := proofbound.DvLenUpper(proofbound.HeldDelta(verts[j+1], record.apex))
		d1 := proofbound.DvLenUpper(proofbound.HeldDelta(verts[jn+1], record.apex))
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
		maxFar = max(maxFar, proofbound.DvLenUpper(proofbound.HeldDelta(v, record.apex)))
	}
	diameter := proofbound.AbsSumUpper(maxFar, proofbound.ProductUpper(perimeter,
		max(basisU, basisV)), meshBound)
	pp := facetedPayload{
		Verts: verts, Tris: tris, Src: src, groups: groups, vertexBound: vertexBound,
		meshBound: meshBound, volSymDiff: volumeGap, areaSlack: areaGap,
		dPair: diameter, xform: r3.Identity(),
		pointSection: &record, pointPlaneSourceEligible: true,
	}
	return buildFacetedBody(ctx, d, ref, pp)
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
