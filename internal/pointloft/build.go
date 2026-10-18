package pointloft

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Build reuses the certified two-sided station generator with
// the same authenticated record on both sides. Only its station proof is
// reused: this evaluator builds one apex, one far ring, and one cap.
func Build(ctx context.Context, record facetedrecord.PointSectionRecord, frame r3.Frame,
	area float64, work0, work1 *freeform.FreeformWork, ref provenance.Producer) (facetedrecord.Payload, error) {
	walkSet, err := momentinput.ResolveProfileWalks(record.Profile, work0)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	walks := [][]survey2d.SegmentWalk{walkSet.Outer}
	offsets := []int{0}
	target, err := loftmesh.StationCapGate(record.Profile, record.Profile,
		[2]float64{area, area}, offsets, walks, walks)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	pairs, sectionDelta, matchedDelta, stationRound, err := loftmesh.PairRecords(
		record.Profile, record.Profile, offsets, walks, walks, target, work0, work1)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	if len(pairs) != 1 || len(pairs[0].W) < 3 {
		return facetedrecord.Payload{}, fmt.Errorf("%w: a point loft needs one closed ring of at least three stations", decaderr.ErrDegenerate)
	}
	pair := pairs[0]
	if len(pair.W) > loftmesh.StationCapCeiling {
		return facetedrecord.Payload{}, fmt.Errorf("%w: a point loft exceeds its station cap", decaderr.ErrUnsupported)
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
		return facetedrecord.Payload{}, err
	} else if failed {
		return facetedrecord.Payload{}, fmt.Errorf("%w: point loft chords %d and %d can meet their source curves",
			decaderr.ErrUnsupported, failure.ChordA, failure.ChordB)
	}
	capTriangles, err := triangulation.Triangulate(ctx, pair.W, [][]int{idx})
	if err != nil {
		return facetedrecord.Payload{}, triangulation.WrapLoftError(err)
	}
	verts := make([]r3.Vec, len(pair.W)+1)
	verts[0] = record.Apex
	maxLocal := 0.0
	for j, pt := range pair.W {
		if err := ctx.Err(); err != nil {
			return facetedrecord.Payload{}, err
		}
		maxLocal = max(maxLocal, math.Abs(pt.U), math.Abs(pt.V))
		verts[j+1] = frame.ToWorldUV(pt.U, pt.V)
		if !proofbound.FiniteVec(verts[j+1]) {
			return facetedrecord.Payload{}, loftmesh.ErrPointUnrepresentable("far section vertex")
		}
	}
	tris := make([][3]int, 0, len(pair.W)+len(capTriangles))
	src := make([]int, 0, len(pair.W)+len(capTriangles))
	groups := make([]facetedrecord.Group, len(record.Profile.Outer.Segments)+1)
	for j, seg := range record.Profile.Outer.Segments {
		_, planar := seg.(sectionrecord.LineSeg)
		groups[j] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("side(0,%d)", j)}}, Planar: planar}
	}
	capGroup := len(groups) - 1
	groups[capGroup] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}}, Planar: true}
	if center, proved := facetedrecord.PointFarCapCentroid(record.Profile, record.Plane); proved {
		groups[capGroup].Centroid, groups[capGroup].HasCentroid = center, true
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
		return facetedrecord.Payload{}, fmt.Errorf("%w: the point loft encloses no volume", decaderr.ErrDegenerate)
	}
	if sign < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	if layout, ok := CapLayout(tris, len(pair.W)); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, verts, tris, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris)
	}
	if err != nil {
		return facetedrecord.Payload{}, err
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
	record.SourceStation = make([]sectionrecord.Point2, len(pair.W))
	for j := range pair.ArcUpperW {
		record.SourceStation[j] = sectionrecord.Point2{U: pair.W[j].U, V: pair.W[j].V}
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
		return facetedrecord.Payload{}, fmt.Errorf("%w: the point loft has no finite boundary bound", decaderr.ErrUnsupported)
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
		return facetedrecord.Payload{}, err
	}
	volumeGap := proofbound.AbsSumUpper(conedGap, proofbound.SweptVolumeAllow(liftGap, areaUpper))
	wallAreaGap := 0.0
	for j := range pair.W {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return facetedrecord.Payload{}, err
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
		return facetedrecord.Payload{}, fmt.Errorf("%w: the point loft has no finite volume or area proof", decaderr.ErrUnsupported)
	}
	maxFar := 0.0
	for _, v := range verts[1:] {
		maxFar = max(maxFar, proofbound.DvLenUpper(proofbound.HeldDelta(v, record.Apex)))
	}
	diameter := proofbound.AbsSumUpper(maxFar, proofbound.ProductUpper(perimeter,
		max(basisU, basisV)), meshBound)
	pp := facetedrecord.Payload{
		Verts: verts, Tris: tris, Src: src, Groups: groups, VertexBound: vertexBound,
		MeshBound: meshBound, VolSymDiff: volumeGap, AreaSlack: areaGap,
		DPair: diameter, Xform: r3.Identity(),
		PointSection: &record, PointPlaneSourceEligible: true,
	}
	return pp, nil
}

// CapLayout is a point loft's wall and cap structure as
// Build assembles it: sides apex fan triangles (0, b, c), then the
// far cap's triangles over the ring vertices 1..sides.
func CapLayout(tris [][3]int, sides int) (loftmesh.LoftCapLayout, bool) {
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
