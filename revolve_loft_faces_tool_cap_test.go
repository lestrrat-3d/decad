package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolCirclesConsumeLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	require.NotEmpty(t, sides.circles)
	for _, circle := range sides.circles {
		require.Same(t, blank, bodyHandle(circle.blankFace.n.Body))
		require.Same(t, tooth, bodyHandle(circle.capFace.n.Body))
		require.Positive(t, circle.radius.Sign())
		tool := proof.trim.Toe
		if circle.tool == 1 {
			tool = proof.trim.Heel
		}
		require.Zero(t, lfSub(lfAdd(circle.x,
			lfMul(tool.Slope, circle.radius)), tool.Apex).Sign())
	}
	t.Run("changed non-root Revolve surface", func(t *testing.T) {
		for index, span := range proof.section.Spans {
			if index == proof.section.RootSpan || span.Face == nil {
				continue
			}
			cone, ok := span.Face.Surface.(surfacegeom.Cone)
			if !ok {
				continue
			}
			original := span.Face.Surface
			defer func() { span.Face.Surface = original }()
			cone.Origin.X += 0.125
			span.Face.Surface = cone
			_, accepted := revolveLoftFacesToolCapSidesOf(proof)
			require.False(t, accepted)
			return
		}
		t.Fatal("missing non-root cone face")
	})
	changed := *proof
	changed.section = proof.section
	changed.section.Spans = append([]revolveSectionSpan(nil), proof.section.Spans...)
	changed.section.Spans[proof.section.RootSpan].End.V += 0.125
	_, accepted := revolveLoftFacesToolCapSidesOf(&changed)
	require.False(t, accepted)
	changed = *proof
	changed.toeFace = proof.heelFace
	_, accepted = revolveLoftFacesToolCapSidesOf(&changed)
	require.False(t, accepted)
	toeSlope := proof.trim.Toe.Slope
	proof.trim.Toe.Slope = lfAdd(toeSlope, big.NewRat(1, 32))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	proof.trim.Toe.Slope = toeSlope
	originalPayload := tooth.n.Payload
	t.Cleanup(func() { tooth.n.Payload = originalPayload })
	payload := originalPayload.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	toeFaceIndex := indexOfFace(tooth.Faces(), proof.toeFace)
	require.GreaterOrEqual(t, toeFaceIndex, 0)
	changedTriangle := -1
	for index, group := range payload.Src {
		if group != proof.trim.ToeCapGroup &&
			group != proof.trim.HeelCapGroup && payload.FaceOf[index] != toeFaceIndex {
			changedTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, changedTriangle, 0)
	payload.FaceOf[changedTriangle] = toeFaceIndex
	tooth.n.Payload = payload
	require.False(t, revolveLoftFacesTrimCurrent(tooth, proof.trim))
	require.Same(t, proof.toeFace,
		uniqueResultSourceFace(tooth, proof.trim.ToeCapGroup))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.n.Payload = originalPayload
	payload = originalPayload.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	capTriangle := -1
	for index, group := range payload.Src {
		if group == proof.trim.ToeCapGroup {
			capTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, capTriangle, 0)
	payload.FaceOf[capTriangle] = len(tooth.Faces())
	tooth.n.Payload = payload
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.n.Payload = originalPayload
	shell := blank.Shells()[0]
	originalFaces := shell.n.Faces
	t.Cleanup(func() { shell.n.Faces = originalFaces })
	shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
}

func TestRevolveLoftFacesToolCircleOnFiniteSection(t *testing.T) {
	tool := &pointConeLimit{Apex: big.NewRat(8, 1), Far: big.NewRat(-2, 1),
		Slope: big.NewRat(1, 1)}
	section := revolveSectionSpan{Start: planePoint{U: 5, V: 2}, End: planePoint{U: 6, V: 4}}
	x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	requireReducedRat := func(q *big.Rat) {
		t.Helper()
		require.Positive(t, q.Denom().Sign())
		require.Zero(t, new(big.Int).GCD(nil, nil, new(big.Int).Abs(q.Num()), q.Denom()).Cmp(big.NewInt(1)))
	}
	require.Zero(t, x.Cmp(big.NewRat(16, 3)))
	requireReducedRat(x)
	require.Zero(t, radius.Cmp(big.NewRat(8, 3)))
	requireReducedRat(radius)
	require.Zero(t, lfSub(lfAdd(x, radius), tool.Apex).Sign())

	// A vertical Revolve side can also meet the cone at an interior circle.
	section = revolveSectionSpan{Start: planePoint{U: 6, V: 1}, End: planePoint{U: 6, V: 3}}
	x, radius, crosses, decided = revolveloftfaces.ToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Zero(t, x.Cmp(big.NewRat(6, 1)))
	requireReducedRat(x)
	require.Zero(t, radius.Cmp(big.NewRat(2, 1)))
	requireReducedRat(radius)

	// A complete generator on one strict side has no contact circle.
	section = revolveSectionSpan{Start: planePoint{U: 5, V: 1}, End: planePoint{U: 6, V: 1}}
	_, _, crosses, decided = revolveloftfaces.ToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.False(t, crosses)

	// A shared endpoint needs a separate topology proof.
	section.End = planePoint{U: 7, V: 1}
	_, _, crosses, decided = revolveloftfaces.ToolCircleOnSpan(tool, section)
	require.False(t, decided)
	require.False(t, crosses)
}

func TestRevolveLoftFacesCircleMissUsesWholeMappedSourceBox(t *testing.T) {
	box := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1),
		big.NewRat(-1, 1), big.NewRat(1, 1)}
	sectionMap := func(x *big.Rat) loftFaceMap {
		return loftFaceMap{Constant: loftFaceRatVec{lfZero(), lfZero(), lfZero()},
			Q0: loftFaceRatVec{x, big.NewRat(2, 1), lfZero()},
			QU: loftFaceRatVec{lfZero(), lfOne(), lfZero()},
			QV: loftFaceRatVec{lfZero(), lfZero(), lfOne()},
			G0: lfOne(), GU: lfZero(), GV: lfZero()}
	}
	maps := []loftFaceMap{sectionMap(big.NewRat(5, 1)), sectionMap(big.NewRat(7, 1))}
	require.True(t, loftface.CircleOutsideMaps(maps, box,
		big.NewRat(6, 1), big.NewRat(1, 2)))
	require.False(t, loftface.CircleOutsideMaps(maps, box,
		big.NewRat(6, 1), big.NewRat(2, 1)), "the source box can meet this circle")
	require.True(t, loftface.CircleOutsideMaps(maps, box,
		big.NewRat(8, 1), big.NewRat(2, 1)), "the circle is beyond every band")
	require.False(t, loftface.CircleOutsideMaps(maps, box,
		big.NewRat(6, 1), big.NewRat(1, 1)), "boundary equality cannot prove a miss")
	invalid := slices.Clone(maps)
	invalid[0].G0 = lfZero()
	require.False(t, loftface.CircleOutsideMaps(invalid, box,
		big.NewRat(6, 1), big.NewRat(1, 2)), "a zero map denominator cannot prove a miss")
	require.False(t, loftface.CircleOutsideMaps(maps[:1], box,
		big.NewRat(6, 1), big.NewRat(1, 2)), "one cap does not define a band")
}

func TestRevolveLoftFacesCircleMissConsumesLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	require.NotEmpty(t, sides.circles)
	misses := 0
	for _, circle := range sides.circles {
		require.Equal(t, revolveloftfaces.ToolCircleOutsideSource(
			proof.trim, circle.x, circle.radius), circle.sourceMiss)
		if circle.sourceMiss {
			misses++
		}
	}
	require.Len(t, sides.circles, 4)
	require.Equal(t, 2, misses)
	map0 := proof.trim.Maps[0]
	proof.trim.Maps[0] = proof.trim.Maps[1]
	_, accepted := revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted, "the source-miss flag needs the current live maps")
	proof.trim.Maps[0] = map0
	sourceBox := proof.trim.Source.SourceBox
	require.True(t, facetedrecord.PointSourceCurrent(proof.trim.Source))
	proof.trim.Source.SourceBox[1] = (sourceBox[0] + sourceBox[1]) / 2
	require.False(t, facetedrecord.PointSourceCurrent(proof.trim.Source))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted, "a narrowed stored source box cannot prove a whole-circle miss")
	proof.trim.Source.SourceBox = sourceBox
}

func TestRevolveLoftFacesToolCapArcBindsLiveSourceCells(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	box, err := revolveloftfaces.LfSourceBox(proof.trim.Source)
	require.NoError(t, err)
	checked := 0
	for _, circle := range sides.circles {
		if circle.tool != 1 || circle.sourceMiss {
			continue
		}
		span := proof.section.Spans[circle.span]
		var arc *revolveLoftFacesToolCapOccupiedArc
		var certified bool
		if span.Start.U == span.End.U {
			arc, certified, err = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		} else {
			arc, certified, err = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		}
		require.NoError(t, err)
		require.True(t, certified)
		band, time, unique := loftface.ConstantXBand(proof.trim.Maps, box, circle.x)
		require.True(t, unique)
		require.Equal(t, band, arc.band)
		require.Zero(t, time.Cmp(arc.time))
		current, currentErr := revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.NoError(t, currentErr)
		require.True(t, current)
		if span.Start.U != span.End.U {
			lower := lfZero()
			if circle.x.Cmp(proof.trim.Toe.Apex) < 0 {
				lower = lfDiv(lfSub(proof.trim.Toe.Apex, circle.x), proof.trim.Toe.Slope)
			}
			section, sourceOK, sourceErr := revolveLoftFacesConstantXSourceCells(t.Context(), proof,
				circle.x, lower, circle.radius)
			require.NoError(t, sourceErr)
			require.True(t, sourceOK)
			changedSection := *section
			changedSection.cells = slices.Clone(section.cells)
			changedArcCell := false
			for i := range changedSection.cells {
				if changedSection.cells[i].source != nil {
					continue
				}
				changedSection.cells[i].source = make(freeform.BezierSpan, 0)
				changedArcCell = true
				break
			}
			require.True(t, changedArcCell)
			_, certified, sourceErr = revolveLoftFacesHeelOccupiedFromSource(t.Context(), proof,
				circle, &changedSection)
			require.NoError(t, sourceErr)
			require.False(t, certified, "an empty spline record cannot replace the live whole Arc")
		}

		changed := *arc
		changed.band++
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another cap band cannot own the same arc")
		changed = *arc
		changed.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another time cannot own the same source cells")
		changed = *arc
		changed.negativeZ.parameter.Lo = new(big.Rat).Add(
			arc.negativeZ.parameter.Lo, arc.negativeZ.parameter.Hi)
		changed.negativeZ.parameter.Lo.Quo(changed.negativeZ.parameter.Lo, big.NewRat(2, 1))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "a narrowed root bracket is not the certified source bracket")
		changed = *arc
		changed.positiveZ.face = bornFace(&Face{})
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another retained face cannot own the source crossing")
		checked++
	}
	require.Equal(t, 2, checked, "the real source has vertical and sloped heel arcs")
}

func TestRevolveLoftFacesToolCapSourceCellsPreserveSlicePresence(t *testing.T) {
	original := &revolveLoftFacesConstantXSource{}
	changed := *original
	changed.cells = []revolveLoftFacesVerticalCell{}
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))

	original.cells = []revolveLoftFacesVerticalCell{{}}
	changed = *original
	changed.cells = slices.Clone(original.cells)
	changed.cells[0].source = make(freeform.BezierSpan, 0)
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))

	changed.cells[0] = original.cells[0]
	changed.cells[0].retained = []loftface.VerticalPiece{}
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))
}

func TestRevolveLoftFacesToolBoundaryUsesSketchWalkDirection(t *testing.T) {
	// Sketch always reports increasing natural parameters. Reversed changes
	// the loop walk without changing those parameters or the radial root.
	edge := sketch.BoundaryEdge{TStart: 0, TEnd: 1}
	insideToOutside := loftface.VerticalLimit{Before: -1, After: 1}
	outward, ok := revolveloftfaces.ToolBoundaryOutward(edge, insideToOutside)
	require.True(t, ok)
	require.True(t, outward)

	edge.Reversed = true
	outward, ok = revolveloftfaces.ToolBoundaryOutward(edge, insideToOutside)
	require.True(t, ok)
	require.False(t, outward)

	outward, ok = revolveloftfaces.ToolBoundaryOutward(edge,
		loftface.VerticalLimit{Before: 1, After: -1})
	require.True(t, ok)
	require.True(t, outward)

	edge.Partial = true
	_, ok = revolveloftfaces.ToolBoundaryOutward(edge, insideToOutside)
	require.False(t, ok)
}

func TestRevolveLoftFacesHeelVerticalArcConsumesLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(t.Context(), proof.trim)
	require.NoError(t, err)
	require.True(t, simple)
	require.Len(t, loop.Cells, 8)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	for _, circle := range sides.circles {
		if circle.tool != 1 || circle.x.Cmp(big.NewRat(6, 1)) != 0 {
			continue
		}
		arc, certified, arcErr := revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.True(t, certified)
		require.NotNil(t, arc)
		require.False(t, circle.sourceMiss)
		require.Negative(t, arc.negativeZ.box[2].Hi.Sign())
		require.Positive(t, arc.positiveZ.box[2].Lo.Sign())
		require.NotEqual(t, arc.negativeZ.outward, arc.positiveZ.outward)
		changed := circle
		changed.x = new(big.Rat).Add(circle.x, big.NewRat(1, 16))
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the circle must use its live blank section")
		changed = circle
		changed.sourceMiss = true
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "a certified source miss has no occupied arc")
		changed = circle
		changed.capOrigin = provenance.Ref{}
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must name its live cap origin")
		originalOrigins := circle.capFace.n.Origins
		t.Cleanup(func() { circle.capFace.n.Origins = originalOrigins })
		circle.capFace.n.Origins = []provenance.Ref{{Role: "unrelated cap"}}
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "a replaced live cap-face origin cannot own the arc")
		circle.capFace.n.Origins = originalOrigins
		shell := blank.Shells()[0]
		originalFaces := shell.n.Faces
		t.Cleanup(func() { shell.n.Faces = originalFaces })
		shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "an unclassified live blank face cannot own the occupied arc")
		return
	}
	t.Fatal("the real heel cone has no vertical contact circle at X=6")
}

func TestRevolveLoftFacesSlopedRadialInsideNeedsUniqueOuterCrossing(t *testing.T) {
	points := []planePoint{{U: 3.2}, {U: 6}, {U: 6, V: 3.5},
		{U: 5.8, V: 3.3}, {U: 5.7, V: 2.28},
		{U: 4.7, V: 1.88}, {U: 3.2, V: 3.2}}
	section := revolveSectionCertificate{AxisSpan: 0}
	for i, start := range points {
		section.Spans = append(section.Spans, revolveSectionSpan{
			Start: start, End: points[(i+1)%len(points)]})
	}
	x := lfF(5.75)
	fraction := lfDiv(lfSub(x, lfF(5.8)), lfSub(lfF(5.7), lfF(5.8)))
	radius := lfAdd(lfF(3.3), lfMul(fraction, lfSub(lfF(2.28), lfF(3.3))))
	require.True(t, revolveloftfaces.SlopedRadialInside(section, 3, x, radius))
	require.False(t, revolveloftfaces.SlopedRadialInside(section, 3, lfF(5.7), lfF(2.28)),
		"a section vertex at the circle X needs another proof")
	require.False(t, revolveloftfaces.SlopedRadialInside(section, 2, x, radius),
		"the selected face must be the sole positive-radius crossing")
	section.Spans[5].Start.U = 5.9
	require.False(t, revolveloftfaces.SlopedRadialInside(section, 3, x, radius),
		"another side crossing leaves the radial fill undecided")
}

func TestRevolveLoftFacesSlopedRadialInsideRequiresAxisWalkDirection(t *testing.T) {
	section := revolveSectionCertificate{AxisSpan: 0,
		Spans: []revolveSectionSpan{
			{Start: planePoint{U: 3}, End: planePoint{U: 6}},
			{Start: planePoint{U: 6, V: 2}, End: planePoint{U: 3, V: 2}},
		}}
	require.True(t, revolveloftfaces.SlopedRadialInside(section, 1, lfF(4), lfF(2)))
	section.Spans[0].Start.U, section.Spans[0].End.U = 6, 3
	require.False(t, revolveloftfaces.SlopedRadialInside(section, 1, lfF(4), lfF(2)))
}

func TestRevolveLoftFacesHeelSlopedArcConsumesLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(t.Context(), proof.trim)
	require.NoError(t, err)
	require.True(t, simple)
	require.Len(t, loop.Cells, 8)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	found := 0
	for _, circle := range sides.circles {
		span := proof.section.Spans[circle.span]
		if circle.tool != 1 || circle.sourceMiss || span.Start.U == span.End.U {
			continue
		}
		arc, certified, arcErr := revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.True(t, certified, "sloped heel circle at X=%s", circle.x.RatString())
		require.NotNil(t, arc)
		require.Negative(t, arc.negativeZ.box[2].Hi.Sign())
		require.Positive(t, arc.positiveZ.box[2].Lo.Sign())
		lower := lfZero()
		if circle.x.Cmp(proof.trim.Toe.Apex) < 0 {
			lower = lfDiv(lfSub(proof.trim.Toe.Apex, circle.x), proof.trim.Toe.Slope)
		}
		section, sourceOK, sourceErr := revolveLoftFacesConstantXSourceCells(t.Context(), proof,
			circle.x, lower, circle.radius)
		require.NoError(t, sourceErr)
		require.True(t, sourceOK)
		changedSection := *section
		changedSection.cells = slices.Clone(section.cells)
		changedRoots := 0
		for i := range changedSection.cells {
			limit := &changedSection.cells[i].upper
			if limit.Root.Lo == nil {
				continue
			}
			limit.Before, limit.After = limit.After, limit.Before
			changedRoots++
		}
		require.Equal(t, 2, changedRoots)
		_, certified, arcErr = revolveLoftFacesHeelOccupiedFromSource(t.Context(), proof,
			circle, &changedSection)
		require.NoError(t, arcErr)
		require.False(t, certified, "changed crossing signs cannot select another arc")
		changed := circle
		changed.x = new(big.Rat).Add(circle.x, big.NewRat(1, 16))
		_, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must use the live sloped circle")
		changed = circle
		changed.capOrigin = provenance.Ref{}
		_, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must name its live cap origin")
		originalOrigins := circle.capFace.n.Origins
		t.Cleanup(func() { circle.capFace.n.Origins = originalOrigins })
		circle.capFace.n.Origins = []provenance.Ref{{Role: "unrelated cap"}}
		_, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "a replaced live cap-face origin cannot own the arc")
		circle.capFace.n.Origins = originalOrigins
		shell := blank.Shells()[0]
		originalFaces := shell.n.Faces
		t.Cleanup(func() { shell.n.Faces = originalFaces })
		shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "an unclassified live blank face cannot own the arc")
		shell.n.Faces = originalFaces
		found++
	}
	require.Positive(t, found)
}
