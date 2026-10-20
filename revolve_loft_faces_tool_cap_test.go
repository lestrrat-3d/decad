package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolCirclesConsumeLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolvecontact.ToolCapSidesOf(proof)
	require.True(t, ok)
	require.NotEmpty(t, sides.Circles)
	for _, circle := range sides.Circles {
		require.Same(t, bodyNode(blank), circle.BlankFace.Body)
		require.Same(t, bodyNode(tooth), circle.CapFace.Body)
		require.Positive(t, circle.Radius.Sign())
		tool := proof.Trim.Toe
		if circle.Tool == 1 {
			tool = proof.Trim.Heel
		}
		require.Zero(t, lfSub(lfAdd(circle.X,
			lfMul(tool.Slope, circle.Radius)), tool.Apex).Sign())
	}
	t.Run("changed non-root Revolve surface", func(t *testing.T) {
		for index, span := range proof.Section.Spans {
			if index == proof.Section.RootSpan || span.Face == nil {
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
			_, accepted := revolvecontact.ToolCapSidesOf(proof)
			require.False(t, accepted)
			return
		}
		t.Fatal("missing non-root cone face")
	})
	changed := *proof
	changed.Section = proof.Section
	changed.Section.Spans = append([]revolveSectionSpan(nil), proof.Section.Spans...)
	changed.Section.Spans[proof.Section.RootSpan].End.V += 0.125
	_, accepted := revolvecontact.ToolCapSidesOf(&changed)
	require.False(t, accepted)
	changed = *proof
	changed.ToeFace = proof.HeelFace
	_, accepted = revolvecontact.ToolCapSidesOf(&changed)
	require.False(t, accepted)
	toeSlope := proof.Trim.Toe.Slope
	proof.Trim.Toe.Slope = lfAdd(toeSlope, big.NewRat(1, 32))
	_, accepted = revolvecontact.ToolCapSidesOf(proof)
	require.False(t, accepted)
	proof.Trim.Toe.Slope = toeSlope
	originalPayload := tooth.n.Payload
	t.Cleanup(func() { tooth.n.Payload = originalPayload })
	payload := originalPayload.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	toeFaceIndex := loftfacestrim.IndexOfFace(topobuild.Faces(bodyNode(tooth)), proof.ToeFace)
	require.GreaterOrEqual(t, toeFaceIndex, 0)
	changedTriangle := -1
	for index, group := range payload.Src {
		if group != proof.Trim.ToeCapGroup &&
			group != proof.Trim.HeelCapGroup && payload.FaceOf[index] != toeFaceIndex {
			changedTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, changedTriangle, 0)
	payload.FaceOf[changedTriangle] = toeFaceIndex
	tooth.n.Payload = payload
	require.False(t, revolvecontact.TrimCurrent(bodyNode(tooth), proof.Trim))
	require.Same(t, proof.ToeFace,
		revolvecontact.UniqueResultSourceFace(bodyNode(tooth), proof.Trim.ToeCapGroup))
	_, accepted = revolvecontact.ToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.n.Payload = originalPayload
	payload = originalPayload.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	capTriangle := -1
	for index, group := range payload.Src {
		if group == proof.Trim.ToeCapGroup {
			capTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, capTriangle, 0)
	payload.FaceOf[capTriangle] = len(tooth.Faces())
	tooth.n.Payload = payload
	_, accepted = revolvecontact.ToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.n.Payload = originalPayload
	shell := blank.Shells()[0]
	originalFaces := shell.n.Faces
	t.Cleanup(func() { shell.n.Faces = originalFaces })
	shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
	_, accepted = revolvecontact.ToolCapSidesOf(proof)
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolvecontact.ToolCapSidesOf(proof)
	require.True(t, ok)
	require.NotEmpty(t, sides.Circles)
	misses := 0
	for _, circle := range sides.Circles {
		require.Equal(t, revolveloftfaces.ToolCircleOutsideSource(
			proof.Trim, circle.X, circle.Radius), circle.SourceMiss)
		if circle.SourceMiss {
			misses++
		}
	}
	require.Len(t, sides.Circles, 4)
	require.Equal(t, 2, misses)
	map0 := proof.Trim.Maps[0]
	proof.Trim.Maps[0] = proof.Trim.Maps[1]
	_, accepted := revolvecontact.ToolCapSidesOf(proof)
	require.False(t, accepted, "the source-miss flag needs the current live maps")
	proof.Trim.Maps[0] = map0
	sourceBox := proof.Trim.Source.SourceBox
	require.True(t, facetedrecord.PointSourceCurrent(proof.Trim.Source))
	proof.Trim.Source.SourceBox[1] = (sourceBox[0] + sourceBox[1]) / 2
	require.False(t, facetedrecord.PointSourceCurrent(proof.Trim.Source))
	_, accepted = revolvecontact.ToolCapSidesOf(proof)
	require.False(t, accepted, "a narrowed stored source box cannot prove a whole-circle miss")
	proof.Trim.Source.SourceBox = sourceBox
}

func TestRevolveLoftFacesToolCapArcBindsLiveSourceCells(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolvecontact.ToolCapSidesOf(proof)
	require.True(t, ok)
	box, err := revolveloftfaces.LfSourceBox(proof.Trim.Source)
	require.NoError(t, err)
	checked := 0
	for _, circle := range sides.Circles {
		if circle.Tool != 1 || circle.SourceMiss {
			continue
		}
		span := proof.Section.Spans[circle.Span]
		var arc *revolvecontact.ToolCapOccupiedArc
		var certified bool
		if span.Start.U == span.End.U {
			arc, certified, err = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, circle)
		} else {
			arc, certified, err = revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, circle)
		}
		require.NoError(t, err)
		require.True(t, certified)
		band, time, unique := loftface.ConstantXBand(proof.Trim.Maps, box, circle.X)
		require.True(t, unique)
		require.Equal(t, band, arc.Band)
		require.Zero(t, time.Cmp(arc.Time))
		current, currentErr := revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.NoError(t, currentErr)
		require.True(t, current)
		if span.Start.U != span.End.U {
			lower := lfZero()
			if circle.X.Cmp(proof.Trim.Toe.Apex) < 0 {
				lower = lfDiv(lfSub(proof.Trim.Toe.Apex, circle.X), proof.Trim.Toe.Slope)
			}
			section, sourceOK, sourceErr := revolvecontact.ConstantXSourceCells(t.Context(), proof,
				circle.X, lower, circle.Radius)
			require.NoError(t, sourceErr)
			require.True(t, sourceOK)
			changedSection := *section
			changedSection.Cells = slices.Clone(section.Cells)
			changedArcCell := false
			for i := range changedSection.Cells {
				if changedSection.Cells[i].Source != nil {
					continue
				}
				changedSection.Cells[i].Source = make(freeform.BezierSpan, 0)
				changedArcCell = true
				break
			}
			require.True(t, changedArcCell)
			_, certified, sourceErr = revolvecontact.HeelOccupiedFromSource(t.Context(), proof,
				circle, &changedSection)
			require.NoError(t, sourceErr)
			require.False(t, certified, "an empty spline record cannot replace the live whole Arc")
		}

		changed := *arc
		changed.Band++
		current, currentErr = revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another cap band cannot own the same arc")
		changed = *arc
		changed.Time = new(big.Rat).Add(arc.Time, big.NewRat(1, 1<<20))
		current, currentErr = revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another time cannot own the same source cells")
		changed = *arc
		changed.NegativeZ.Parameter.Lo = new(big.Rat).Add(
			arc.NegativeZ.Parameter.Lo, arc.NegativeZ.Parameter.Hi)
		changed.NegativeZ.Parameter.Lo.Quo(changed.NegativeZ.Parameter.Lo, big.NewRat(2, 1))
		current, currentErr = revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "a narrowed root bracket is not the certified source bracket")
		changed = *arc
		changed.PositiveZ.Face = faceNode(bornFace(&Face{}))
		current, currentErr = revolvecontact.ToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another retained face cannot own the source crossing")
		checked++
	}
	require.Equal(t, 2, checked, "the real source has vertical and sloped heel arcs")
}

func TestRevolveLoftFacesToolCapSourceCellsPreserveSlicePresence(t *testing.T) {
	original := &revolvecontact.ConstantXSource{}
	changed := *original
	changed.Cells = []revolvecontact.VerticalCell{}
	require.False(t, revolvecontact.ToolCapSourceCellsEqual(original, &changed))

	original.Cells = []revolvecontact.VerticalCell{{}}
	changed = *original
	changed.Cells = slices.Clone(original.Cells)
	changed.Cells[0].Source = make(freeform.BezierSpan, 0)
	require.False(t, revolvecontact.ToolCapSourceCellsEqual(original, &changed))

	changed.Cells[0] = original.Cells[0]
	changed.Cells[0].Retained = []loftface.VerticalPiece{}
	require.False(t, revolvecontact.ToolCapSourceCellsEqual(original, &changed))
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(t.Context(), proof.Trim)
	require.NoError(t, err)
	require.True(t, simple)
	require.Len(t, loop.Cells, 8)
	sides, ok := revolvecontact.ToolCapSidesOf(proof)
	require.True(t, ok)
	for _, circle := range sides.Circles {
		if circle.Tool != 1 || circle.X.Cmp(big.NewRat(6, 1)) != 0 {
			continue
		}
		arc, certified, arcErr := revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.True(t, certified)
		require.NotNil(t, arc)
		require.False(t, circle.SourceMiss)
		require.Negative(t, arc.NegativeZ.Box[2].Hi.Sign())
		require.Positive(t, arc.PositiveZ.Box[2].Lo.Sign())
		require.NotEqual(t, arc.NegativeZ.Outward, arc.PositiveZ.Outward)
		changed := circle
		changed.X = new(big.Rat).Add(circle.X, big.NewRat(1, 16))
		_, certified, arcErr = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the circle must use its live blank section")
		changed = circle
		changed.SourceMiss = true
		_, certified, arcErr = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "a certified source miss has no occupied arc")
		changed = circle
		changed.CapOrigin = provenance.Ref{}
		_, certified, arcErr = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must name its live cap origin")
		originalOrigins := circle.CapFace.Origins
		t.Cleanup(func() { circle.CapFace.Origins = originalOrigins })
		circle.CapFace.Origins = []provenance.Ref{{Role: "unrelated cap"}}
		_, certified, arcErr = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "a replaced live cap-face origin cannot own the arc")
		circle.CapFace.Origins = originalOrigins
		shell := blank.Shells()[0]
		originalFaces := shell.n.Faces
		t.Cleanup(func() { shell.n.Faces = originalFaces })
		shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, certified, arcErr = revolvecontact.HeelVerticalOccupiedArc(t.Context(), proof, circle)
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
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(t.Context(), proof.Trim)
	require.NoError(t, err)
	require.True(t, simple)
	require.Len(t, loop.Cells, 8)
	sides, ok := revolvecontact.ToolCapSidesOf(proof)
	require.True(t, ok)
	found := 0
	for _, circle := range sides.Circles {
		span := proof.Section.Spans[circle.Span]
		if circle.Tool != 1 || circle.SourceMiss || span.Start.U == span.End.U {
			continue
		}
		arc, certified, arcErr := revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.True(t, certified, "sloped heel circle at X=%s", circle.X.RatString())
		require.NotNil(t, arc)
		require.Negative(t, arc.NegativeZ.Box[2].Hi.Sign())
		require.Positive(t, arc.PositiveZ.Box[2].Lo.Sign())
		lower := lfZero()
		if circle.X.Cmp(proof.Trim.Toe.Apex) < 0 {
			lower = lfDiv(lfSub(proof.Trim.Toe.Apex, circle.X), proof.Trim.Toe.Slope)
		}
		section, sourceOK, sourceErr := revolvecontact.ConstantXSourceCells(t.Context(), proof,
			circle.X, lower, circle.Radius)
		require.NoError(t, sourceErr)
		require.True(t, sourceOK)
		changedSection := *section
		changedSection.Cells = slices.Clone(section.Cells)
		changedRoots := 0
		for i := range changedSection.Cells {
			limit := &changedSection.Cells[i].Upper
			if limit.Root.Lo == nil {
				continue
			}
			limit.Before, limit.After = limit.After, limit.Before
			changedRoots++
		}
		require.Equal(t, 2, changedRoots)
		_, certified, arcErr = revolvecontact.HeelOccupiedFromSource(t.Context(), proof,
			circle, &changedSection)
		require.NoError(t, arcErr)
		require.False(t, certified, "changed crossing signs cannot select another arc")
		changed := circle
		changed.X = new(big.Rat).Add(circle.X, big.NewRat(1, 16))
		_, certified, arcErr = revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must use the live sloped circle")
		changed = circle
		changed.CapOrigin = provenance.Ref{}
		_, certified, arcErr = revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, changed)
		require.NoError(t, arcErr)
		require.False(t, certified, "the occupied arc must name its live cap origin")
		originalOrigins := circle.CapFace.Origins
		t.Cleanup(func() { circle.CapFace.Origins = originalOrigins })
		circle.CapFace.Origins = []provenance.Ref{{Role: "unrelated cap"}}
		_, certified, arcErr = revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "a replaced live cap-face origin cannot own the arc")
		circle.CapFace.Origins = originalOrigins
		shell := blank.Shells()[0]
		originalFaces := shell.n.Faces
		t.Cleanup(func() { shell.n.Faces = originalFaces })
		shell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, certified, arcErr = revolvecontact.HeelSlopedOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "an unclassified live blank face cannot own the arc")
		shell.n.Faces = originalFaces
		found++
	}
	require.Positive(t, found)
}
