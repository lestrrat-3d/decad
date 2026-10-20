package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func neutralArcSeamCap(t *testing.T, slab *Body, role provenance.Ref) *Face {
	t.Helper()
	faces, err := Faces(FaceCreatedBy(featureRefOf(role))).SelectFaces(slab)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	return faces[0]
}

func neutralArcSeamCone(t *testing.T, doc *Document, apex float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := []*sketch.Point{
		s.CreatePoint(-2, 0), s.CreatePoint(-2, apex+2), s.CreatePoint(apex, 0),
	}
	for i, point := range points {
		s.Fix(point)
		s.CreateLine(point, points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	tool, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return tool
}

func neutralArcSeamSource(t *testing.T) (*Body, *Body, *loftFacesTrimRecord) {
	t.Helper()
	doc := New()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0),
		r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		point := s.CreatePoint(u, v)
		s.Fix(point)
		return point
	}
	center := fixed(0, 0)
	innerRight, innerLeft := fixed(2.2, -0.2), fixed(2.2, 0.2)
	right := []*sketch.Point{fixed(2.8, -0.24), fixed(3.5, -0.3), fixed(4.2, -0.35)}
	left := []*sketch.Point{fixed(2.8, 0.24), fixed(3.5, 0.3), fixed(4.2, 0.35)}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[2], left[2])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(innerRight, right[0])
	s.CreateLine(left[0], innerLeft)
	s.CreateArc(center, innerRight, innerLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	inner, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	slabs := make([]*Body, 0, 8)
	for k := 1; k <= 8; k++ {
		sheetWorld := sketch.NewWorld()
		sheetSketch, sheetErr := sheetWorld.CreateSketch(sheetWorld.XY())
		require.NoError(t, sheetErr)
		x := 8 - 0.7*float64(k)
		a, b := sheetSketch.CreatePoint(x, -20), sheetSketch.CreatePoint(x, 20)
		sheetSketch.Fix(a)
		sheetSketch.Fix(b)
		sheetSketch.CreateLine(a, b)
		_, sheetErr = sheetSketch.Solve(t.Context())
		require.NoError(t, sheetErr)
		require.Len(t, sheetSketch.Chains(), 1)
		sheet, sheetErr := Extrude(t.Context(), doc, sheetSketch, sheetSketch.Chains()[0], TwoSided{
			One: DistanceSide{D: units.Millimeters(20)},
			Two: DistanceSide{D: units.Millimeters(20)},
		})
		require.NoError(t, sheetErr)
		parts, splitErr := Split(t.Context(), inner, sheet)
		require.NoError(t, splitErr)
		require.Len(t, parts, 2)
		inner = parts[0]
		slabs = append(slabs, parts[1])
	}
	sections := make([]*Face, 0, 9)
	sections = append(sections, neutralArcSeamCap(t, slabs[7], refOf(CapStart(slabs[7]))))
	for i := range slices.Backward(slabs) {
		sections = append(sections, neutralArcSeamCap(t, slabs[i], refOf(CapEnd(slabs[i]))))
	}
	loft, err := LoftFaces(t.Context(), doc, sections...)
	require.NoError(t, err)
	toe, err := Cut(t.Context(), loft, neutralArcSeamCone(t, doc, 6.5))
	require.NoError(t, err)
	trimmed, err := Intersect(t.Context(), toe, neutralArcSeamCone(t, doc, 8))
	require.NoError(t, err)
	trimmedPayload, ok := trimmed.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, trimmedPayload.LoftFacesTrim)
	blankWorld := sketch.NewWorld()
	blankSketch, err := blankWorld.CreateSketch(blankWorld.XY())
	require.NoError(t, err)
	section := []planePoint{{U: 3.2}, {U: 6}, {U: 6, V: 3.5},
		{U: 5.8, V: 3.3}, {U: 5.7, V: 2.28}, {U: 4.7, V: 1.88}, {U: 3.2, V: 3.2}}
	points := make([]*sketch.Point, len(section))
	for i, point := range section {
		points[i] = blankSketch.CreatePoint(point.U, point.V)
		blankSketch.Fix(points[i])
	}
	for i := range points {
		blankSketch.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = blankSketch.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, blankSketch.Profiles(), 1)
	blank, err := Revolve(t.Context(), doc, blankSketch, blankSketch.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return blank, trimmed, trimmedPayload.LoftFacesTrim
}

func TestCurvedLoftFacesSourceCertifiesLiveProducer(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, proof)
	require.Same(t, bodyNode(blank), proof.Blank)
	require.Same(t, bodyNode(tooth), proof.Tooth)
	require.Same(t, trim, proof.Trim)
	require.Len(t, proof.SourceRefs, len(trim.Maps)-1)
	require.NotNil(t, proof.ToeFace)
	require.NotNil(t, proof.HeelFace)
	require.NotNil(t, proof.Crossings[0].Span)
	require.NotNil(t, proof.Crossings[1].Span)
	require.NotEqual(t, proof.Crossings[0].Segment, proof.Crossings[1].Segment)
	t.Run("changed retained Revolve side surface", func(t *testing.T) {
		_, complete, err := revolvecontact.AllSideManifest(t.Context(), proof)
		require.NoError(t, err)
		require.True(t, complete)
		var face *Face
		for index, span := range proof.Section.Spans {
			if index == proof.Section.RootSpan || span.Face == nil {
				continue
			}
			if _, ok := span.Face.Surface.(surfacegeom.Cone); ok {
				face = faceHandle(span.Face)
				break
			}
		}
		require.NotNil(t, face)
		original := face.n.Surface
		defer func() { face.n.Surface = original }()
		changed := original.(surfacegeom.Cone)
		changed.Origin.X += 0.125
		face.n.Surface = changed
		require.False(t, revolvecontact.SectionCurrent(proof))
		_, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
		require.NoError(t, err)
		require.False(t, certified)
		_, complete, err = revolvecontact.AllSideManifest(t.Context(), proof)
		require.NoError(t, err)
		require.False(t, complete)
	})

	t.Run("changed cap departure", func(t *testing.T) {
		original := trim.Bounds[0]
		defer func() { trim.Bounds[0] = original }()
		changed := original
		changed.SourceDeparture = slices.Clone(original.SourceDeparture)
		require.NotEmpty(t, changed.SourceDeparture)
		changed.SourceDeparture[0]++
		trim.Bounds[0] = changed
		_, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
		require.NoError(t, err)
		require.False(t, certified)
	})
	t.Run("changed station group", func(t *testing.T) {
		original := trim.StationGroup[0]
		defer func() { trim.StationGroup[0] = original }()
		trim.StationGroup[0] = original + 1
		_, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
		require.NoError(t, err)
		require.False(t, certified)
	})

	var owner *Face
	var ref provenance.Ref
	for band, face := range proof.Crossings[0].Faces {
		if face != nil {
			owner = faceHandle(face)
			ref = proof.SourceRefs[band][proof.Crossings[0].Segment]
			break
		}
	}
	require.NotNil(t, owner)
	t.Run("missing crossing owner", func(t *testing.T) {
		original := owner.n.Origins
		defer func() { owner.n.Origins = original }()
		owner.n.Origins = slices.DeleteFunc(slices.Clone(original),
			func(got provenance.Ref) bool { return got == ref })
		_, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
		require.NoError(t, err)
		require.False(t, certified)
	})
	t.Run("duplicate crossing owner", func(t *testing.T) {
		var other *Face
		for _, face := range tooth.Faces() {
			if face != owner {
				other = face
				break
			}
		}
		require.NotNil(t, other)
		original := other.n.Origins
		defer func() { other.n.Origins = original }()
		other.n.Origins = append(slices.Clone(original), ref)
		_, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
		require.NoError(t, err)
		require.False(t, certified)
	})
}

func neutralSimpleSourceSketch(t *testing.T, extra bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		point := s.CreatePoint(u, v)
		s.Fix(point)
		return point
	}
	center := fixed(0, 0)
	innerRight, innerLeft := fixed(2.2, -0.2), fixed(2.2, 0.2)
	right := []*sketch.Point{fixed(2.8, -0.24), fixed(3.5, -0.3), fixed(4.2, -0.35)}
	left := []*sketch.Point{fixed(2.8, 0.24), fixed(3.5, 0.3), fixed(4.2, 0.35)}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[2], left[2])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(innerRight, right[0])
	s.CreateLine(left[0], innerLeft)
	s.CreateArc(center, innerRight, innerLeft)
	if extra {
		s.CreateLine(fixed(5, 1), fixed(6, 1))
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func neutralSimpleSourceRecord(t *testing.T, s *sketch.Sketch,
	profile *sketch.Profile) *loftFacesTrimRecord {
	t.Helper()
	recorded, plane, _, err := sectioncert.RecordProfile(s, profile)
	require.NoError(t, err)
	frame, err := s.Plane().Frame()
	require.NoError(t, err)
	return &loftFacesTrimRecord{Source: &pointSectionRecord{
		Profile: recorded, Plane: plane,
		Source: &pointSectionSource{Sketch: s, Profile: profile,
			Revision: s.Revision(), Frame: frame,
			Entities: append([]sketch.Entity(nil), profile.Entities...)},
	}}
}

func TestRevolveLoftFacesSimpleSourceCertifiesWholeCurveLoop(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	loop, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(),
		neutralSimpleSourceRecord(t, s, profile))
	require.NoError(t, err)
	require.True(t, certified)
	require.True(t, loop.CCW)
	require.Len(t, loop.Cells, 8, "two fitted flanks each reconstruct as two Bezier spans")
	for i, cell := range loop.Cells {
		next := loop.Cells[(i+1)%len(loop.Cells)]
		require.True(t, revolveloftfaces.SimpleNeighbors(cell, next), "joint %d", i)
	}
}

func TestRevolveLoftFacesSimpleSourceRefusesExtraSketchEntity(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, true)
	_, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(),
		neutralSimpleSourceRecord(t, s, profile))
	require.NoError(t, err)
	require.False(t, certified, "an extra entity may change the true region")
}

func TestRevolveLoftFacesSimpleSourceRefusesChangedRecordedArc(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	trim := neutralSimpleSourceRecord(t, s, profile)
	arc := trim.Source.Profile.Outer.Segments[1].(arcSeg)
	arc.Center.U = 2
	trim.Source.Profile.Outer.Segments[1] = arc
	require.True(t, facetedrecord.PointSourceCurrent(trim.Source),
		"the existing revision gate does not compare stored curve geometry")
	_, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(), trim)
	require.NoError(t, err)
	require.False(t, certified, "the live Sketch arc still has its original center")
}

// The invalid-profile and holed-profile cases below are refused by the
// currency gate first (it re-records the live profile), so the later checks in
// SimpleSourceLoop repeat what that gate already decided.
func TestRevolveLoftFacesSimpleSourceRefusesAnUncertifiedSource(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	tests := map[string]func(*loftFacesTrimRecord){
		"a source that is not current": func(trim *loftFacesTrimRecord) { trim.Source.SourceBox[0]++ },
		"no source":                    func(trim *loftFacesTrimRecord) { trim.Source = nil },
	}
	for name, edit := range tests {
		trim := neutralSimpleSourceRecord(t, s, profile)
		edit(trim)
		loop, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(), trim)
		require.NoError(t, err, name)
		require.False(t, certified, name)
		require.Nil(t, loop, name)
	}
	loop, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(), nil)
	require.NoError(t, err)
	require.False(t, certified)
	require.Nil(t, loop)

	//nolint:staticcheck // a nil context is a documented refusal, not a call to make.
	loop, certified, err = revolveloftfaces.SimpleSourceLoop(nil, neutralSimpleSourceRecord(t, s, profile))
	require.NoError(t, err)
	require.False(t, certified)
	require.Nil(t, loop)

	for name, edit := range map[string]func(*sketch.Profile){
		"an invalid profile": func(p *sketch.Profile) { p.Valid = false },
		"a holed profile":    func(p *sketch.Profile) { p.Holes = append(p.Holes, p.Outer) },
	} {
		s, profile := neutralSimpleSourceSketch(t, false)
		trim := neutralSimpleSourceRecord(t, s, profile)
		edit(trim.Source.Source.Profile)
		loop, certified, err := revolveloftfaces.SimpleSourceLoop(t.Context(), trim)
		require.NoError(t, err, name)
		require.False(t, certified, name)
		require.Nil(t, loop, name)
	}
}

func TestRevolveLoftFacesSimpleNeighborsRequireOppositeInteriorSides(t *testing.T) {
	span := func(a, b float64) loftface.SimpleCell {
		cell, ok := loftface.SimpleBezier(0, 0, freeform.BezierSpan{
			{U: lfF(a), V: lfZero()}, {U: lfF(b), V: lfZero()}})
		require.True(t, ok)
		return cell
	}
	first := span(0, 1)
	require.True(t, revolveloftfaces.SimpleNeighbors(first, span(1, 2)))
	require.False(t, revolveloftfaces.SimpleNeighbors(first, span(1, 0)),
		"a curve that doubles back shares more than the endpoint")
}

func TestRevolveLoftFacesSimpleSeparatedRequiresStrictAxisGap(t *testing.T) {
	span := func(startU, startV, endU, endV float64) loftface.SimpleCell {
		cell, ok := loftface.SimpleBezier(0, 0, freeform.BezierSpan{
			{U: lfF(startU), V: lfF(startV)},
			{U: lfF(endU), V: lfF(endV)}})
		require.True(t, ok)
		return cell
	}
	first := span(0, 0, 1, 1)
	require.True(t, loftface.SimpleSeparated(first, span(2, 0, 3, 1)))
	require.False(t, loftface.SimpleSeparated(first, span(1, 0, 2, 1)),
		"a shared box boundary cannot certify separation")
	require.False(t, loftface.SimpleSeparated(first, span(0.5, 0.5, 1.5, 1.5)),
		"overlap in both coordinate boxes needs a stronger proof")
}

func TestRevolveLoftFacesShortArcSupportsOnlyItsChordOutside(t *testing.T) {
	arc := arcSeg{Center: planePoint{},
		Start: planePoint{U: 4, V: -0.5}, End: planePoint{U: 4, V: 0.5},
		TStart: 0, TEnd: 1}
	require.True(t, revolveloftfaces.SimpleArcChordSide(arc, 0, lfF(4), 1))
	require.False(t, revolveloftfaces.SimpleArcChordSide(arc, 0, lfF(4), -1))
	require.True(t, revolveloftfaces.SimpleArcMonotoneSide(arc, 1, lfF(-0.5), 1))
	require.True(t, revolveloftfaces.SimpleArcMonotoneSide(arc, 1, lfF(0.5), -1))
	require.False(t, revolveloftfaces.SimpleArcMonotoneSide(arc, 0, lfF(4), 1),
		"the bulging coordinate is not monotone")
}

func TestRevolveLoftFacesArcRefusesUnequalEndpointRadius(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	trim := neutralSimpleSourceRecord(t, s, profile)
	arc := trim.Source.Profile.Outer.Segments[1].(arcSeg)
	arc.End.U += 0.01
	_, certified := revolveloftfaces.SimpleArc(trim, 1, arc, false)
	require.False(t, certified)
}
