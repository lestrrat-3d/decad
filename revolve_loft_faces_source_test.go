package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
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
	inner, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, s.Profiles()[0])
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
	loft, err := doc.LoftFaces(t.Context(), sections...)
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
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, proof)
	require.Same(t, blank, proof.blank)
	require.Same(t, tooth, proof.tooth)
	require.Same(t, trim, proof.trim)
	require.Len(t, proof.sourceRefs, len(trim.Maps)-1)
	require.NotNil(t, proof.toeFace)
	require.NotNil(t, proof.heelFace)
	require.NotNil(t, proof.crossings[0].span)
	require.NotNil(t, proof.crossings[1].span)
	require.NotEqual(t, proof.crossings[0].segment, proof.crossings[1].segment)
	t.Run("changed retained Revolve side surface", func(t *testing.T) {
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		require.NoError(t, err)
		require.True(t, complete)
		var face *Face
		for index, span := range proof.section.Spans {
			if index == proof.section.RootSpan || span.Face == nil {
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
		require.False(t, revolveLoftFacesSectionCurrent(proof))
		_, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
		require.NoError(t, err)
		require.False(t, certified)
		_, complete, err = revolveLoftFacesAllSideManifest(t.Context(), proof)
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
		_, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
		require.NoError(t, err)
		require.False(t, certified)
	})
	t.Run("changed station group", func(t *testing.T) {
		original := trim.StationGroup[0]
		defer func() { trim.StationGroup[0] = original }()
		trim.StationGroup[0] = original + 1
		_, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
		require.NoError(t, err)
		require.False(t, certified)
	})

	var owner *Face
	var ref provenance.Ref
	for band, face := range proof.crossings[0].faces {
		if face != nil {
			owner = face
			ref = proof.sourceRefs[band][proof.crossings[0].segment]
			break
		}
	}
	require.NotNil(t, owner)
	t.Run("missing crossing owner", func(t *testing.T) {
		original := owner.n.Origins
		defer func() { owner.n.Origins = original }()
		owner.n.Origins = slices.DeleteFunc(slices.Clone(original),
			func(got provenance.Ref) bool { return got == ref })
		_, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
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
		_, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
		require.NoError(t, err)
		require.False(t, certified)
	})
}
