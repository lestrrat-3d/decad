package decad

import (
	"math/big"
	"slices"
	"testing"

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
		require.Same(t, blank, circle.blankFace.body)
		require.Same(t, tooth, circle.capFace.body)
		require.Positive(t, circle.radius.Sign())
		tool := proof.trim.toe
		if circle.tool == 1 {
			tool = proof.trim.heel
		}
		require.Zero(t, lfSub(lfAdd(circle.x,
			lfMul(tool.slope, circle.radius)), tool.apex).Sign())
	}
	t.Run("changed non-root Revolve surface", func(t *testing.T) {
		for index, span := range proof.section.spans {
			if index == proof.section.rootSpan || span.face == nil {
				continue
			}
			cone, ok := span.face.surface.(Cone)
			if !ok {
				continue
			}
			original := span.face.surface
			defer func() { span.face.surface = original }()
			cone.Origin.X += 0.125
			span.face.surface = cone
			_, accepted := revolveLoftFacesToolCapSidesOf(proof)
			require.False(t, accepted)
			return
		}
		t.Fatal("missing non-root cone face")
	})
	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[proof.section.rootSpan].end.V += 0.125
	_, accepted := revolveLoftFacesToolCapSidesOf(&changed)
	require.False(t, accepted)
	changed = *proof
	changed.toeFace = proof.heelFace
	_, accepted = revolveLoftFacesToolCapSidesOf(&changed)
	require.False(t, accepted)
	toeSlope := proof.trim.toe.slope
	proof.trim.toe.slope = lfAdd(toeSlope, big.NewRat(1, 32))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	proof.trim.toe.slope = toeSlope
	originalPayload := tooth.payload
	t.Cleanup(func() { tooth.payload = originalPayload })
	payload := originalPayload.(facetedPayload)
	payload.faceOf = slices.Clone(payload.faceOf)
	toeFaceIndex := indexOfFace(tooth.Faces(), proof.toeFace)
	require.GreaterOrEqual(t, toeFaceIndex, 0)
	changedTriangle := -1
	for index, group := range payload.src {
		if group != proof.trim.toeCapGroup &&
			group != proof.trim.heelCapGroup && payload.faceOf[index] != toeFaceIndex {
			changedTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, changedTriangle, 0)
	payload.faceOf[changedTriangle] = toeFaceIndex
	tooth.payload = payload
	require.False(t, revolveLoftFacesTrimCurrent(tooth, proof.trim))
	require.Same(t, proof.toeFace,
		uniqueResultSourceFace(tooth, proof.trim.toeCapGroup))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.payload = originalPayload
	payload = originalPayload.(facetedPayload)
	payload.faceOf = slices.Clone(payload.faceOf)
	capTriangle := -1
	for index, group := range payload.src {
		if group == proof.trim.toeCapGroup {
			capTriangle = index
			break
		}
	}
	require.GreaterOrEqual(t, capTriangle, 0)
	payload.faceOf[capTriangle] = len(tooth.Faces())
	tooth.payload = payload
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	tooth.payload = originalPayload
	shell := blank.Shells()[0]
	originalFaces := shell.faces
	t.Cleanup(func() { shell.faces = originalFaces })
	shell.faces = append(slices.Clone(originalFaces), &Face{
		body: blank, origins: []FeatureRef{{Role: "unmatched side"}},
	})
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
}

func TestRevolveLoftFacesToolCircleOnFiniteSection(t *testing.T) {
	tool := &pointConeLimit{apex: big.NewRat(8, 1), far: big.NewRat(-2, 1),
		slope: big.NewRat(1, 1)}
	section := revolveSectionSpan{start: Point2{U: 5, V: 2}, end: Point2{U: 6, V: 4}}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Equal(t, big.NewRat(16, 3), x)
	require.Equal(t, big.NewRat(8, 3), radius)
	require.Zero(t, lfSub(lfAdd(x, radius), tool.apex).Sign())

	// A vertical Revolve side can also meet the cone at an interior circle.
	section = revolveSectionSpan{start: Point2{U: 6, V: 1}, end: Point2{U: 6, V: 3}}
	x, radius, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Equal(t, big.NewRat(6, 1), x)
	require.Equal(t, big.NewRat(2, 1), radius)

	// A complete generator on one strict side has no contact circle.
	section = revolveSectionSpan{start: Point2{U: 5, V: 1}, end: Point2{U: 6, V: 1}}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.False(t, crosses)

	// A shared endpoint needs a separate topology proof.
	section.end = Point2{U: 7, V: 1}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.False(t, decided)
	require.False(t, crosses)
}
