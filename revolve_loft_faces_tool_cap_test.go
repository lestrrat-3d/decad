package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
			lfMul(tool.slope, circle.radius)), tool.apex).Sign())
	}
	t.Run("changed non-root Revolve surface", func(t *testing.T) {
		for index, span := range proof.section.spans {
			if index == proof.section.rootSpan || span.face == nil {
				continue
			}
			cone, ok := span.face.n.Surface.(surfacegeom.Cone)
			if !ok {
				continue
			}
			original := span.face.n.Surface
			defer func() { span.face.n.Surface = original }()
			cone.Origin.X += 0.125
			span.face.n.Surface = cone
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
	toeSlope := proof.trim.Toe.slope
	proof.trim.Toe.slope = lfAdd(toeSlope, big.NewRat(1, 32))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted)
	proof.trim.Toe.slope = toeSlope
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
	tool := &pointConeLimit{apex: big.NewRat(8, 1), far: big.NewRat(-2, 1),
		slope: big.NewRat(1, 1)}
	section := revolveSectionSpan{start: planePoint{U: 5, V: 2}, end: planePoint{U: 6, V: 4}}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(tool, section)
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
	require.Zero(t, lfSub(lfAdd(x, radius), tool.apex).Sign())

	// A vertical Revolve side can also meet the cone at an interior circle.
	section = revolveSectionSpan{start: planePoint{U: 6, V: 1}, end: planePoint{U: 6, V: 3}}
	x, radius, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Zero(t, x.Cmp(big.NewRat(6, 1)))
	requireReducedRat(x)
	require.Zero(t, radius.Cmp(big.NewRat(2, 1)))
	requireReducedRat(radius)

	// A complete generator on one strict side has no contact circle.
	section = revolveSectionSpan{start: planePoint{U: 5, V: 1}, end: planePoint{U: 6, V: 1}}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.False(t, crosses)

	// A shared endpoint needs a separate topology proof.
	section.end = planePoint{U: 7, V: 1}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
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
		require.Equal(t, revolveLoftFacesToolCircleOutsideSource(
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
	sourceBox := proof.trim.Source.sourceBox
	require.True(t, pointSourceCurrent(proof.trim.Source))
	proof.trim.Source.sourceBox[1] = (sourceBox[0] + sourceBox[1]) / 2
	require.False(t, pointSourceCurrent(proof.trim.Source))
	_, accepted = revolveLoftFacesToolCapSidesOf(proof)
	require.False(t, accepted, "a narrowed stored source box cannot prove a whole-circle miss")
	proof.trim.Source.sourceBox = sourceBox
}
