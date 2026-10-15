package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

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
	require.True(t, revolveLoftFacesSlopedRadialInside(section, 3, x, radius))
	require.False(t, revolveLoftFacesSlopedRadialInside(section, 3, lfF(5.7), lfF(2.28)),
		"a section vertex at the circle X needs another proof")
	require.False(t, revolveLoftFacesSlopedRadialInside(section, 2, x, radius),
		"the selected face must be the sole positive-radius crossing")
	section.Spans[5].Start.U = 5.9
	require.False(t, revolveLoftFacesSlopedRadialInside(section, 3, x, radius),
		"another side crossing leaves the radial fill undecided")
}

func TestRevolveLoftFacesSlopedRadialInsideRequiresAxisWalkDirection(t *testing.T) {
	section := revolveSectionCertificate{AxisSpan: 0,
		Spans: []revolveSectionSpan{
			{Start: planePoint{U: 3}, End: planePoint{U: 6}},
			{Start: planePoint{U: 6, V: 2}, End: planePoint{U: 3, V: 2}},
		}}
	require.True(t, revolveLoftFacesSlopedRadialInside(section, 1, lfF(4), lfF(2)))
	section.Spans[0].Start.U, section.Spans[0].End.U = 6, 3
	require.False(t, revolveLoftFacesSlopedRadialInside(section, 1, lfF(4), lfF(2)))
}

func TestRevolveLoftFacesHeelSlopedArcConsumesLiveNeutralSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(t.Context(), proof.trim)
	require.NoError(t, err)
	require.True(t, simple)
	require.Len(t, loop.cells, 8)
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
		if circle.x.Cmp(proof.trim.Toe.apex) < 0 {
			lower = lfDiv(lfSub(proof.trim.Toe.apex, circle.x), proof.trim.Toe.slope)
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
