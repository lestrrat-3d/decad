package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolBoundaryUsesSketchWalkDirection(t *testing.T) {
	// Sketch always reports increasing natural parameters. Reversed changes
	// the loop walk without changing those parameters or the radial root.
	edge := sketch.BoundaryEdge{TStart: 0, TEnd: 1}
	insideToOutside := loftface.VerticalLimit{Before: -1, After: 1}
	outward, ok := revolveLoftFacesToolBoundaryOutward(edge, insideToOutside)
	require.True(t, ok)
	require.True(t, outward)

	edge.Reversed = true
	outward, ok = revolveLoftFacesToolBoundaryOutward(edge, insideToOutside)
	require.True(t, ok)
	require.False(t, outward)

	outward, ok = revolveLoftFacesToolBoundaryOutward(edge,
		loftface.VerticalLimit{Before: 1, After: -1})
	require.True(t, ok)
	require.True(t, outward)

	edge.Partial = true
	_, ok = revolveLoftFacesToolBoundaryOutward(edge, insideToOutside)
	require.False(t, ok)
}

func TestRevolveLoftFacesHeelVerticalArcConsumesLiveNeutralSource(t *testing.T) {
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
		originalFaces := shell.faces
		t.Cleanup(func() { shell.faces = originalFaces })
		shell.faces = append(slices.Clone(originalFaces), bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}}))
		_, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		require.NoError(t, arcErr)
		require.False(t, certified, "an unclassified live blank face cannot own the occupied arc")
		return
	}
	t.Fatal("the real heel cone has no vertical contact circle at X=6")
}
