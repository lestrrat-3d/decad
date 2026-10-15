package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCurvedSourcePairCensusCoversLiveFaces(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, census.manifest)
	require.Len(t, census.pairs, len(blank.Faces())*len(trimmed.Faces()))

	seen := make(map[[2]*Face]struct{})
	counts := make(map[revolveLoftFacesPairVerdict]int)
	for _, pair := range census.pairs {
		key := [2]*Face{pair.blankFace, pair.trimmedFace}
		_, duplicate := seen[key]
		require.False(t, duplicate, "a live face pair must occur once")
		seen[key] = struct{}{}
		require.Equal(t, proof.section.Spans[pair.sectionIndex].Face, faceNode(pair.blankFace))
		counts[pair.verdict]++
		switch pair.verdict {
		case revolveLoftFacesPairMiss:
			require.Nil(t, pair.toolArc)
		case revolveLoftFacesPairSourceCandidate:
			require.Positive(t, len(pair.sourceCells)+len(pair.verticalCells))
			for _, cell := range pair.sourceCells {
				require.Equal(t, pair.trimmedFace, cell.face)
			}
			for _, cell := range pair.verticalCells {
				require.Equal(t, pair.trimmedFace, cell.face)
				require.NotEmpty(t, cell.retained)
			}
		case revolveLoftFacesPairToolArc:
			require.NotNil(t, pair.toolArc)
			require.Equal(t, pair.trimmedFace, proof.heelFace)
			require.Equal(t, pair.blankFace, pair.toolArc.circle.blankFace)
			require.Equal(t, pair.trimmedFace, pair.toolArc.circle.capFace)
		default:
			t.Fatalf("uncertified face pair verdict %d", pair.verdict)
		}
	}
	for _, blankFace := range blank.Faces() {
		for _, trimmedFace := range trimmed.Faces() {
			_, found := seen[[2]*Face{blankFace, trimmedFace}]
			require.True(t, found)
		}
	}
	require.Equal(t, 0, counts[revolveLoftFacesPairRefusal])
	require.Equal(t, 2, counts[revolveLoftFacesPairToolArc])
	require.Positive(t, counts[revolveLoftFacesPairSourceCandidate])
	_, err = Union(t.Context(), blank, trimmed)
	require.ErrorIs(t, err, ErrUnsupported,
		"certified face pairs still need a held seam and volume proof")
}

func TestCurvedSourcePairCensusRefusesUnownedRetainedFace(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	shell := trimmed.Shells()[0]
	original := shell.n.Faces
	t.Cleanup(func() { shell.n.Faces = original })
	shell.n.Faces = append(slices.Clone(original), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unowned source face"}}, Body: bodyNode(trimmed)}})))
	_, certified, err := revolveLoftFacesCensus(t.Context(), proof)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, certified,
		"every live retained face needs an authenticated source role")
}
