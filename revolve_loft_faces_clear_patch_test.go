package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCurvedClearPatchCertifiesRealTrimmedTool(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	source, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	classifier, built, err := newRevolveLoftFacesClearPatchContext(t.Context(), source, units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, built)
	require.NotEmpty(t, classifier.shell.mesh.tris)
	require.NotEmpty(t, classifier.tool.triangles)
	index := -1
	for i := range classifier.shell.mesh.tris {
		location, certified, checkErr := classifier.classify(t.Context(), i)
		require.NoError(t, checkErr)
		if certified {
			require.Equal(t, pointquery.Outside, location)
			index = i
			break
		}
	}
	require.GreaterOrEqual(t, index, 0)
	seamVertex := classifier.shell.seams[0][len(classifier.shell.seams[0])/2]
	seamTriangle := -1
	for i, tri := range classifier.shell.mesh.tris {
		if slices.Contains(tri[:], seamVertex) {
			seamTriangle = i
			break
		}
	}
	require.GreaterOrEqual(t, seamTriangle, 0)
	location, certified, err := classifier.classify(t.Context(), seamTriangle)
	require.NoError(t, err)
	require.False(t, certified)
	require.Equal(t, pointquery.Undecided, location)
}

func TestCurvedClearPatchRefusesUnprovedClaims(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	source, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	classifier, ok, err := newRevolveLoftFacesClearPatchContext(t.Context(), source, units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, ok)
	shell, tool := classifier.shell, classifier.tool
	index := -1
	for i := range shell.mesh.tris {
		location, certified, checkErr := revolveLoftFacesClearPatch(t.Context(), source, shell, tool, i)
		require.NoError(t, checkErr)
		if certified {
			require.Equal(t, pointquery.Outside, location)
			index = i
			break
		}
	}
	require.GreaterOrEqual(t, index, 0)
	refuses := func(t *testing.T, shell *revolveLoftFacesBlankSeamShell, tool *Mesh) {
		t.Helper()
		location, certified, checkErr := revolveLoftFacesClearPatch(t.Context(), source, shell, tool, index)
		require.NoError(t, checkErr)
		require.False(t, certified)
		require.Equal(t, pointquery.Undecided, location)
	}
	t.Run("stale producer", func(t *testing.T) {
		original := trim.stationGroup[0]
		trim.stationGroup[0] = original + 1
		defer func() { trim.stationGroup[0] = original }()
		refuses(t, shell, tool)
	})
	t.Run("small face bound", func(t *testing.T) {
		changed := *shell
		changed.faceBound = 0
		refuses(t, &changed, tool)
	})
	t.Run("wrong triangle provenance", func(t *testing.T) {
		changed := *shell
		changed.triangleCells = slices.Clone(shell.triangleCells)
		changed.triangleCells[index].section = len(source.section.spans)
		refuses(t, &changed, tool)
	})
	t.Run("wrong source group", func(t *testing.T) {
		changed := *shell
		changed.mesh = shell.mesh
		changed.mesh.src = slices.Clone(shell.mesh.src)
		changed.mesh.src[index] = len(shell.mesh.groups)
		refuses(t, &changed, tool)
	})
	t.Run("understated corner bound", func(t *testing.T) {
		changed := *shell
		changed.vertexBounds = slices.Clone(shell.vertexBounds)
		changed.vertexBounds[shell.mesh.tris[index][0]] = shell.faceBound + 1
		refuses(t, &changed, tool)
	})
	t.Run("insufficient separation", func(t *testing.T) {
		changed := *tool
		changed.bound = 1000
		refuses(t, shell, &changed)
	})
	t.Run("undecided point location", func(t *testing.T) {
		changed := *tool
		changed.volSymDiff = math.MaxFloat64
		refuses(t, shell, &changed)
	})
	t.Run("unverified tool", func(t *testing.T) {
		changed := *tool
		changed.symDiffOK = false
		refuses(t, shell, &changed)
	})
}
