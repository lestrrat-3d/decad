package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	require.NotEmpty(t, classifier.shell.Mesh.Tris)
	require.NotEmpty(t, classifier.tool.n.Triangles)
	index := -1
	for i := range classifier.shell.Mesh.Tris {
		location, certified, checkErr := classifier.classify(t.Context(), i)
		require.NoError(t, checkErr)
		if certified {
			require.Equal(t, pointquery.Outside, location)
			index = i
			break
		}
	}
	require.GreaterOrEqual(t, index, 0)
	seamVertex := classifier.shell.Seams[0][len(classifier.shell.Seams[0])/2]
	seamTriangle := -1
	for i, tri := range classifier.shell.Mesh.Tris {
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
	for i := range shell.Mesh.Tris {
		location, certified, checkErr := classifier.classify(t.Context(), i)
		require.NoError(t, checkErr)
		if certified {
			require.Equal(t, pointquery.Outside, location)
			index = i
			break
		}
	}
	require.GreaterOrEqual(t, index, 0)
	refuses := func(t *testing.T, classifier *revolveLoftFacesClearPatchContext) {
		t.Helper()
		location, certified, checkErr := classifier.classify(t.Context(), index)
		require.NoError(t, checkErr)
		require.False(t, certified)
		require.Equal(t, pointquery.Undecided, location)
	}
	refusesEntry := func(t *testing.T) {
		t.Helper()
		_, built, buildErr := newRevolveLoftFacesClearPatchContext(t.Context(), source, units.Millimeters(0.1))
		require.NoError(t, buildErr)
		require.False(t, built)
	}
	t.Run("stale producer", func(t *testing.T) {
		original := trim.StationGroup[0]
		trim.StationGroup[0] = original + 1
		defer func() { trim.StationGroup[0] = original }()
		refusesEntry(t)
	})
	t.Run("changed body facet", func(t *testing.T) {
		original := trimmed.n.Payload
		changed := original.(facetedPayload)
		changed.Verts = slices.Clone(changed.Verts)
		changed.Verts[0].X += 1
		trimmed.n.Payload = changed
		defer func() { trimmed.n.Payload = original }()
		refusesEntry(t)
	})
	t.Run("changed body facet after context", func(t *testing.T) {
		original := trimmed.n.Payload
		changed := original.(facetedPayload)
		changed.Verts = slices.Clone(changed.Verts)
		changed.Verts[0].X += 1
		trimmed.n.Payload = changed
		defer func() { trimmed.n.Payload = original }()
		refuses(t, classifier)
	})
	t.Run("changed body bound after context", func(t *testing.T) {
		original := trimmed.n.Payload
		changed := original.(facetedPayload)
		changed.MeshBound = math.Nextafter(changed.MeshBound, 0)
		trimmed.n.Payload = changed
		defer func() { trimmed.n.Payload = original }()
		refuses(t, classifier)
	})
	t.Run("changed body volume proof after context", func(t *testing.T) {
		original := trimmed.n.Payload
		changed := original.(facetedPayload)
		changed.VolSymDiff = math.Nextafter(changed.VolSymDiff, math.Inf(1))
		trimmed.n.Payload = changed
		defer func() { trimmed.n.Payload = original }()
		refuses(t, classifier)
	})
	t.Run("changed owned tool facet after context", func(t *testing.T) {
		original := classifier.tool.n.Vertices[0]
		classifier.tool.n.Vertices[0].X += 1
		defer func() { classifier.tool.n.Vertices[0] = original }()
		refuses(t, classifier)
	})
	t.Run("understated owned tool bound after context", func(t *testing.T) {
		original := classifier.tool.n.Bound
		require.Positive(t, original)
		classifier.tool.n.Bound = math.Nextafter(original, 0)
		defer func() { classifier.tool.n.Bound = original }()
		refuses(t, classifier)
	})
	t.Run("changed owned shell vertex after context", func(t *testing.T) {
		vertex := shell.Mesh.Tris[index][0]
		original := shell.Mesh.Verts[vertex]
		shell.Mesh.Verts[vertex].X += 1
		defer func() { shell.Mesh.Verts[vertex] = original }()
		refuses(t, classifier)
	})
	t.Run("understated owned shell bound after context", func(t *testing.T) {
		original := shell.FaceBound
		require.Positive(t, original)
		shell.FaceBound = math.Nextafter(original, 0)
		defer func() { shell.FaceBound = original }()
		refuses(t, classifier)
	})
	t.Run("changed cached facet", func(t *testing.T) {
		original := trimmed.n.Tess.Load()
		changed := bornMesh(&Mesh{n: tool.n})
		changed.n.Vertices = slices.Clone(tool.n.Vertices)
		changed.n.Vertices[0].X += 1
		trimmed.n.Tess.Store(&topo.TessEntry{
			ChordBits: math.Float64bits(0.1), Verify: int(VerifyAll), Mesh: meshNode(changed)})
		defer trimmed.n.Tess.Store(original)
		refusesEntry(t)
	})
	t.Run("understated positive cached bound", func(t *testing.T) {
		require.Positive(t, tool.n.Bound)
		original := trimmed.n.Tess.Load()
		changed := bornMesh(&Mesh{n: tool.n})
		changed.n.Bound = math.Nextafter(tool.n.Bound, 0)
		require.Positive(t, changed.n.Bound)
		trimmed.n.Tess.Store(&topo.TessEntry{
			ChordBits: math.Float64bits(0.1), Verify: int(VerifyAll), Mesh: meshNode(changed)})
		defer trimmed.n.Tess.Store(original)
		refusesEntry(t)
	})
	t.Run("small face bound", func(t *testing.T) {
		changed := *shell
		changed.FaceBound = 0
		modified := *classifier
		modified.shell = &changed
		refuses(t, &modified)
	})
	t.Run("wrong triangle provenance", func(t *testing.T) {
		changed := *shell
		changed.TriangleCells = slices.Clone(shell.TriangleCells)
		changed.TriangleCells[index].Section = len(source.section.Spans)
		modified := *classifier
		modified.shell = &changed
		refuses(t, &modified)
	})
	t.Run("wrong source group", func(t *testing.T) {
		changed := *shell
		changed.Mesh = shell.Mesh
		changed.Mesh.Src = slices.Clone(shell.Mesh.Src)
		changed.Mesh.Src[index] = len(shell.Mesh.Groups)
		modified := *classifier
		modified.shell = &changed
		refuses(t, &modified)
	})
	t.Run("understated corner bound", func(t *testing.T) {
		changed := *shell
		changed.VertexBounds = slices.Clone(shell.VertexBounds)
		changed.VertexBounds[shell.Mesh.Tris[index][0]] = shell.FaceBound + 1
		modified := *classifier
		modified.shell = &changed
		refuses(t, &modified)
	})
	t.Run("insufficient separation", func(t *testing.T) {
		changed := bornMesh(&Mesh{n: tool.n})
		changed.n.Bound = 1000
		modified := *classifier
		modified.tool = changed
		refuses(t, &modified)
	})
	t.Run("undecided point location", func(t *testing.T) {
		changed := bornMesh(&Mesh{n: tool.n})
		changed.n.VolSymDiff = math.MaxFloat64
		modified := *classifier
		modified.tool = changed
		refuses(t, &modified)
	})
	t.Run("unverified tool", func(t *testing.T) {
		changed := bornMesh(&Mesh{n: tool.n})
		changed.n.SymDiffOK = false
		modified := *classifier
		modified.tool = changed
		refuses(t, &modified)
	})
}
