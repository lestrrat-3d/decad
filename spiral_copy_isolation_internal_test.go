package decad

import (
	"context"
	"maps"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// spiralNonTangentCapTilt turns the tooth's heel cap plane away from the heel
// cone, as apitest's spiralCapTilt does: an exactly tangent plane makes the
// cone-trim verdict depend on FMA rounding.
const spiralNonTangentCapTilt = -1e-5

// meshSnapshot is a deep copy of every field of a mesh that the spiral
// transports might share with their source by aliasing.
type meshSnapshot struct {
	source      []*topo.Face
	faceBound   map[*topo.Face]float64
	triangles   [][3]int
	vertices    []r3.Vec
	vertexBound []float64
	scalars     [6]float64
}

func snapshotMesh(m *Mesh) meshSnapshot {
	return meshSnapshot{
		source:      slices.Clone(m.n.Source),
		faceBound:   maps.Clone(m.n.FaceBound),
		triangles:   slices.Clone(m.n.Triangles),
		vertices:    slices.Clone(m.n.Vertices),
		vertexBound: slices.Clone(m.n.VertexBound),
		scalars: [6]float64{m.n.Bound, m.n.CoordBound, m.n.VolSymDiff, m.n.AreaSlack,
			float64(len(m.n.Source)), float64(len(m.n.FaceBound))},
	}
}

func requireMeshSnapshot(t *testing.T, want meshSnapshot, m *Mesh) {
	t.Helper()
	require.Empty(t, meshSnapshotChange(want, snapshotMesh(m)),
		"the source mesh must be unchanged by the transport")
}

// meshSnapshotChange reports one field name, so a failed isolation check does
// not format a shared topology graph through testify.
func meshSnapshotChange(want, got meshSnapshot) string {
	switch {
	case !slices.Equal(want.source, got.source):
		return "source faces"
	case !maps.Equal(want.faceBound, got.faceBound):
		return "face bounds"
	case !slices.Equal(want.triangles, got.triangles):
		return "triangles"
	case !slices.Equal(want.vertices, got.vertices):
		return "vertices"
	case !slices.Equal(want.vertexBound, got.vertexBound):
		return "vertex bounds"
	case want.scalars != got.scalars:
		return "scalar proofs"
	default:
		return ""
	}
}

// The combined transport must detach the source's face-bound map before
// restating bounds from a placed triangle.
func TestSpiralCombinedProxyMeshKeepsOneTriangleSourceIntact(t *testing.T) {
	face := &topo.Face{}
	face.Owner = face // A topology owner cycle must never enter a failure message.
	original := bornMesh(&Mesh{n: topo.Mesh{
		Vertices:    []r3.Vec{{}, {X: 1}, {Y: 1}},
		Triangles:   [][3]int{{0, 1, 2}},
		Source:      []*topo.Face{face},
		VertexBound: []float64{0.2, 0.3, 0.4},
		FaceBound:   map[*topo.Face]float64{face: 0.4},
		Bound:       0.4,
		CoordBound:  0.2,
		VolSymDiff:  1,
		AreaSlack:   2,
	}})
	want := snapshotMesh(original)
	placed := facetproof.PlacedMesh{
		Verts:       []r3.Vec{{}, {X: 2}, {Y: 2}},
		VertexBound: []float64{0.5, 0.7, 0.6},
		VolSymDiff:  3,
		AreaSlack:   4,
	}
	next, err := spiralCombinedProxyMesh(original, []facetproof.PlacedMesh{placed})
	require.NoError(t, err)
	require.True(t, next != original, "the transport must return a separate mesh handle")
	require.Equal(t, 0.4, original.n.FaceBound[face], "the source face bound stays unchanged")
	requireMeshSnapshot(t, want, original)
	require.Equal(t, 0.7, next.n.FaceBound[face], "the placed triangle sets its face bound")
	require.Equal(t, 0.7, next.n.Bound, "the global bound follows the face bound")
	require.Equal(t, 0.7, next.n.CoordBound, "the coordinate bound covers the new bound")
	require.Equal(t, math.Nextafter(3, math.Inf(1)), next.n.VolSymDiff,
		"the placed volume proof rounds upward")
	require.Equal(t, math.Nextafter(4, math.Inf(1)), next.n.AreaSlack,
		"the placed area proof rounds upward")
	require.True(t, slices.Equal(placed.VertexBound, next.n.VertexBound),
		"the placed vertex bounds are retained")
	require.True(t, next.n.Source[0] == face, "the triangle keeps its source face")
	next.n.FaceBound[face] = 9
	requireMeshSnapshot(t, want, original)
	changed := snapshotMesh(original)
	changed.faceBound[face] = 9
	require.Equal(t, "face bounds", meshSnapshotChange(want, changed),
		"a failed check reports a bounded field name")
}

// spiralNonTangentBlankAndTooth builds the revolved blank and the cone-trimmed
// spiral tooth, the root twin of apitest's spiralBlankAndTooth.
func spiralNonTangentBlankAndTooth(t *testing.T, doc *Document) (*Body, *Body) {
	t.Helper()
	d := spiralDerive()
	section := spiralBlankSection(d)
	blank := spiralRevolve(t, doc, section)

	point := spiralPointTooth(t, doc, d, spiralNonTangentCapTilt)
	f := spiralSlabFrame(section)
	slabs := spiralSlabs(t, doc, d, f, point)
	total := spiralTraceTotal(t, d, f)
	twisted, _ := spiralTwistSlabs(t, slabs, f, total)
	crowned := spiralCrownSlabs(t, twisted, f, total)
	curved, err := LoftFaces(t.Context(), doc, spiralLoftSections(t, crowned, f)...)
	require.NoError(t, err)
	for _, slab := range crowned {
		require.NoError(t, Remove(slab))
	}
	endCone := func(apex float64) *Body {
		far := -0.1 * d.coneDist
		radius := (apex - far) / math.Tan(d.coneAngle)
		return spiralRevolve(t, doc, []spiralAxial{{station: far}, {station: far, radius: radius}, {station: apex}})
	}
	toeKeeper, err := Cut(t.Context(), curved, endCone(section[4].station+section[4].radius*math.Tan(d.coneAngle)))
	require.NoError(t, err)
	trimmed, err := Intersect(t.Context(), toeKeeper, endCone(d.coneDist/math.Cos(d.coneAngle)))
	require.NoError(t, err)
	return blank, trimmed
}

// toothSourceIDs numbers the tooth's faces after the blank's, as the combined
// Union does, and maps the mesh's source faces to those ids.
func toothSourceIDs(ctx context.Context, t *testing.T, blank, tooth *Body, m *Mesh) []int {
	t.Helper()
	faceID := make(map[*topo.Face]int)
	for i, face := range tooth.Faces() {
		faceID[faceNode(face)] = len(blank.Faces()) + i
	}
	src, err := meshbool.SourceIDs(ctx, m.n.Source, faceID)
	require.NoError(t, err)
	return src
}

// The offset transport copies a tooth mesh and rewrites its vertices, vertex
// bounds and per-face bounds. The tooth mesh it was handed (possibly the
// body's cached one) keeps every field it had.
func TestSpiralOffsetToothMeshLeavesOriginalIntact(t *testing.T) {
	doc := New()
	blank, tooth := spiralNonTangentBlankAndTooth(t, doc)
	proof, ok, err := certifySpiralFirstJoin(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	original, _, err := tessellateBooleanOperand(t.Context(), tooth, 0.1)
	require.NoError(t, err)
	want := snapshotMesh(original)
	require.NotEmpty(t, want.faceBound)

	next, ok, err := spiralOffsetToothMesh(t.Context(), blank, tooth, original,
		toothSourceIDs(t.Context(), t, blank, tooth, original), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, original != next, "the proxy must have a separate mesh handle")

	requireMeshSnapshot(t, want, original)
	require.False(t, maps.Equal(want.faceBound, next.n.FaceBound), "the proxy states larger face bounds")
	require.False(t, slices.Equal(want.vertices, next.n.Vertices), "the proxy moves vertices")
	require.True(t, slices.Equal(want.source, next.n.Source), "the proxy keeps the source faces")
}

// The combined transport copies the combined tooth operand's mesh, drops its
// face bounds and restates its vertices and bounds from the proxy. The operand
// mesh it was handed keeps every field it had.
func TestSpiralPrepareCombinedUnionLeavesOriginalIntact(t *testing.T) {
	doc := New()
	blank, tooth := spiralNonTangentBlankAndTooth(t, doc)
	copied, err := Copy(t.Context(), tooth, WithCopyRotation(r3.Vec{}, r3.Vec{X: 1},
		units.Radians(2*math.Pi*23/spiralTeeth)))
	require.NoError(t, err)
	source, err := spiralCombineInputs(t.Context(), blank, []*Body{tooth, copied})
	require.NoError(t, err)
	combined, err := spiralCombinedToolBody(t.Context(), doc, producerID(0), source)
	require.NoError(t, err)
	original, _, err := tessellateBooleanOperand(t.Context(), combined, 0.1)
	require.NoError(t, err)
	want := snapshotMesh(original)
	require.NotEmpty(t, want.faceBound)

	next, meta, err := spiralPrepareCombinedUnion(t.Context(), meshbool.OpUnion, blank, combined, original, 0.1)
	require.NoError(t, err)
	require.NotNil(t, meta)
	require.True(t, original != next, "the proxy must have a separate mesh handle")

	requireMeshSnapshot(t, want, original)
	require.True(t, slices.Equal(want.source, next.n.Source), "the proxy keeps the source faces")
	require.NotEmpty(t, next.n.FaceBound, "setVertexBounds restates the face bounds")
}
