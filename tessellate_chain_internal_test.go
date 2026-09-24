package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalChainRibbonBody builds extrude_chain_test.go's own T132 fixture — a
// rectangle with one side erased, three straight walls — restated here since
// an internal test cannot import the decad_test package that owns it. Three
// walls is the smallest ribbon with more than one wall to attribute wrongly.
func internalChainRibbonBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 6)
	d := s.CreatePoint(0, 6)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	body, err := doc.ExtrudeChain(s, chains[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return body
}

// TestTessellateChainBuildsOneExactQuadPerWall is docs/surface-design.md
// §13.4's own claim, read off the mesh: every wall restates as two
// triangles, the whole ribbon publishes no occupied-volume proof, and an
// unplaced, axis-aligned chain's own corners are exact.
func TestTessellateChainBuildsOneExactQuadPerWall(t *testing.T) {
	t.Parallel()
	body := internalChainRibbonBody(t, New())
	pp, ok := body.payload.(chainPayload)
	require.True(t, ok)
	mesh, err := tessellateChain(t.Context(), body, pp)
	require.NoError(t, err)
	require.Len(t, mesh.triangles, 6, "three walls, two triangles each")
	require.Zero(t, mesh.bound, "an unplaced, axis-aligned chain's own vertices are exact")
	require.False(t, mesh.symDiffOK, "a sheet publishes no occupied-volume proof")
	require.Zero(t, mesh.volSymDiff)
}

// TestTessellateChainRejectsWronglyAttributedWalls is the free-edge
// attribution's own falsifier for the chain path, mirroring
// tessellate_sheet_internal_test.go's identical proof for a profile-fed
// sheet: the public API can never build a chain mesh whose free boundary
// attributes to the wrong wall — every triangle tessellateChain emits
// carries its own wall's Face — so this hand-corrupts a real chain mesh's
// SourceFaces to prove requireSheetMesh actually refuses a wrongly chorded
// wall set rather than passing vacuously.
//
// requireMatchingFreeAttribution compares CHAIN counts per face
// (tessellate_sheet.go), not raw edge identity, so reassigning a triangle to
// an ADJACENT wall can leave both faces' own chain counts unchanged — the
// stolen edge merely merges into a chain that is already there on one side,
// and the chain it leaves behind on the other stays one chain regardless.
// The corruption below instead moves a triangle to a wall that shares NO
// vertex with it at all: the stolen edge cannot merge with anything the
// target already has, so it always mints one new chain there, which is what
// makes the mismatch unconditional rather than an accident of which two
// distinct faces happened to be picked.
func TestTessellateChainRejectsWronglyAttributedWalls(t *testing.T) {
	t.Parallel()
	body := internalChainRibbonBody(t, New())
	pp, ok := body.payload.(chainPayload)
	require.True(t, ok)
	mesh, err := tessellateChain(t.Context(), body, pp)
	require.NoError(t, err)
	require.NoError(t, requireSheetMesh(t.Context(), body, mesh), "premise: the real mesh passes the audit as built")

	const moveIdx = 0
	moved := mesh.triangles[moveIdx]
	movedVerts := map[int]struct{}{moved[0]: {}, moved[1]: {}, moved[2]: {}}
	homeFace := mesh.source[moveIdx]

	var targetFace *Face
search:
	for i, f := range mesh.source {
		if f == homeFace {
			continue
		}
		for _, v := range mesh.triangles[i] {
			if _, ok := movedVerts[v]; ok {
				continue search
			}
		}
		targetFace = f
		break
	}
	require.NotNil(t, targetFace, "premise: the ribbon's far wall shares no vertex with the moved triangle")

	broken := *mesh
	broken.source = append([]*Face(nil), mesh.source...)
	broken.source[moveIdx] = targetFace
	err = requireSheetMesh(t.Context(), body, &broken)
	require.Error(t, err, "moving a triangle to a vertex-disjoint wall mints a new free-boundary chain there, which the body's own recorded count does not have")
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestTessellateChainReflectedPlacementFlipsWinding is this file's own
// reflection reasoning made concrete: a chain ribbon PLACED under a
// determinant-negative transform still produces a mesh whose triangles wind
// outward — CCW as seen from the wall's own positive side — never backward,
// which the payload's own reversed field never states for a straight wall
// (buildWallGeometry's default-case return stays false), so this proof
// cannot come from that field and instead confirms it directly against the
// wall's own Face.NormalAt.
func TestTessellateChainReflectedPlacementFlipsWinding(t *testing.T) {
	t.Parallel()
	body := internalChainRibbonBody(t, New())
	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	reflect, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), reflect)
	require.NoError(t, err)
	pp, ok := placed.payload.(chainPayload)
	require.True(t, ok)
	require.True(t, pp.xform.IsReflection(), "premise: the placement negates the determinant")

	mesh, err := tessellateChain(t.Context(), placed, pp)
	require.NoError(t, err)
	for i, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		normal := b.Sub(a).Cross(c.Sub(a))
		face := mesh.source[i]
		published, err := face.NormalAt(a)
		require.NoError(t, err)
		require.Greater(t, normal.Dot(published.Value), 0.0,
			"a reflected wall's own triangle must still wind outward, toward its published positive-side normal")
	}
}
