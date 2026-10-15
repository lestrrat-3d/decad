package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedSourceCapCirclesSplitClosedBlankShell(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	sectionIndex, cellIndex := -1, -1
	for _, side := range manifest.nonvertical {
		for index, cell := range side.cells {
			if cell.mapped.side == revolveLoftFacesJointContact &&
				cell.mapped.jointKind == revolveLoftFacesWholeArcContact {
				sectionIndex, cellIndex = side.sectionIndex, index
				break
			}
		}
		if cellIndex >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, cellIndex, 0)
	shell, ok, err := revolveLoftFacesBlankShellWithCapRings(t.Context(), proof,
		sectionIndex, cellIndex)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, shell.capRings, 2)
	require.Len(t, shell.vertexBounds, len(shell.mesh.Verts))
	require.Less(t, shell.faceBound, 0.1)
	require.True(t, revolveLoftFacesBlankShellEulerTwo(&shell.mesh))
	edges := make(map[[2]int]struct{})
	for _, triangle := range shell.mesh.Tris {
		for corner := range 3 {
			a, b := triangle[corner], triangle[(corner+1)%3]
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	require.Equal(t, 2, len(shell.mesh.Verts)-len(edges)+len(shell.mesh.Tris))
	vertical, sloped := 0, 0
	for _, ring := range shell.capRings {
		require.Len(t, ring.vertices, len(shell.capRings[0].vertices))
		require.Equal(t, ring.sectionIndex, ring.arc.circle.span)
		require.NotNil(t, ring.ties)
		require.NotEqual(t, ring.endpoints[0], ring.endpoints[1])
		require.GreaterOrEqual(t, len(ring.occupied), 2)
		require.Equal(t, ring.endpoints[0], ring.occupied[0])
		require.Equal(t, ring.endpoints[1], ring.occupied[len(ring.occupied)-1])
		throughAxis := false
		for _, vertex := range ring.occupied {
			point := shell.mesh.Verts[vertex]
			throughAxis = throughAxis || point.Y > 0 && point.Z == 0
		}
		require.Equal(t, ring.arc.throughPositiveY, throughAxis)
		for side, endpoint := range ring.endpoints {
			require.True(t, slices.Contains(ring.vertices, endpoint))
			require.Less(t, shell.vertexBounds[endpoint], 0.1)
			require.Equal(t, ring.endpointProof[side].point, shell.mesh.Verts[endpoint])
			require.Equal(t, ring.endpointProof[side].blankBound,
				shell.vertexBounds[endpoint])
			require.Less(t, ring.endpointProof[side].loftBound, 0.1)
		}
		require.True(t, ring.ties.negativeZ.sourceCell != nil ||
			ring.ties.negativeZ.verticalCell != nil)
		require.True(t, ring.ties.positiveZ.sourceCell != nil ||
			ring.ties.positiveZ.verticalCell != nil)
		x, radius := ring.arc.circle.x, ring.arc.circle.radius
		xf, _ := x.Float64()
		rf, _ := radius.Float64()
		span := proof.section.Spans[ring.sectionIndex]
		if span.Start.U == span.End.U {
			vertical++
		} else {
			sloped++
		}
		for _, index := range ring.vertices {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.mesh.Verts))
			point := shell.mesh.Verts[index]
			bound := shell.vertexBounds[index]
			require.InDelta(t, xf, point.X, bound+1e-12)
			require.InDelta(t, rf*rf, point.Y*point.Y+point.Z*point.Z,
				2*rf*bound+bound*bound+1e-12)
			incident := 0
			for _, triangle := range shell.mesh.Tris {
				if slices.Contains(triangle[:], index) {
					incident++
				}
			}
			require.GreaterOrEqual(t, incident, 4,
				"both split meridian bands must reuse cap-ring vertex %d", index)
		}
	}
	require.Equal(t, 1, vertical)
	require.Equal(t, 1, sloped)
	t.Logf("source-aligned blank shell with %d heel circles: %d vertices, %d triangles",
		len(shell.capRings), len(shell.mesh.Verts), len(shell.mesh.Tris))
	stale := *shell.capRings[0].arc
	stale.time = new(big.Rat).Add(stale.time, big.NewRat(1, 1000))
	_, accepted, err := revolveLoftFacesBlankShellAtArcWithCuts(t.Context(), proof, manifest,
		sectionIndex, cellIndex, []revolveLoftFacesShellCut{{
			sectionIndex: stale.circle.span,
			x:            stale.circle.x, radius: stale.circle.radius, arc: &stale,
		}})
	require.NoError(t, err)
	require.False(t, accepted)
}
