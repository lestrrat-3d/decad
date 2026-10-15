package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/stretchr/testify/require"
)

func TestCurvedContactGraphHeldAnchors(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.pairs, len(blank.Faces())*len(trimmed.Faces()))
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.pairs, len(census.pairs))
	for i, pair := range census.pairs {
		require.Equal(t, pair.verdict, graph.pairs[i].verdict)
		require.Equal(t, pair.blankFace, graph.pairs[i].blankFace)
		require.Equal(t, pair.trimmedFace, graph.pairs[i].trimmedFace)
		require.Len(t, graph.pairs[i].cellIDs,
			len(pair.sourceCells)+len(pair.verticalCells))
	}
	expectedCells := 0
	for _, side := range census.manifest.nonvertical {
		expectedCells += len(side.cells)
	}
	for _, side := range census.manifest.vertical {
		if side.side != nil {
			expectedCells += len(side.side.cells)
		}
	}
	require.Len(t, graph.cells, expectedCells)
	seenCells := make(map[int]bool)
	for pairID, pair := range graph.pairs {
		for _, cellID := range pair.cellIDs {
			require.False(t, seenCells[cellID])
			seenCells[cellID] = true
			require.Equal(t, pairID, graph.cells[cellID].pairID)
			require.Equal(t, pair.blankFace, graph.cells[cellID].blankFace)
			require.Equal(t, pair.trimmedFace, graph.cells[cellID].trimmedFace)
		}
	}
	for cellID, cell := range graph.cells {
		if cell.trimmedFace == nil {
			require.Equal(t, -1, cell.pairID)
			require.False(t, seenCells[cellID])
		} else {
			require.True(t, seenCells[cellID])
		}
	}
	require.Len(t, graph.runs, 2)
	seenEventIDs := make(map[int]bool)
	for _, arc := range graph.arcs {
		for _, id := range arc.endpointIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
	}
	seenStationIDs := make(map[int]bool)
	for _, run := range graph.runs {
		for _, id := range run.stationIDs {
			require.False(t, seenEventIDs[id])
			seenEventIDs[id] = true
		}
		cell := graph.cells[run.cellID]
		require.NotNil(t, cell.source)
		mapped := cell.source.mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolveLoftFacesCertifiedArcPatches(t.Context(),
			blank, trimmed, cell.sectionIndex, mapped.segment, mapped.band,
			loftFaceInterval{Lo: mapped.tLo, Hi: mapped.tHi}, run.stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		shell, built, shellErr := revolveLoftFacesBlankShellWithCapRings(t.Context(), proof,
			cell.sectionIndex, cell.cellIndex)
		require.NoError(t, shellErr)
		require.True(t, built)
		require.Equal(t, run.stationIDs, shell.seamIDs)
		require.Equal(t, run.stationIDs, blankPatch.stationIDs)
		require.Equal(t, run.stationIDs, loftPatch.stationIDs)
		require.Len(t, run.stationIDs, len(run.stations))
		seenShellVertices := make(map[int]bool)
		for i, id := range run.stationIDs {
			require.False(t, seenStationIDs[id])
			seenStationIDs[id] = true
			shellIndex, blankIndex, loftIndex := shell.seam[i], blankPatch.seam[i], loftPatch.seam[i]
			require.False(t, seenShellVertices[shellIndex])
			seenShellVertices[shellIndex] = true
			require.Equal(t, seam[i].point, shell.mesh.Verts[shellIndex])
			require.Equal(t, seam[i].point, blankPatch.vertices[blankIndex])
			require.Equal(t, seam[i].point, loftPatch.vertices[loftIndex])
			require.GreaterOrEqual(t, shell.vertexBounds[shellIndex], seam[i].blankBound)
			require.GreaterOrEqual(t, blankPatch.vertexBounds[blankIndex], seam[i].blankBound)
			require.GreaterOrEqual(t, loftPatch.vertexBounds[loftIndex], seam[i].loftBound)
		}
		require.Len(t, shell.capRings, len(graph.arcs))
		seenEndpointIDs, seenRings := make(map[int]bool), make(map[int]bool)
		for _, arc := range graph.arcs {
			matched := -1
			for i, ring := range shell.capRings {
				if revolveLoftFacesToolCapOccupiedArcEqual(arc.arc, ring.arc) {
					require.Equal(t, -1, matched)
					matched = i
				}
			}
			require.GreaterOrEqual(t, matched, 0)
			require.False(t, seenRings[matched])
			seenRings[matched] = true
			ring := shell.capRings[matched]
			require.Equal(t, arc.endpointIDs, ring.endpointGraphIDs)
			require.Equal(t, arc.endpointCellIDs, ring.endpointCellIDs)
			for endpoint, cellID := range arc.endpointCellIDs {
				id := arc.endpointIDs[endpoint]
				require.False(t, seenEndpointIDs[id])
				seenEndpointIDs[id] = true
				require.Equal(t, ring.endpointProof[endpoint].point,
					shell.mesh.Verts[ring.endpoints[endpoint]])
				require.GreaterOrEqual(t, graph.cells[cellID].pairID, 0)
				require.True(t, graph.endpointCellMatches(arc, &ring, endpoint, cellID))
			}
		}
		require.Len(t, seenRings, len(graph.arcs))
	}
	require.Len(t, seenStationIDs, 36)
	require.Len(t, seenEventIDs, 36+2*len(graph.arcs))
	current, err := revolveLoftFacesContactGraphCurrent(t.Context(), graph)
	require.NoError(t, err)
	require.True(t, current)
}

func TestCurvedContactGraphRejectsStaleSourceEvents(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	current := func() bool {
		live, liveErr := revolveLoftFacesContactGraphCurrent(t.Context(), graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.cells
		graph.cells = slices.Clone(original[:len(original)-1])
		require.False(t, current())
		graph.cells = original
	})
	t.Run("duplicate endpoint owner", func(t *testing.T) {
		original := graph.arcs[0].endpointCellIDs
		graph.arcs[0].endpointCellIDs[1] = original[0]
		require.False(t, current())
		graph.arcs[0].endpointCellIDs = original
	})
	t.Run("cross-kind event ID", func(t *testing.T) {
		original := graph.arcs[0].endpointIDs
		graph.arcs[0].endpointIDs[0] = graph.runs[0].stationIDs[0]
		require.False(t, current())
		graph.arcs[0].endpointIDs = original
	})
	t.Run("swapped face", func(t *testing.T) {
		original := graph.cells[0].trimmedFace
		graph.cells[0].trimmedFace = graph.pairs[0].trimmedFace
		if graph.cells[0].trimmedFace == original {
			graph.cells[0].trimmedFace = graph.pairs[1].trimmedFace
		}
		require.False(t, current())
		graph.cells[0].trimmedFace = original
	})
	t.Run("changed source ref", func(t *testing.T) {
		original := graph.cells[0].source.ref
		graph.cells[0].source.ref = provenance.Ref{Role: "changed graph source"}
		require.False(t, current())
		graph.cells[0].source.ref = original
	})
	t.Run("changed open interval", func(t *testing.T) {
		original := graph.cells[0].source.mapped.tLoOpen
		graph.cells[0].source.mapped.tLoOpen = !original
		require.False(t, current())
		graph.cells[0].source.mapped.tLoOpen = original
	})
	t.Run("changed live station", func(t *testing.T) {
		original := trim.source.sourceStation[0]
		trim.source.sourceStation[0].U += 0.001
		require.False(t, current())
		trim.source.sourceStation[0] = original
	})
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.arcs[0].arc.negativeZ.parameter.Lo
		graph.arcs[0].arc.negativeZ.parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.arcs[0].arc.negativeZ.parameter.Lo = original
	})
	require.True(t, current())
}
