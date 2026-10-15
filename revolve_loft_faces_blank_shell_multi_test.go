package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCurvedMultiRunShellConsumesLiveGraph(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, certified)
	shell, graph, built, err := revolveLoftFacesBlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, built)
	require.Len(t, graph.runs, 2)
	require.Len(t, graph.arcs, 2)
	require.Len(t, shell.seams, len(graph.runs))
	require.Len(t, shell.capRings, len(graph.arcs))
	require.Len(t, shell.triangleCells, len(shell.mesh.Tris))
	require.Less(t, shell.faceBound, 0.1)
	require.True(t, revolveLoftFacesBlankShellEulerTwo(&shell.mesh))
	seenIDs := make(map[int]struct{})
	seenVertices := make(map[int]struct{})
	for _, arc := range graph.arcs {
		for _, id := range arc.endpointIDs {
			_, duplicate := seenIDs[id]
			require.False(t, duplicate)
			seenIDs[id] = struct{}{}
		}
	}
	for i, run := range graph.runs {
		require.Equal(t, run.stationIDs, shell.seamRunIDs[i])
		require.Len(t, shell.seams[i], len(run.stations))
		cell := graph.cells[run.cellID]
		mapped := cell.source.mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolveLoftFacesCertifiedArcPatches(t.Context(),
			blank, trimmed, cell.sectionIndex, mapped.segment, mapped.band,
			loftFaceInterval{Lo: mapped.tLo, Hi: mapped.tHi}, run.stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		for station, index := range shell.seams[i] {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.mesh.Verts))
			_, duplicate := seenVertices[index]
			require.False(t, duplicate)
			seenVertices[index] = struct{}{}
			id := run.stationIDs[station]
			_, duplicate = seenIDs[id]
			require.False(t, duplicate)
			seenIDs[id] = struct{}{}
			require.Equal(t, seam[station].point, shell.mesh.Verts[index])
			require.Equal(t, seam[station].point,
				blankPatch.vertices[blankPatch.seam[station]])
			require.Equal(t, seam[station].point,
				loftPatch.vertices[loftPatch.seam[station]])
			require.GreaterOrEqual(t, shell.vertexBounds[index], seam[station].blankBound)
			require.GreaterOrEqual(t,
				blankPatch.vertexBounds[blankPatch.seam[station]], seam[station].blankBound)
			require.GreaterOrEqual(t,
				loftPatch.vertexBounds[loftPatch.seam[station]], seam[station].loftBound)
		}
	}
	require.Len(t, seenVertices, 36)
	require.Len(t, seenIDs, 36+2*len(graph.arcs))
	for _, arc := range graph.arcs {
		matched := -1
		for i, ring := range shell.capRings {
			if revolveLoftFacesToolCapOccupiedArcEqual(arc.arc, ring.arc) {
				require.Equal(t, -1, matched)
				matched = i
			}
		}
		require.GreaterOrEqual(t, matched, 0)
		ring := shell.capRings[matched]
		require.Equal(t, arc.endpointIDs, ring.endpointGraphIDs)
		require.Equal(t, arc.endpointCellIDs, ring.endpointCellIDs)
		for endpoint, cellID := range arc.endpointCellIDs {
			index := ring.endpoints[endpoint]
			require.Equal(t, ring.endpointProof[endpoint].point, shell.mesh.Verts[index])
			require.GreaterOrEqual(t, shell.vertexBounds[index], ring.endpointProof[endpoint].blankBound)
			require.True(t, graph.endpointCellMatches(arc, &ring, endpoint, cellID))
		}
	}
	sectionGroup := make([]int, len(proof.section.spans))
	group := 0
	for i, span := range proof.section.spans {
		sectionGroup[i] = -1
		if span.face != nil {
			sectionGroup[i] = group
			group++
		}
	}
	seenCells := make(map[revolveLoftFacesShellTriangleCell]struct{})
	for triIndex, cell := range shell.triangleCells {
		require.GreaterOrEqual(t, cell.section, 0)
		require.Less(t, cell.section, len(shell.sectionRings))
		require.GreaterOrEqual(t, cell.band, 0)
		require.Less(t, cell.band+1, len(shell.sectionRings[cell.section]))
		require.GreaterOrEqual(t, cell.ray, 0)
		require.Less(t, cell.ray, shell.rayCount)
		require.Equal(t, sectionGroup[cell.section], shell.mesh.Src[triIndex])
		_, duplicate := seenCells[cell]
		require.False(t, duplicate)
		seenCells[cell] = struct{}{}
		start := shell.sectionRings[cell.section][cell.band]
		end := shell.sectionRings[cell.section][cell.band+1]
		next := (cell.ray + 1) % shell.rayCount
		var want [3]int
		switch {
		case len(start) == 1:
			want = [3]int{start[0], end[cell.ray], end[next]}
		case len(end) == 1:
			want = [3]int{start[cell.ray], end[0], start[next]}
		case cell.half == 0:
			want = [3]int{start[cell.ray], end[cell.ray], end[next]}
		default:
			want = [3]int{start[cell.ray], end[next], start[next]}
		}
		got := shell.mesh.Tris[triIndex]
		slices.Sort(got[:])
		slices.Sort(want[:])
		require.Equal(t, want, got)
	}
}

func TestCurvedMultiRunShellRayOwnersRefuseCollisions(t *testing.T) {
	station := revolveLoftFacesSeamVertex{
		exact: loftFaceRatVec{lfF(0), lfF(2), lfF(1)},
		point: r3.Vec{Y: 2, Z: 1}, loftBound: 0.001,
	}
	seams := [][]revolveLoftFacesSeamVertex{{station}, {station}}
	rays := revolveLoftFacesShellRaysForRuns(16, seams, nil)
	require.NotNil(t, rays)
	require.True(t, revolveLoftFacesAllRayOrderBounded(rays, seams, nil))
	shared := 0
	for _, ray := range rays {
		if len(ray.seamRefs) == 2 {
			shared++
		}
	}
	require.Equal(t, 1, shared)
	require.Nil(t, revolveLoftFacesShellRaysForRuns(16,
		[][]revolveLoftFacesSeamVertex{{station, station}}, nil))
	require.Nil(t, revolveLoftFacesShellRaysForRuns(16, seams,
		[]revolveLoftFacesCapSeamVertex{{point: r3.Vec{Y: 2, Z: 1}}}))
}
