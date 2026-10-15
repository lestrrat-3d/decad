package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCurvedVerticalTraceConsumesLiveGraphAndShell(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	selected := -1
	for id, cell := range graph.cells {
		if cell.vertical == nil || len(cell.vertical.retained) == 0 || cell.pairID < 0 {
			continue
		}
		for _, arc := range graph.arcs {
			if arc.sectionIndex == cell.sectionIndex &&
				id != arc.endpointCellIDs[0] && id != arc.endpointCellIDs[1] {
				selected = id
				break
			}
		}
		if selected >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, selected, 0)
	shell, boundGraph, trace, built, err := revolveLoftFacesBlankShellWithVerticalTrace(
		t.Context(), proof, selected)
	require.NoError(t, err)
	require.True(t, built)
	require.NotNil(t, shell)
	require.NotNil(t, boundGraph)
	require.NotNil(t, trace)
	require.Contains(t, trace.cellIDs, selected)
	require.Len(t, shell.seams, len(boundGraph.runs))
	require.Len(t, shell.capRings, len(boundGraph.arcs))
	require.Len(t, shell.triangleCells, len(shell.mesh.Tris))
	require.True(t, revolveLoftFacesBlankShellEulerTwo(&shell.mesh))
	require.Less(t, shell.faceBound, 0.1)
	require.Less(t, trace.loftBound, 0.1)
	require.Greater(t, len(trace.vertices), 2)
	require.Len(t, trace.edgeWedges, len(trace.vertices)-1)
	require.Len(t, trace.edgeCellIDs, len(trace.edgeWedges))
	require.Len(t, trace.vertexRays, len(trace.vertices))
	seenCells := make(map[revolveLoftFacesShellTriangleCell]bool)
	incident := make(map[[2]int][]revolveLoftFacesShellTriangleCell)
	for i, tri := range shell.mesh.Tris {
		cell := shell.triangleCells[i]
		require.False(t, seenCells[cell])
		seenCells[cell] = true
		require.GreaterOrEqual(t, cell.ray, 0)
		require.Less(t, cell.ray, shell.rayCount)
		for side := range 3 {
			edge := [2]int{tri[side], tri[(side+1)%3]}
			slices.Sort(edge[:])
			incident[edge] = append(incident[edge], cell)
		}
	}
	seenRayContacts := make(map[int]int)
	for i, wedge := range trace.edgeWedges {
		edge := [2]int{trace.vertices[i], trace.vertices[i+1]}
		slices.Sort(edge[:])
		require.Len(t, incident[edge], 2)
		for _, cell := range incident[edge] {
			require.Equal(t, trace.sectionIndex, cell.section)
			require.Equal(t, wedge, cell.ray)
		}
		require.Contains(t, trace.cellIDs, trace.edgeCellIDs[i])
		seenRayContacts[wedge]++
	}
	require.Greater(t, seenRayContacts[trace.edgeWedges[0]], 1)
	var capRing *revolveLoftFacesShellCapRing
	for i := range shell.capRings {
		candidate := &shell.capRings[i]
		if candidate.sectionIndex == trace.sectionIndex && candidate.endpointGraphIDs == trace.capIDs {
			capRing = candidate
			break
		}
	}
	require.NotNil(t, capRing)
	require.Len(t, trace.cellIDs, 3)
	for endpoint, capVertex := range capRing.endpoints {
		capRay := slices.Index(capRing.vertices, capVertex)
		require.GreaterOrEqual(t, capRay, 0)
		contacts := make([]int, 0, 2)
		for i, ray := range trace.vertexRays {
			if ray == capRay {
				contacts = append(contacts, i)
			}
		}
		require.Len(t, contacts, 2, "each cap ray must retain its inner arc contact")
		inner := contacts[1-endpoint]
		require.Greater(t, inner, 0)
		require.Less(t, inner, len(trace.vertices)-1)
		require.Equal(t, trace.cellIDs[1], trace.edgeCellIDs[inner-1])
		require.Equal(t, trace.cellIDs[1], trace.edgeCellIDs[inner])
		require.NotEqual(t, trace.edgeWedges[inner-1], trace.edgeWedges[inner])
	}
	t.Logf("vertical trace: %d cells, %d vertices, bound %.9g mm; shell %d vertices, %d triangles",
		len(trace.cellIDs), len(trace.vertices), trace.loftBound,
		len(shell.mesh.Verts), len(shell.mesh.Tris))
}

func TestVerticalTraceRefusesChangedSourceAndRayOwners(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	shell, graph, ok, err := revolveLoftFacesBlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	var arc *revolveLoftFacesGraphArc
	var ring *revolveLoftFacesShellCapRing
	for i := range graph.arcs {
		candidate := &graph.arcs[i]
		if proof.section.Spans[candidate.sectionIndex].Start.U ==
			proof.section.Spans[candidate.sectionIndex].End.U {
			arc = candidate
			break
		}
	}
	require.NotNil(t, arc)
	for i := range shell.capRings {
		if shell.capRings[i].sectionIndex == arc.sectionIndex {
			ring = &shell.capRings[i]
			break
		}
	}
	require.NotNil(t, ring)
	pieces, chained := revolveLoftFacesVerticalTraceChain(graph, arc)
	require.True(t, chained)
	require.True(t, revolveLoftFacesVerticalTraceSourceProof(pieces, arc.arc.circle.radius))

	copyGraph := func() *revolveLoftFacesContactGraph {
		clone := *graph
		clone.cells = append([]revolveLoftFacesGraphCell(nil), graph.cells...)
		return &clone
	}
	bad := copyGraph()
	owner := arc.endpointCellIDs[0]
	vertical := *bad.cells[owner].vertical
	vertical.retained = append([]loftface.VerticalPiece(nil), vertical.retained...)
	vertical.retained[0].LoOpen = true
	bad.cells[owner].vertical = &vertical
	_, chained = revolveLoftFacesVerticalTraceChain(bad, arc)
	require.False(t, chained, "an open retained root cannot own a cap endpoint")

	bad = copyGraph()
	vertical = *bad.cells[owner].vertical
	vertical.upper.Root.Lo = lfZero()
	bad.cells[owner].vertical = &vertical
	_, chained = revolveLoftFacesVerticalTraceChain(bad, arc)
	require.False(t, chained, "a changed root interval loses its graph-owned cap tie")

	duplicate := *arc
	duplicate.endpointCellIDs[1] = duplicate.endpointCellIDs[0]
	_, chained = revolveLoftFacesVerticalTraceChain(graph, &duplicate)
	require.False(t, chained)
	missing := *arc
	missing.endpointCellIDs[0] = -1
	_, chained = revolveLoftFacesVerticalTraceChain(graph, &missing)
	require.False(t, chained)

	wrongCenter := *pieces[1].affine
	wrongCenter.Offset[0] = lfAdd(wrongCenter.Offset[0], lfOne())
	_, centered := revolveLoftFacesVerticalTraceArcRadius(&wrongCenter, *pieces[1].arc)
	require.False(t, centered, "a moved Sketch arc is not centered on the blank axis")
	payload := tooth.n.Payload.(facetedPayload)
	trim := *payload.LoftFacesTrim
	source := *trim.Source
	live := *source.Source
	live.Entities = slices.Clone(live.Entities)
	live.Entities[pieces[1].cell.segment] = nil
	source.Source = &live
	trim.Source = &source
	payload.LoftFacesTrim = &trim
	partialTooth := bornBody(&Body{n: topo.Body{Payload: payload}})
	partialGraph := *graph
	partialGraph.tooth = partialTooth
	_, chained = revolveLoftFacesVerticalTraceChain(&partialGraph, arc)
	require.False(t, chained, "a partial live Sketch cannot supply the arc contact")

	capRays := [2]int{-1, -1}
	for endpoint := range capRays {
		capRays[endpoint] = slices.Index(ring.vertices, ring.endpoints[endpoint])
		require.GreaterOrEqual(t, capRays[endpoint], 0)
	}
	rays := append([]revolveLoftFacesRay(nil), shell.rays...)
	arcHits, hitsOK := revolveLoftFacesTraceSourceHits(pieces[1], rays, [2]int{-1, -1})
	require.True(t, hitsOK)
	require.Greater(t, len(arcHits), 2)
	rays[arcHits[1].ray] = rays[arcHits[0].ray]
	_, hitsOK = revolveLoftFacesTraceSourceHits(pieces[1], rays, [2]int{-1, -1})
	require.False(t, hitsOK, "two ray owners cannot claim the same arc crossing")

	badRing := *ring
	badRing.endpoints[1] = badRing.endpoints[0]
	pathNodes, pathEdges, pathBound, built, err := revolveLoftFacesVerticalTracePath(t.Context(),
		proof, shell, pieces, badRing)
	require.NoError(t, err)
	require.False(t, built, "a repeated cap endpoint cannot start a trace")
	require.Empty(t, pathNodes)
	require.Empty(t, pathEdges)
	require.Zero(t, pathBound)
	startRay := shell.rays[capRays[0]]
	join, _, direction, joined := revolveLoftFacesTraceJoin(proof, pieces[0], pieces[0].to)
	require.True(t, joined)
	moved := shell.mesh.Verts[ring.endpoints[0]]
	moved.Y++
	motionBound, bounded := revolveLoftFacesVerticalTraceSegmentBound(proof, pieces[0],
		revolveLoftFacesVerticalTraceHit{piece: 0, ray: capRays[0],
			lo: pieces[0].cell.upper.Root.Lo, hi: pieces[0].cell.upper.Root.Hi},
		revolveLoftFacesVerticalTraceHit{piece: 0, ray: -1,
			lo: pieces[0].to, hi: pieces[0].to},
		[2]*big.Rat{startRay.y, startRay.z}, direction, moved, join, shell.rays)
	require.True(t, bounded)
	require.Greater(t, motionBound, 0.1, "a moved endpoint exceeds the trace fit limit")

	badProof := *proof
	badProof.tooth = nil
	_, built, err = revolveLoftFacesInsertVerticalTrace(t.Context(), &badProof,
		shell, graph, arc.endpointCellIDs[0])
	require.NoError(t, err)
	require.False(t, built, "a stale source certificate cannot move the shell")
	bad = copyGraph()
	bad.cells[owner].sideIndex = -1
	_, built, err = revolveLoftFacesInsertVerticalTrace(t.Context(), proof,
		shell, bad, owner)
	require.NoError(t, err)
	require.False(t, built, "a stale graph cannot move the shell")
}
