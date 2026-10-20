package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCurvedVerticalTraceConsumesLiveGraphAndShell(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolvecontact.ContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	selected := -1
	for id, cell := range graph.Cells {
		if cell.Vertical == nil || len(cell.Vertical.Retained) == 0 || cell.PairID < 0 {
			continue
		}
		for _, arc := range graph.Arcs {
			if arc.SectionIndex == cell.SectionIndex &&
				id != arc.EndpointCellIDs[0] && id != arc.EndpointCellIDs[1] {
				selected = id
				break
			}
		}
		if selected >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, selected, 0)
	shell, boundGraph, trace, built, err := revolvecontact.BlankShellWithVerticalTrace(
		t.Context(), proof, selected)
	require.NoError(t, err)
	require.True(t, built)
	require.NotNil(t, shell)
	require.NotNil(t, boundGraph)
	require.NotNil(t, trace)
	require.Contains(t, trace.CellIDs, selected)
	require.Len(t, shell.Seams, len(boundGraph.Runs))
	require.Len(t, shell.CapRings, len(boundGraph.Arcs))
	require.Len(t, shell.TriangleCells, len(shell.Mesh.Tris))
	require.True(t, shell.EulerTwo())
	require.Less(t, shell.FaceBound, 0.1)
	require.Less(t, trace.LoftBound, 0.1)
	require.Greater(t, len(trace.Vertices), 2)
	require.Len(t, trace.EdgeWedges, len(trace.Vertices)-1)
	require.Len(t, trace.EdgeCellIDs, len(trace.EdgeWedges))
	require.Len(t, trace.VertexRays, len(trace.Vertices))
	seenCells := make(map[revolveloftfaces.ShellTriangleCell]bool)
	incident := make(map[[2]int][]revolveloftfaces.ShellTriangleCell)
	for i, tri := range shell.Mesh.Tris {
		cell := shell.TriangleCells[i]
		require.False(t, seenCells[cell])
		seenCells[cell] = true
		require.GreaterOrEqual(t, cell.Ray, 0)
		require.Less(t, cell.Ray, shell.RayCount)
		for side := range 3 {
			edge := [2]int{tri[side], tri[(side+1)%3]}
			slices.Sort(edge[:])
			incident[edge] = append(incident[edge], cell)
		}
	}
	seenRayContacts := make(map[int]int)
	for i, wedge := range trace.EdgeWedges {
		edge := [2]int{trace.Vertices[i], trace.Vertices[i+1]}
		slices.Sort(edge[:])
		require.Len(t, incident[edge], 2)
		for _, cell := range incident[edge] {
			require.Equal(t, trace.SectionIndex, cell.Section)
			require.Equal(t, wedge, cell.Ray)
		}
		require.Contains(t, trace.CellIDs, trace.EdgeCellIDs[i])
		seenRayContacts[wedge]++
	}
	require.Greater(t, seenRayContacts[trace.EdgeWedges[0]], 1)
	var capRing *revolvecontact.ShellCapRing
	for i := range shell.CapRings {
		candidate := &shell.CapRings[i]
		if candidate.Section == trace.SectionIndex && candidate.EndpointGraphIDs == trace.CapIDs {
			capRing = candidate
			break
		}
	}
	require.NotNil(t, capRing)
	require.Len(t, trace.CellIDs, 3)
	for endpoint, capVertex := range capRing.Endpoints {
		capRay := slices.Index(capRing.Vertices, capVertex)
		require.GreaterOrEqual(t, capRay, 0)
		contacts := make([]int, 0, 2)
		for i, ray := range trace.VertexRays {
			if ray == capRay {
				contacts = append(contacts, i)
			}
		}
		require.Len(t, contacts, 2, "each cap ray must retain its inner arc contact")
		inner := contacts[1-endpoint]
		require.Greater(t, inner, 0)
		require.Less(t, inner, len(trace.Vertices)-1)
		require.Equal(t, trace.CellIDs[1], trace.EdgeCellIDs[inner-1])
		require.Equal(t, trace.CellIDs[1], trace.EdgeCellIDs[inner])
		require.NotEqual(t, trace.EdgeWedges[inner-1], trace.EdgeWedges[inner])
	}
	t.Logf("vertical trace: %d cells, %d vertices, bound %.9g mm; shell %d vertices, %d triangles",
		len(trace.CellIDs), len(trace.Vertices), trace.LoftBound,
		len(shell.Mesh.Verts), len(shell.Mesh.Tris))
}

func TestVerticalTraceRefusesChangedSourceAndRayOwners(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(tooth))
	require.NoError(t, err)
	require.True(t, ok)
	shell, graph, ok, err := revolvecontact.BlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	var arc *revolvecontact.GraphArc
	var ring *revolvecontact.ShellCapRing
	for i := range graph.Arcs {
		candidate := &graph.Arcs[i]
		if proof.Section.Spans[candidate.SectionIndex].Start.U ==
			proof.Section.Spans[candidate.SectionIndex].End.U {
			arc = candidate
			break
		}
	}
	require.NotNil(t, arc)
	for i := range shell.CapRings {
		if shell.CapRings[i].Section == arc.SectionIndex {
			ring = &shell.CapRings[i]
			break
		}
	}
	require.NotNil(t, ring)
	pieces, chained := revolvecontact.VerticalTraceChain(graph, arc)
	require.True(t, chained)
	sourcePieces := make([]revolveloftfaces.VerticalTraceSourcePiece, len(pieces))
	for i := range pieces {
		sourcePieces[i] = pieces[i].VerticalTraceSourcePiece
	}
	require.True(t, revolveloftfaces.VerticalTraceSourceProof(sourcePieces, arc.Arc.Circle.Radius))

	copyGraph := func() *revolvecontact.ContactGraph {
		clone := *graph
		clone.Cells = append([]revolvecontact.GraphCell(nil), graph.Cells...)
		return &clone
	}
	bad := copyGraph()
	owner := arc.EndpointCellIDs[0]
	vertical := *bad.Cells[owner].Vertical
	vertical.Retained = append([]loftface.VerticalPiece(nil), vertical.Retained...)
	vertical.Retained[0].LoOpen = true
	bad.Cells[owner].Vertical = &vertical
	_, chained = revolvecontact.VerticalTraceChain(bad, arc)
	require.False(t, chained, "an open retained root cannot own a cap endpoint")

	bad = copyGraph()
	vertical = *bad.Cells[owner].Vertical
	vertical.Upper.Root.Lo = lfZero()
	bad.Cells[owner].Vertical = &vertical
	_, chained = revolvecontact.VerticalTraceChain(bad, arc)
	require.False(t, chained, "a changed root interval loses its graph-owned cap tie")

	duplicate := *arc
	duplicate.EndpointCellIDs[1] = duplicate.EndpointCellIDs[0]
	_, chained = revolvecontact.VerticalTraceChain(graph, &duplicate)
	require.False(t, chained)
	missing := *arc
	missing.EndpointCellIDs[0] = -1
	_, chained = revolvecontact.VerticalTraceChain(graph, &missing)
	require.False(t, chained)

	wrongCenter := *pieces[1].Affine
	wrongCenter.Offset[0] = lfAdd(wrongCenter.Offset[0], lfOne())
	_, centered := revolveloftfaces.VerticalTraceArcRadius(&wrongCenter, *pieces[1].Arc)
	require.False(t, centered, "a moved Sketch arc is not centered on the blank axis")
	payload := tooth.n.Payload.(facetedPayload)
	trim := *payload.LoftFacesTrim
	source := *trim.Source
	live := *source.Source
	live.Entities = slices.Clone(live.Entities)
	live.Entities[pieces[1].Cell.Segment] = nil
	source.Source = &live
	trim.Source = &source
	payload.LoftFacesTrim = &trim
	partialTooth := bornBody(&Body{n: topo.Body{Payload: payload}})
	partialGraph := *graph
	partialGraph.Tooth = bodyNode(partialTooth)
	_, chained = revolvecontact.VerticalTraceChain(&partialGraph, arc)
	require.False(t, chained, "a partial live Sketch cannot supply the arc contact")

	capRays := [2]int{-1, -1}
	for endpoint := range capRays {
		capRays[endpoint] = slices.Index(ring.Vertices, ring.Endpoints[endpoint])
		require.GreaterOrEqual(t, capRays[endpoint], 0)
	}
	rays := append([]revolveloftfaces.ShellRay(nil), shell.Rays...)
	arcHits, hitsOK := revolveloftfaces.VerticalTraceSourceHits(sourcePieces[1], rays, [2]int{-1, -1})
	require.True(t, hitsOK)
	require.Greater(t, len(arcHits), 2)
	rays[arcHits[1].Ray] = rays[arcHits[0].Ray]
	_, hitsOK = revolveloftfaces.VerticalTraceSourceHits(sourcePieces[1], rays, [2]int{-1, -1})
	require.False(t, hitsOK, "two ray owners cannot claim the same arc crossing")

	badEndpoints := ring.Endpoints
	badEndpoints[1] = badEndpoints[0]
	path, built, err := revolveloftfaces.VerticalTracePath(t.Context(), proof.Trim.Maps,
		shell.Rays, shell.Mesh.Verts, shell.VertexBounds, ring.Vertices, badEndpoints,
		ring.Arc.Circle.Radius, sourcePieces)
	require.NoError(t, err)
	require.False(t, built, "a repeated cap endpoint cannot start a trace")
	require.Empty(t, path.Nodes)
	require.Empty(t, path.Edges)
	require.Zero(t, path.LoftBound)
	startRay := shell.Rays[capRays[0]]
	join, _, direction, joined := revolveloftfaces.VerticalTraceJoin(sourcePieces[0], pieces[0].To)
	require.True(t, joined)
	moved := shell.Mesh.Verts[ring.Endpoints[0]]
	moved.Y++
	motionBound, bounded := revolveloftfaces.VerticalTraceSegmentBound(proof.Trim.Maps, sourcePieces[0],
		revolveloftfaces.VerticalTraceHit{Piece: 0, Ray: capRays[0],
			Lo: pieces[0].Cell.Upper.Root.Lo, Hi: pieces[0].Cell.Upper.Root.Hi},
		revolveloftfaces.VerticalTraceHit{Piece: 0, Ray: -1,
			Lo: pieces[0].To, Hi: pieces[0].To},
		[2]*big.Rat{startRay.Y, startRay.Z}, direction, moved, join, shell.Rays)
	require.True(t, bounded)
	require.Greater(t, motionBound, 0.1, "a moved endpoint exceeds the trace fit limit")

	badProof := *proof
	badProof.Tooth = nil
	_, built, err = revolvecontact.InsertVerticalTrace(t.Context(), &badProof,
		shell, graph, arc.EndpointCellIDs[0])
	require.NoError(t, err)
	require.False(t, built, "a stale source certificate cannot move the shell")
	bad = copyGraph()
	bad.Cells[owner].SideIndex = -1
	_, built, err = revolvecontact.InsertVerticalTrace(t.Context(), proof,
		shell, bad, owner)
	require.NoError(t, err)
	require.False(t, built, "a stale graph cannot move the shell")
}
