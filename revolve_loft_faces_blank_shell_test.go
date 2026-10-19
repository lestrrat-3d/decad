package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCurvedSourceArcSplitsClosedRevolveShell(t *testing.T) {
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
			if cell.mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.mapped.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact {
				sectionIndex, cellIndex = side.sectionIndex, index
				break
			}
		}
		if cellIndex >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, sectionIndex, 0)
	shell, ok, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotEmpty(t, shell.Mesh.Tris)
	require.Len(t, shell.Mesh.Src, len(shell.Mesh.Tris))
	require.Len(t, shell.VertexBounds, len(shell.Mesh.Verts))
	require.Len(t, shell.seam, 18)
	require.Less(t, shell.ChordBound, 0.1)
	require.GreaterOrEqual(t, shell.FaceBound, shell.ChordBound)
	require.Less(t, shell.FaceBound, 0.1)
	require.True(t, shell.EulerTwo())
	for _, bound := range shell.VertexBounds {
		require.Less(t, bound, 0.1)
	}
	seenSeam := make(map[int]struct{}, len(shell.seam))
	for _, index := range shell.seam {
		_, duplicate := seenSeam[index]
		require.False(t, duplicate, "each source station needs its own mesh vertex")
		seenSeam[index] = struct{}{}
		lower, upper := false, false
		for _, tri := range shell.Mesh.Tris {
			if !slices.Contains(tri[:], index) {
				continue
			}
			for _, vertex := range tri {
				lower = lower || shell.Mesh.Verts[vertex].X < shell.Mesh.Verts[index].X
				upper = upper || shell.Mesh.Verts[vertex].X > shell.Mesh.Verts[index].X
			}
		}
		require.True(t, lower && upper, "both meridian halves must use shared seam vertex %d", index)
	}
	var selected *revolveLoftFacesSourceCell
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			selected = &manifest.nonvertical[i].cells[cellIndex]
			break
		}
	}
	require.NotNil(t, selected)
	original := *selected
	changedRef := false
	for _, side := range manifest.nonvertical {
		for _, other := range side.cells {
			if other.ref == original.ref || other.face == nil {
				continue
			}
			selected.ref, selected.face = other.ref, other.face
			_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
				[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
			require.NoError(t, err)
			require.False(t, admitted)
			*selected = original
			changedRef = true
			break
		}
		if changedRef {
			break
		}
	}
	require.True(t, changedRef)
	require.Less(t, original.mapped.TLo.Cmp(original.mapped.THi), 0)
	selected.mapped.TLo = lfDiv(lfAdd(original.mapped.TLo, original.mapped.THi),
		big.NewRat(2, 1))
	_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
	require.NoError(t, err)
	require.False(t, admitted)
	*selected = original
	changedVertical := false
	for i := range manifest.vertical {
		entry := &manifest.vertical[i]
		if entry.outcome != revolveLoftFacesVerticalContact {
			continue
		}
		before := *entry
		entry.outcome, entry.side = revolveLoftFacesVerticalMiss, nil
		manifest.sideOutcome[entry.sectionIndex] = revolveLoftFacesSideMiss
		_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.sideOutcome[entry.sectionIndex] = revolveLoftFacesSideSourceCandidate
		*entry = before
		changedVertical = true
		break
	}
	require.True(t, changedVertical)
	changedVerticalCell := false
	for i := range manifest.vertical {
		entry := &manifest.vertical[i]
		if entry.outcome != revolveLoftFacesVerticalContact ||
			entry.side == nil || len(entry.side.cells) == 0 {
			continue
		}
		before := entry.side.cells[0]
		entry.side.cells[0].span++
		_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		entry.side.cells[0] = before
		changedVerticalCell = true
		break
	}
	require.True(t, changedVerticalCell)
	checkedEmptyOtherSide := false
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			continue
		}
		cells := manifest.nonvertical[i].cells
		manifest.nonvertical[i].cells = nil
		_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.nonvertical[i].cells = cells
		checkedEmptyOtherSide = true
		break
	}
	require.True(t, checkedEmptyOtherSide)
	checkedReplacedOtherSide := false
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			continue
		}
		cells := manifest.nonvertical[i].cells
		manifest.nonvertical[i].cells = []revolveLoftFacesSourceCell{original}
		_, admitted, err := revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.nonvertical[i].cells = cells
		checkedReplacedOtherSide = true
		break
	}
	require.True(t, checkedReplacedOtherSide)
	liveShell := trimmed.Shells()[0]
	originalFaces := liveShell.n.Faces
	t.Cleanup(func() { liveShell.n.Faces = originalFaces })
	liveShell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unowned retained face"}}, Body: bodyNode(trimmed)}})))
	_, admitted, err = revolveLoftFacesBlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, admitted)
	liveShell.n.Faces = originalFaces
	t.Logf("source-aligned Revolve shell: %d vertices, %d triangles, chord bound %.6g mm",
		len(shell.Mesh.Verts), len(shell.Mesh.Tris), shell.ChordBound)
}

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
	require.Len(t, shell.Seams, len(graph.runs))
	require.Len(t, shell.capRings, len(graph.arcs))
	require.Len(t, shell.TriangleCells, len(shell.Mesh.Tris))
	require.Less(t, shell.FaceBound, 0.1)
	require.True(t, shell.EulerTwo())
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
		require.Len(t, shell.Seams[i], len(run.stations))
		cell := graph.cells[run.cellID]
		mapped := cell.source.mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolveLoftFacesCertifiedArcPatches(t.Context(),
			blank, trimmed, cell.sectionIndex, mapped.Segment, mapped.Band,
			loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, run.stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		for station, index := range shell.Seams[i] {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.Mesh.Verts))
			_, duplicate := seenVertices[index]
			require.False(t, duplicate)
			seenVertices[index] = struct{}{}
			id := run.stationIDs[station]
			_, duplicate = seenIDs[id]
			require.False(t, duplicate)
			seenIDs[id] = struct{}{}
			require.Equal(t, seam[station].Point, shell.Mesh.Verts[index])
			require.Equal(t, seam[station].Point,
				blankPatch.Vertices[blankPatch.Seam[station]])
			require.Equal(t, seam[station].Point,
				loftPatch.Vertices[loftPatch.Seam[station]])
			require.GreaterOrEqual(t, shell.VertexBounds[index], seam[station].BlankBound)
			require.GreaterOrEqual(t,
				blankPatch.VertexBounds[blankPatch.Seam[station]], seam[station].BlankBound)
			require.GreaterOrEqual(t,
				loftPatch.VertexBounds[loftPatch.Seam[station]], seam[station].LoftBound)
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
			index := ring.Endpoints[endpoint]
			require.Equal(t, ring.endpointProof[endpoint].point, shell.Mesh.Verts[index])
			require.GreaterOrEqual(t, shell.VertexBounds[index], ring.endpointProof[endpoint].blankBound)
			require.True(t, graph.endpointCellMatches(arc, &ring, endpoint, cellID))
		}
	}
	sectionGroup := make([]int, len(proof.section.Spans))
	group := 0
	for i, span := range proof.section.Spans {
		sectionGroup[i] = -1
		if span.Face != nil {
			sectionGroup[i] = group
			group++
		}
	}
	seenCells := make(map[revolveloftfaces.ShellTriangleCell]struct{})
	for triIndex, cell := range shell.TriangleCells {
		require.GreaterOrEqual(t, cell.Section, 0)
		require.Less(t, cell.Section, len(shell.SectionRings))
		require.GreaterOrEqual(t, cell.Band, 0)
		require.Less(t, cell.Band+1, len(shell.SectionRings[cell.Section]))
		require.GreaterOrEqual(t, cell.Ray, 0)
		require.Less(t, cell.Ray, shell.RayCount)
		require.Equal(t, sectionGroup[cell.Section], shell.Mesh.Src[triIndex])
		_, duplicate := seenCells[cell]
		require.False(t, duplicate)
		seenCells[cell] = struct{}{}
		start := shell.SectionRings[cell.Section][cell.Band]
		end := shell.SectionRings[cell.Section][cell.Band+1]
		next := (cell.Ray + 1) % shell.RayCount
		var want [3]int
		switch {
		case len(start) == 1:
			want = [3]int{start[0], end[cell.Ray], end[next]}
		case len(end) == 1:
			want = [3]int{start[cell.Ray], end[0], start[next]}
		case cell.Half == 0:
			want = [3]int{start[cell.Ray], end[cell.Ray], end[next]}
		default:
			want = [3]int{start[cell.Ray], end[next], start[next]}
		}
		got := shell.Mesh.Tris[triIndex]
		slices.Sort(got[:])
		slices.Sort(want[:])
		require.Equal(t, want, got)
	}
}
