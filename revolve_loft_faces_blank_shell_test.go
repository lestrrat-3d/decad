package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCurvedSourceArcSplitsClosedRevolveShell(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolvecontact.AllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	sectionIndex, cellIndex := -1, -1
	for _, side := range manifest.Nonvertical {
		for index, cell := range side.Cells {
			if cell.Mapped.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.Mapped.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact {
				sectionIndex, cellIndex = side.SectionIndex, index
				break
			}
		}
		if cellIndex >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, sectionIndex, 0)
	shell, ok, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotEmpty(t, shell.Mesh.Tris)
	require.Len(t, shell.Mesh.Src, len(shell.Mesh.Tris))
	require.Len(t, shell.VertexBounds, len(shell.Mesh.Verts))
	require.Len(t, shell.Seam, 18)
	require.Less(t, shell.ChordBound, 0.1)
	require.GreaterOrEqual(t, shell.FaceBound, shell.ChordBound)
	require.Less(t, shell.FaceBound, 0.1)
	require.True(t, shell.EulerTwo())
	for _, bound := range shell.VertexBounds {
		require.Less(t, bound, 0.1)
	}
	seenSeam := make(map[int]struct{}, len(shell.Seam))
	for _, index := range shell.Seam {
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
	var selected *revolvecontact.SourceCell
	for i := range manifest.Nonvertical {
		if manifest.Nonvertical[i].SectionIndex == sectionIndex {
			selected = &manifest.Nonvertical[i].Cells[cellIndex]
			break
		}
	}
	require.NotNil(t, selected)
	original := *selected
	changedRef := false
	for _, side := range manifest.Nonvertical {
		for _, other := range side.Cells {
			if other.Ref == original.Ref || other.Face == nil {
				continue
			}
			selected.Ref, selected.Face = other.Ref, other.Face
			_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
				[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
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
	require.Less(t, original.Mapped.TLo.Cmp(original.Mapped.THi), 0)
	selected.Mapped.TLo = lfDiv(lfAdd(original.Mapped.TLo, original.Mapped.THi),
		big.NewRat(2, 1))
	_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
	require.NoError(t, err)
	require.False(t, admitted)
	*selected = original
	changedVertical := false
	for i := range manifest.Vertical {
		entry := &manifest.Vertical[i]
		if entry.Outcome != revolveloftfaces.VerticalContact {
			continue
		}
		before := *entry
		entry.Outcome, entry.Side = revolveloftfaces.VerticalMiss, nil
		manifest.SideOutcome[entry.SectionIndex] = revolvecontact.SideMiss
		_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.SideOutcome[entry.SectionIndex] = revolvecontact.SideSourceCandidate
		*entry = before
		changedVertical = true
		break
	}
	require.True(t, changedVertical)
	changedVerticalCell := false
	for i := range manifest.Vertical {
		entry := &manifest.Vertical[i]
		if entry.Outcome != revolveloftfaces.VerticalContact ||
			entry.Side == nil || len(entry.Side.Cells) == 0 {
			continue
		}
		before := entry.Side.Cells[0]
		entry.Side.Cells[0].Span++
		_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		entry.Side.Cells[0] = before
		changedVerticalCell = true
		break
	}
	require.True(t, changedVerticalCell)
	checkedEmptyOtherSide := false
	for i := range manifest.Nonvertical {
		if manifest.Nonvertical[i].SectionIndex == sectionIndex {
			continue
		}
		cells := manifest.Nonvertical[i].Cells
		manifest.Nonvertical[i].Cells = nil
		_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.Nonvertical[i].Cells = cells
		checkedEmptyOtherSide = true
		break
	}
	require.True(t, checkedEmptyOtherSide)
	checkedReplacedOtherSide := false
	for i := range manifest.Nonvertical {
		if manifest.Nonvertical[i].SectionIndex == sectionIndex {
			continue
		}
		cells := manifest.Nonvertical[i].Cells
		manifest.Nonvertical[i].Cells = []revolvecontact.SourceCell{original}
		_, admitted, err := revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
			[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
		require.NoError(t, err)
		require.False(t, admitted)
		manifest.Nonvertical[i].Cells = cells
		checkedReplacedOtherSide = true
		break
	}
	require.True(t, checkedReplacedOtherSide)
	liveShell := trimmed.Shells()[0]
	originalFaces := liveShell.n.Faces
	t.Cleanup(func() { liveShell.n.Faces = originalFaces })
	liveShell.n.Faces = append(slices.Clone(originalFaces), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unowned retained face"}}, Body: bodyNode(trimmed)}})))
	_, admitted, err = revolvecontact.BlankShellAtRunsWithCuts(t.Context(), proof, manifest,
		[]revolvecontact.ShellSeamSelector{{SectionIndex: sectionIndex, CellIndex: cellIndex}}, nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, admitted)
	liveShell.n.Faces = originalFaces
	t.Logf("source-aligned Revolve Shell: %d vertices, %d triangles, chord bound %.6g mm",
		len(shell.Mesh.Verts), len(shell.Mesh.Tris), shell.ChordBound)
}

func TestCurvedMultiRunShellConsumesLiveGraph(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, certified, err := revolvecontact.CertifySource(t.Context(), bodyNode(blank), bodyNode(trimmed))
	require.NoError(t, err)
	require.True(t, certified)
	shell, graph, built, err := revolvecontact.BlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, built)
	require.Len(t, graph.Runs, 2)
	require.Len(t, graph.Arcs, 2)
	require.Len(t, shell.Seams, len(graph.Runs))
	require.Len(t, shell.CapRings, len(graph.Arcs))
	require.Len(t, shell.TriangleCells, len(shell.Mesh.Tris))
	require.Less(t, shell.FaceBound, 0.1)
	require.True(t, shell.EulerTwo())
	seenIDs := make(map[int]struct{})
	seenVertices := make(map[int]struct{})
	for _, arc := range graph.Arcs {
		for _, id := range arc.EndpointIDs {
			_, duplicate := seenIDs[id]
			require.False(t, duplicate)
			seenIDs[id] = struct{}{}
		}
	}
	for i, run := range graph.Runs {
		require.Equal(t, run.StationIDs, shell.SeamRunIDs[i])
		require.Len(t, shell.Seams[i], len(run.Stations))
		cell := graph.Cells[run.CellID]
		mapped := cell.Source.Mapped
		seam, blankPatch, loftPatch, patched, patchErr := revolvecontact.CertifiedArcPatches(t.Context(),
			bodyNode(blank), bodyNode(trimmed), cell.SectionIndex, mapped.Segment, mapped.Band,
			loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, run.Stations)
		require.NoError(t, patchErr)
		require.True(t, patched)
		for station, index := range shell.Seams[i] {
			require.GreaterOrEqual(t, index, 0)
			require.Less(t, index, len(shell.Mesh.Verts))
			_, duplicate := seenVertices[index]
			require.False(t, duplicate)
			seenVertices[index] = struct{}{}
			id := run.StationIDs[station]
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
	require.Len(t, seenIDs, 36+2*len(graph.Arcs))
	for _, arc := range graph.Arcs {
		matched := -1
		for i, ring := range shell.CapRings {
			if revolvecontact.ToolCapOccupiedArcEqual(arc.Arc, ring.Arc) {
				require.Equal(t, -1, matched)
				matched = i
			}
		}
		require.GreaterOrEqual(t, matched, 0)
		ring := shell.CapRings[matched]
		require.Equal(t, arc.EndpointIDs, ring.EndpointGraphIDs)
		require.Equal(t, arc.EndpointCellIDs, ring.EndpointCellIDs)
		for endpoint, cellID := range arc.EndpointCellIDs {
			index := ring.Endpoints[endpoint]
			require.Equal(t, ring.EndpointProof[endpoint].Point, shell.Mesh.Verts[index])
			require.GreaterOrEqual(t, shell.VertexBounds[index], ring.EndpointProof[endpoint].BlankBound)
			require.True(t, graph.EndpointCellMatches(arc, &ring, endpoint, cellID))
		}
	}
	sectionGroup := make([]int, len(proof.Section.Spans))
	group := 0
	for i, span := range proof.Section.Spans {
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
