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
	shell, ok, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
		sectionIndex, cellIndex)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotEmpty(t, shell.mesh.Tris)
	require.Len(t, shell.mesh.Src, len(shell.mesh.Tris))
	require.Len(t, shell.vertexBounds, len(shell.mesh.Verts))
	require.Len(t, shell.seam, 18)
	require.Less(t, shell.chordBound, 0.1)
	require.GreaterOrEqual(t, shell.faceBound, shell.chordBound)
	require.Less(t, shell.faceBound, 0.1)
	require.True(t, revolveLoftFacesBlankShellEulerTwo(&shell.mesh))
	for _, bound := range shell.vertexBounds {
		require.Less(t, bound, 0.1)
	}
	seenSeam := make(map[int]struct{}, len(shell.seam))
	for _, index := range shell.seam {
		_, duplicate := seenSeam[index]
		require.False(t, duplicate, "each source station needs its own mesh vertex")
		seenSeam[index] = struct{}{}
		lower, upper := false, false
		for _, tri := range shell.mesh.Tris {
			if !slices.Contains(tri[:], index) {
				continue
			}
			for _, vertex := range tri {
				lower = lower || shell.mesh.Verts[vertex].X < shell.mesh.Verts[index].X
				upper = upper || shell.mesh.Verts[vertex].X > shell.mesh.Verts[index].X
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
			_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
				sectionIndex, cellIndex)
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
	_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
		sectionIndex, cellIndex)
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
		_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
			sectionIndex, cellIndex)
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
		_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
			sectionIndex, cellIndex)
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
		_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
			sectionIndex, cellIndex)
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
		_, admitted, err := revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
			sectionIndex, cellIndex)
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
	_, admitted, err = revolveLoftFacesBlankShellAtArc(t.Context(), proof, manifest,
		sectionIndex, cellIndex)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, admitted)
	liveShell.n.Faces = originalFaces
	t.Logf("source-aligned Revolve shell: %d vertices, %d triangles, chord bound %.6g mm",
		len(shell.mesh.Verts), len(shell.mesh.Tris), shell.chordBound)
}
