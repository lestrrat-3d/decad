package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/stretchr/testify/require"
)

func TestCurvedCallScopeSharesLiveRecords(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	ctx := revolveLoftFacesWithCallScope(t.Context())
	p1, g1, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	p2, g2, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, p1, p2)
	require.Same(t, g1, g2)

	c1, ok, err := revolveLoftFacesCensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	c2, ok, err := revolveLoftFacesCensusIn(ctx, p1)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, c1, c2)

	plain := t.Context()
	u1, ug1, ok, err := revolveLoftFacesLiveGraph(plain, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	u2, ug2, ok, err := revolveLoftFacesLiveGraph(plain, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, u1, u2)
	require.NotSame(t, ug1, ug2)

	require.Same(t, revolveLoftFacesCallScopeOf(ctx),
		revolveLoftFacesCallScopeOf(revolveLoftFacesWithCallScope(ctx)))
	require.Nil(t, revolveLoftFacesCallScopeOf(plain))
}

// unscopedAllRunsShell is revolveLoftFacesBlankShellWithAllRuns without the
// call scope, so every nested check certifies and builds its own records.
func unscopedAllRunsShell(ctx context.Context, proof *revolveLoftFacesSourceCertificate,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph, bool, error) {
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]revolveLoftFacesShellSeamSelector, 0, len(graph.runs))
	for _, run := range graph.runs {
		if run.cellID < 0 || run.cellID >= len(graph.cells) {
			return nil, nil, false, nil
		}
		cell := graph.cells[run.cellID]
		if cell.source == nil || cell.pairID < 0 ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, revolveLoftFacesShellSeamSelector{
			sectionIndex: cell.sectionIndex, cellIndex: cell.cellIndex})
	}
	capCuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof,
		graph.manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.faceBound >= 0.1 ||
		len(shell.seams) != len(graph.runs) ||
		len(shell.triangleCells) != len(shell.mesh.Tris) {
		return nil, nil, false, err
	}
	shell.seamRunIDs = make([][]int, len(graph.runs))
	for i := range graph.runs {
		view := *shell
		view.seam = shell.seams[i]
		bound, bindErr := graph.bindBlankShell(ctx, proof, &graph.runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.seamRunIDs[i] = view.seamIDs
	}
	shell.seamIDs = shell.seamRunIDs[0]
	return shell, graph, true, nil
}

func TestCurvedCallScopeShellMatchesUnscoped(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	shell, graph, ok, err := revolveLoftFacesBlankShellWithAllRuns(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ref, refGraph, ok, err := unscopedAllRunsShell(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	require.Equal(t, ref.mesh.Verts, shell.mesh.Verts)
	require.Equal(t, ref.mesh.Tris, shell.mesh.Tris)
	require.Equal(t, ref.vertexBounds, shell.vertexBounds)
	require.Equal(t, ref.seams, shell.seams)
	require.Equal(t, ref.seamRunIDs, shell.seamRunIDs)
	require.Equal(t, ref.seamIDs, shell.seamIDs)
	require.Equal(t, ref.chordBound, shell.chordBound)
	require.Equal(t, ref.faceBound, shell.faceBound)
	require.Equal(t, ref.triangleCells, shell.triangleCells)
	require.NotEmpty(t, shell.capRings)
	require.Len(t, shell.capRings, len(ref.capRings))
	for i, ring := range shell.capRings {
		want := ref.capRings[i]
		require.Equal(t, want.endpointGraphIDs, ring.endpointGraphIDs)
		require.Equal(t, want.endpointCellIDs, ring.endpointCellIDs)
		require.Equal(t, want.endpoints, ring.endpoints)
		require.Equal(t, want.vertices, ring.vertices)
	}

	require.Len(t, graph.pairs, len(refGraph.pairs))
	require.Len(t, graph.cells, len(refGraph.cells))
	require.Len(t, graph.arcs, len(refGraph.arcs))
	require.Len(t, graph.runs, len(refGraph.runs))
	for i, run := range graph.runs {
		require.Equal(t, refGraph.runs[i].cellID, run.cellID)
		require.Equal(t, refGraph.runs[i].stations, run.stations)
		require.Equal(t, refGraph.runs[i].stationIDs, run.stationIDs)
	}
	for i, arc := range graph.arcs {
		require.Equal(t, refGraph.arcs[i].endpointIDs, arc.endpointIDs)
		require.Equal(t, refGraph.arcs[i].endpointCellIDs, arc.endpointCellIDs)
	}
	for i, pair := range graph.pairs {
		require.Equal(t, refGraph.pairs[i].verdict, pair.verdict)
		require.Equal(t, refGraph.pairs[i].cellIDs, pair.cellIDs)
	}
	same, err := revolveLoftFacesContactGraphCurrent(t.Context(), refGraph)
	require.NoError(t, err)
	require.True(t, same)
}

func TestCurvedCallScopeStillRefusesChangedGraph(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	ctx := revolveLoftFacesWithCallScope(t.Context())
	_, scoped, ok, err := revolveLoftFacesLiveGraph(ctx, blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotSame(t, graph, scoped)
	current := func() bool {
		live, liveErr := revolveLoftFacesContactGraphCurrent(ctx, graph)
		require.NoError(t, liveErr)
		return live
	}
	require.True(t, current())
	t.Run("dropped cell", func(t *testing.T) {
		original := graph.cells
		graph.cells = original[: len(original)-1 : len(original)-1]
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
	t.Run("changed cap root", func(t *testing.T) {
		original := graph.arcs[0].arc.negativeZ.parameter.Lo
		graph.arcs[0].arc.negativeZ.parameter.Lo = new(big.Rat).Add(original, big.NewRat(1, 1000000))
		require.False(t, current())
		graph.arcs[0].arc.negativeZ.parameter.Lo = original
	})
	require.True(t, current())
}
