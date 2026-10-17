package tier

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestEnumerateVertexVisitsFaceBeforeEdge(t *testing.T) {
	t.Parallel()
	sink := &clearance.CellSink{}
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 2 {
				require.Len(t, sink.Contribs, 2, "the face must contribute before the edge step")
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	face := &clearance.CFace{
		Kind:   clearance.CkSphere,
		Anchor: r3.NewVec(10, 0, 0),
		Axis:   r3.NewVec(0, 0, 1),
		RefU:   r3.NewVec(1, 0, 0),
		RefV:   r3.NewVec(0, 1, 0),
		Radius: 1,
		Merid:  clearance.AngWindow{Full: true},
		Sweep:  clearance.AngWindow{Full: true},
	}
	edge := &clearance.CEdge{
		Line: true,
		A:    r3.NewVec(-1, 0, 20),
		B:    r3.NewVec(1, 0, 20),
	}
	cells := New(t.Context(), 1e-9, 0)

	err := cells.EnumerateVertex(budget, r3.Vec{}, []*clearance.CFace{face}, []*clearance.CEdge{edge}, sink)

	require.NoError(t, err)
	require.Equal(t, 2, steps)
	require.Len(t, sink.Contribs, 3)
	require.Less(t, sink.Contribs[0].Hi, sink.Contribs[2].Hi)
	require.Less(t, sink.Contribs[1].Hi, sink.Contribs[2].Hi)
}

func TestEnumerateVertexStepsBeforePruning(t *testing.T) {
	t.Parallel()
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return nil },
		ErrFn:  func() error { return nil },
	}
	far := clearance.BoxOf(r3.NewVec(100, 0, 0), r3.NewVec(101, 0, 0))
	face := &clearance.CFace{Box: far}
	edge := &clearance.CEdge{Box: far}
	sink := &clearance.CellSink{Prune: true, Best: 1}
	cells := New(t.Context(), 1e-9, 0)

	err := cells.EnumerateVertex(budget, r3.Vec{}, []*clearance.CFace{face}, []*clearance.CEdge{edge}, sink)

	require.NoError(t, err)
	require.Equal(t, 2, steps)
	require.Equal(t, 2, sink.Skipped)
	require.Empty(t, sink.Contribs)
}

func TestEnumerateVertexCapturesCellStateInOrder(t *testing.T) {
	t.Parallel()
	first := errors.New("first cell")
	last := errors.New("last cell")
	cells := New(t.Context(), 1e-9, 0)

	cells.captureCell(&Kernel{err: first, refused: true})
	cells.captureCell(&Kernel{})
	require.ErrorIs(t, cells.Err(), first)
	require.True(t, cells.Refused())

	cells.captureCell(&Kernel{err: last})
	require.ErrorIs(t, cells.Err(), last)
	require.True(t, cells.Refused())
}
