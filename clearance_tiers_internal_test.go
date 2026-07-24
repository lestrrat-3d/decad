package decad

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests the private clearance-tier work boundary. Public Verify
// coverage cannot distinguish cancellation inside a tier from an outer poll.

func tierPlaneFace(region region2) *cFace {
	return &cFace{
		kind:   ckPlane,
		o:      r3.Vec{},
		u:      r3.NewVec(1, 0, 0),
		v:      r3.NewVec(0, 1, 0),
		n:      r3.NewVec(0, 0, 1),
		region: region,
	}
}

func tierSphereFace(x float64) *cFace {
	return &cFace{
		kind:   ckSphere,
		anchor: r3.NewVec(x, 0, 0),
		axis:   r3.NewVec(0, 0, 1),
		refU:   r3.NewVec(1, 0, 0),
		refV:   r3.NewVec(0, 1, 0),
		radius: 1,
		merid:  angWindow{full: true},
		sweep:  angWindow{full: true},
	}
}

func TestClearanceEnumerationCountsVertexTierFaces(t *testing.T) {
	faces := make([]*cFace, workPollInterval+64)
	for i := range faces {
		faces[i] = tierSphereFace(100 + float64(i))
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "vertexTier"}
	k := &pairKernel{
		a:   &bodyGeom{verts: []r3.Vec{{}}},
		b:   &bodyGeom{faces: faces},
		ctx: ctx,
		tol: 1e-9,
	}

	_, err := k.enumerate()

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestVertexTierCountsEdges(t *testing.T) {
	edges := make([]*cEdge, workPollInterval+64)
	for i := range edges {
		y := 100 + float64(i)
		edges[i] = &cEdge{
			line: true,
			a:    r3.NewVec(100, y, 0),
			b:    r3.NewVec(101, y, 0),
		}
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "vertexTier"}
	k := &pairKernel{ctx: ctx, tol: 1e-9}

	err := k.vertexTier(newWorkBudget(ctx), r3.Vec{}, &bodyGeom{edges: edges}, &cellSink{})

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestVertexFaceCountsBoundaryDistance(t *testing.T) {
	region := internalPolygonRegion(t, 0, 0, 100, workPollInterval+64)
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "regionBoundaryDistBudget"}
	k := &pairKernel{ctx: ctx, tol: region.tol()}

	err := k.vertexFace(newWorkBudget(ctx), r3.NewVec(0, 0, 5), tierPlaneFace(region), &cellSink{})

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestVertexFaceCountsWindingScan(t *testing.T) {
	region := internalPolygonRegion(t, 0, 0, 100, workPollInterval-16)
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "regionContainsBudget"}
	k := &pairKernel{ctx: ctx, tol: region.tol()}

	err := k.vertexFace(newWorkBudget(ctx), r3.NewVec(0, 0, 5), tierPlaneFace(region), &cellSink{})

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestVertexTierPropagatesBudgetExhaustion(t *testing.T) {
	exhausted := errors.New("work budget exhausted")
	steps := 0
	budget := &workBudget{
		stepFn: func() error {
			steps++
			if steps == 2 {
				return exhausted
			}
			return nil
		},
		errFn: func() error { return nil },
	}
	k := &pairKernel{ctx: t.Context(), tol: 1e-9}
	other := &bodyGeom{faces: []*cFace{tierSphereFace(100), tierSphereFace(200)}}

	err := k.vertexTier(budget, r3.Vec{}, other, &cellSink{})

	require.ErrorIs(t, err, exhausted)
	require.Equal(t, 2, steps)
}

func TestVertexTierBudgetKeepsNormalResult(t *testing.T) {
	region := internalPolygonRegion(t, 0, 1, 10, 4)
	k := &pairKernel{ctx: t.Context(), tol: region.tol()}
	sink := &cellSink{}

	err := k.vertexTier(
		newWorkBudget(t.Context()),
		r3.NewVec(0, 0, 5),
		&bodyGeom{faces: []*cFace{tierPlaneFace(region)}},
		sink,
	)

	require.NoError(t, err)
	require.Len(t, sink.contribs, 1)
	require.Equal(t, 5.0, sink.contribs[0].lo)
	require.Equal(t, 5.0, sink.contribs[0].hi)
	require.True(t, sink.contribs[0].exact)
	require.False(t, math.IsInf(sink.contribs[0].lo, 0))
}
