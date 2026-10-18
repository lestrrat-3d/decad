package compositemesh_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositemesh"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPlanProfileCountsLinesAndRefusal(t *testing.T) {
	line := func(x0, y0, x1, y1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: x0, V: y0},
			End:   sectionrecord.Point2{U: x1, V: y1},
		}
	}
	sp := sweeprecord.Payload{}
	sp.Prism.Profile.Outer.Segments = []sectionrecord.CurveSegment{
		line(0, 0, 2, 0), line(2, 0, 2, 1),
		line(2, 1, 0, 1), line(0, 1, 0, 0),
	}
	counts, total, target, err := compositemesh.PlanProfileCounts(t.Context(), sp, 0.01, nil)
	require.NoError(t, err)
	require.Equal(t, 4, total)
	require.Zero(t, target)
	for segment := range 4 {
		require.Equal(t, 1, counts[compositemesh.StationKey{Loop: 0, Segment: segment}])
	}

	sp.Prism.Profile.Outer.Segments = sp.Prism.Profile.Outer.Segments[:2]
	counts, total, target, err = compositemesh.PlanProfileCounts(t.Context(), sp, 0.01, nil)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Nil(t, counts)
	require.Zero(t, total)
	require.Zero(t, target)
}

func TestSpanRingsReverseArcCaps(t *testing.T) {
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, 12), CoordBound: 0.25}
	span := sweeprecord.SpanPayload{Arc: true, ReverseArcCaps: true}
	resolve := func(_ context.Context, _ revolveplan.Payload, limit uint64) (*revolveplan.Resolution, error) {
		require.Zero(t, limit)
		return &revolveplan.Resolution{}, nil
	}
	start, end, bound, err := compositemesh.SpanRings(t.Context(), span, mesh, 3, 0, resolve)
	require.NoError(t, err)
	require.Equal(t, []int{3, 7, 11}, start)
	require.Equal(t, []int{0, 4, 8}, end)
	require.Equal(t, 0.25, bound)
}

func TestSpanRingsRefusals(t *testing.T) {
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, 5)}
	start, end, bound, err := compositemesh.SpanRings(t.Context(), sweeprecord.SpanPayload{}, mesh, 3, 0, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Nil(t, start)
	require.Nil(t, end)
	require.Zero(t, bound)

	span := sweeprecord.SpanPayload{Arc: true}
	resolve := func(_ context.Context, _ revolveplan.Payload, _ uint64) (*revolveplan.Resolution, error) {
		return &revolveplan.Resolution{}, nil
	}
	start, end, bound, err = compositemesh.SpanRings(t.Context(), span, mesh, 3, 0, resolve)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Nil(t, start)
	require.Nil(t, end)
	require.Zero(t, bound)

	stop := errors.New("resolver stopped")
	resolve = func(_ context.Context, _ revolveplan.Payload, _ uint64) (*revolveplan.Resolution, error) {
		return nil, stop
	}
	start, end, bound, err = compositemesh.SpanRings(t.Context(), span, mesh, 3, 0, resolve)
	require.ErrorIs(t, err, stop)
	require.Nil(t, start)
	require.Nil(t, end)
	require.Zero(t, bound)
}

func TestSpanFaceRoleProvenance(t *testing.T) {
	face := &topo.Face{Origins: []provenance.Ref{{Role: provenance.RoleCapStart}}}
	role, err := compositemesh.SpanFaceRole(face, 2, true)
	require.NoError(t, err)
	require.Equal(t, provenance.RoleCapEnd, role)

	role, err = compositemesh.SpanFaceRole(&topo.Face{}, 2, false)
	require.Empty(t, role)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
}
