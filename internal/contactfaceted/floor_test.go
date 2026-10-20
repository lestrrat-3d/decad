package contactfaceted_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func facetedBody(edit func(*facetedrecord.Payload)) *topo.Body {
	p := facetedrecord.Payload{
		Verts:     []r3.Vec{{X: 1, Y: 2, Z: 3}, {X: 4, Y: -1, Z: 5}, {X: 2, Y: 0, Z: 7}},
		MeshBound: 0.5,
	}
	if edit != nil {
		edit(&p)
	}
	return &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: p}
}

func TestBoundedExtentAtPose(t *testing.T) {
	t.Parallel()
	extent, ok, err := contactfaceted.BoundedExtentAtPose(t.Context(), facetedBody(nil), r3.Identity())
	require.NoError(t, err)
	require.True(t, ok)
	for axis, want := range [][2]float64{{1, 4}, {-1, 2}, {3, 7}} {
		lo, _ := extent.Box.Lo[axis].Float64()
		hi, _ := extent.Box.Hi[axis].Float64()
		require.Equal(t, want, [2]float64{lo, hi}, "axis %d", axis)
	}
	bound, _ := extent.Bound.Float64()
	require.Equal(t, 0.5, bound)

	shift, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	moved, ok, err := contactfaceted.BoundedExtentAtPose(t.Context(), facetedBody(nil), shift)
	require.NoError(t, err)
	require.True(t, ok)
	lo, _ := moved.Box.Lo[2].Float64()
	require.Equal(t, 13.0, lo)

	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	tests := []struct {
		name string
		body *topo.Body
		pose r3.Transform
	}{
		{"nil body", nil, r3.Identity()},
		{"not a solid", func() *topo.Body { b := facetedBody(nil); b.Solid = false; return b }(), r3.Identity()},
		{"sheet", func() *topo.Body { b := facetedBody(nil); b.Kind = topo.BodySheet; return b }(), r3.Identity()},
		{"pose off the signed axes", facetedBody(nil), turn},
		{"not a faceted payload", &topo.Body{Solid: true, Kind: topo.BodySolid}, r3.Identity()},
		{"no vertices", facetedBody(func(p *facetedrecord.Payload) { p.Verts = nil }), r3.Identity()},
		{"non-finite mesh bound", facetedBody(func(p *facetedrecord.Payload) { p.MeshBound = math.NaN() }), r3.Identity()},
		{"negative mesh bound", facetedBody(func(p *facetedrecord.Payload) { p.MeshBound = -1 }), r3.Identity()},
		{"non-finite volume difference", facetedBody(func(p *facetedrecord.Payload) { p.VolSymDiff = math.Inf(1) }), r3.Identity()},
		{"negative volume difference", facetedBody(func(p *facetedrecord.Payload) { p.VolSymDiff = -1 }), r3.Identity()},
		{"non-finite vertex", facetedBody(func(p *facetedrecord.Payload) { p.Verts[1].X = math.NaN() }), r3.Identity()},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok, err := contactfaceted.BoundedExtentAtPose(t.Context(), tc.body, tc.pose)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
	t.Run("cancelled", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := cancelledContext(t)
		defer cancel()
		_, _, err := contactfaceted.BoundedExtentAtPose(ctx, facetedBody(nil), r3.Identity())
		require.ErrorIs(t, err, context.Canceled)
	})
}

func floorBox(lo, hi [3]float64) contactbox.SourceBox {
	var b contactbox.SourceBox
	for i := range 3 {
		b.Lo[i], b.Hi[i] = proofarith.MustDyOf(lo[i]), proofarith.MustDyOf(hi[i])
	}
	return b
}

func extentOf(lo, hi [3]float64) contactfaceted.BoundedExtent {
	return contactfaceted.BoundedExtent{Box: floorBox(lo, hi), Bound: proofarith.MustDyOf(0.5)}
}

func TestInsideFloorAndFloorGap(t *testing.T) {
	t.Parallel()
	floor := floorBox([3]float64{0, 0, -5}, [3]float64{10, 10, 0})

	inside := extentOf([3]float64{2, 2, 3}, [3]float64{8, 8, 6})
	require.True(t, contactfaceted.InsideFloor(inside, floor))
	gap, ok := contactfaceted.FloorGap(inside, floor)
	require.True(t, ok)
	require.Equal(t, 3.0, gap.Value.Base())
	require.GreaterOrEqual(t, gap.Bound.Base(), 0.5)
	require.InDelta(t, 0.5, gap.Bound.Base(), 1e-12)

	// The boundary displacement reaches the floor's edge in each direction.
	for _, edge := range []struct {
		name   string
		lo, hi [3]float64
	}{
		{"low U", [3]float64{0.4, 2, 3}, [3]float64{8, 8, 6}},
		{"low V", [3]float64{2, 0.4, 3}, [3]float64{8, 8, 6}},
		{"high U", [3]float64{2, 2, 3}, [3]float64{9.6, 8, 6}},
		{"high V", [3]float64{2, 2, 3}, [3]float64{8, 9.6, 6}},
	} {
		require.False(t, contactfaceted.InsideFloor(extentOf(edge.lo, edge.hi), floor), edge.name)
	}

	// A gap no wider than the displacement is not strictly separated.
	touching := extentOf([3]float64{2, 2, 0.5}, [3]float64{8, 8, 6})
	_, ok = contactfaceted.FloorGap(touching, floor)
	require.False(t, ok)
	_, ok = contactfaceted.FloorGap(extentOf([3]float64{2, 2, 0.4}, [3]float64{8, 8, 6}), floor)
	require.False(t, ok)
}

func cancelledContext(t *testing.T) (context.Context, context.CancelFunc) {
	t.Helper()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	return ctx, cancel
}

func TestAxisSupportAtPoseRefusals(t *testing.T) {
	t.Parallel()
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	tests := []struct {
		name string
		body *topo.Body
		pose r3.Transform
	}{
		{"nil body", nil, r3.Identity()},
		{"not a solid", func() *topo.Body { b := facetedBody(nil); b.Solid = false; return b }(), r3.Identity()},
		{"sheet", func() *topo.Body { b := facetedBody(nil); b.Kind = topo.BodySheet; return b }(), r3.Identity()},
		{"pose off the signed axes", facetedBody(nil), turn},
		{"not a faceted payload", &topo.Body{Solid: true, Kind: topo.BodySolid}, r3.Identity()},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok, err := contactfaceted.AxisSupportAtPose(t.Context(), tc.body, tc.pose)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
}
