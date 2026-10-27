package classbgeom

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestBuildClassBBridgeIsUnionOnly hands buildClassB a Ø6 pin bridging the
// gap of a U-shaped plate, its caps flush in the gap's walls x = 10 and
// x = 30. Union builds; Cut and Intersect, which OfPayloads never sends here,
// refuse with ErrUnsupported rather than build a solid the bridge does not
// state.
func TestBuildClassBBridgeIsUnionOnly(t *testing.T) {
	t.Parallel()
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 40, V: 0}, {U: 40, V: 20}, {U: 30, V: 20}, {U: 30, V: 5},
		{U: 10, V: 5}, {U: 10, V: 20}, {U: 0, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	plate := prismrecord.Payload{Frame: xy, Xform: r3.Identity(), Z0: 0, Z1: 10,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}}

	// The pin's frame sweeps along +x, its section in (y, z) about (12, 5).
	yz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 12, V: 5}, Radius: units.Millimeters(3),
		CCW: true, TStart: 0, TEnd: 1}
	pin := prismrecord.Payload{Frame: yz, Xform: r3.Identity(), Z0: 10, Z1: 30,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}}

	cp, ok, err := admitPair(t.Context(), plate, pin, r3.Identity(), 0, true)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, [2]bool{true, true}, cp.flush)
	slabs, side, ok, err := FlushReach(t.Context(), cp.faces, cp.tool, cp.flush)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, slabs, 2)
	require.Equal(t, FlushOutside, side)

	_, ok, err = buildClassB(t.Context(), meshbool.OpUnion, cp, slabs)
	require.NoError(t, err)
	require.True(t, ok)

	for _, op := range []meshbool.OperationKind{meshbool.OpCut, meshbool.OpIntersect} {
		out, ok, err := buildClassB(t.Context(), op, cp, slabs)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.False(t, ok)
		require.Nil(t, out)
	}
}
