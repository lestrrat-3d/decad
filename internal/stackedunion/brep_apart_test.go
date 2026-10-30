package stackedunion

// An internal test: on every public plate-and-boss pair a later check of the
// brep build (the interface scene's run split, its classification, or the
// planar face audit) also refuses the touching shapes these guards refuse,
// so only a reading of slabRegions and interfaceFaces alone shows each guard
// deciding.

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// apartPrism is a prism over [z0, z1] in the world XY frame.
func apartPrism(t *testing.T, profile momentinput.Profile, z0, z1 float64) Operand {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	op, ok := OperandFromPayload(prismrecord.Payload{Profile: profile, Frame: frame, Z0: z0, Z1: z1, Xform: r3.Identity()})
	require.True(t, ok)
	return op
}

func apartCircle(cx, cy, r float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: cx, V: cy}, Radius: units.Millimeters(r), CCW: true, TEnd: 1},
	}}}
}

func apartRect(u0, v0, u1, v1 float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

// apartBuild is the brep build Join starts for two operands, before any slab
// or interface is read.
func apartBuild(t *testing.T, va, vb Operand) *ubBuild {
	t.Helper()
	shift := new(big.Rat)
	levels, ok := stackedrecord.UnionLevels(va.Slabs, vb.Slabs, shift)
	require.True(t, ok)
	var reach [][2]int
	held := make([]float64, len(levels))
	for k, level := range levels {
		held[k] = level.Held
		if k+1 < len(levels) {
			reach = append(reach, [2]int{
				stackedrecord.UnionSlabOf(va.Slabs, shift, level.Exact, levels[k+1].Exact),
				stackedrecord.UnionSlabOf(vb.Slabs, shift, level.Exact, levels[k+1].Exact),
			})
		}
	}
	st := &stackedUnionState{budget: proofbound.NewWorkBudget(t.Context()), va: va, vb: vb,
		reexpress: &prismcells.Reexpression{Identity: true}}
	return &ubBuild{st: st, levels: levels, reach: reach, scenes: map[string]*ubScene{}, geom: stackedbrep.NewEngine(held)}
}

// TestBrepSlabLoopsApart merges two radius-6 bosses over one slab. Bosses
// 12.5 mm apart centre to centre keep two loops; bosses 12 mm apart touch at
// one point the arrangement does not cut, close as two whole loops all the
// same, and miss.
//
// Shown red: without slabRegions' prismcells.ProveGroupApart reading the
// tangent pair keeps two regions.
func TestBrepSlabLoopsApart(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		x    float64
		want int
	}{
		{name: "apart", x: 32.5, want: 2},
		{name: "tangent", x: 32},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			b := apartBuild(t, apartPrism(t, apartCircle(20, 30, 6), 10, 18), apartPrism(t, apartCircle(tc.x, 30, 6), 10, 18))
			require.Len(t, b.reach, 1)
			regions, err := b.slabRegions(t.Context(), 0)
			if tc.want == 0 {
				require.True(t, errors.Is(err, errUBMiss), "got %d regions, err %v", len(regions), err)
				return
			}
			require.NoError(t, err)
			require.Len(t, regions, tc.want)
			require.Zero(t, b.walk, "whole circles carry no walk charge")
		})
	}
}

// TestBrepHoledFloorRingsApart reads the interface of a 100 × 60 plate with
// one radius-6 boss standing on it. A boss 20 mm in from the plate's edge
// leaves the floor one face with the boss's circle as its hole; a boss 6 mm
// in touches the edge at one point, and the floor misses.
//
// Shown red: without interfaceFaces' prismcells.RingContact reading the
// tangent boss records the pinched floor.
func TestBrepHoledFloorRingsApart(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		x    float64
		miss bool
	}{
		{name: "apart", x: 20},
		{name: "tangent", x: 6, miss: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			b := apartBuild(t, apartPrism(t, apartRect(0, 0, 100, 60), 0, 10), apartPrism(t, apartCircle(tc.x, 30, 6), 10, 18))
			require.Len(t, b.reach, 2)
			err := b.interfaceFaces(t.Context(), 1)
			if tc.miss {
				require.True(t, errors.Is(err, errUBMiss), "err %v", err)
				return
			}
			require.NoError(t, err)
			require.Len(t, b.geom.Faces, 1)
			face := b.geom.Faces[0]
			require.True(t, face.Outward, "the plate's floor faces up")
			require.Equal(t, 1, face.Level)
			require.Len(t, face.Holes, 1)
			require.True(t, face.Holes[0].Closed, "the boss's whole circle is the hole")
		})
	}
}
