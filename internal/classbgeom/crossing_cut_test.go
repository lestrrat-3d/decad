package classbgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// shaftCut is the class-B Cut of the r = 10, L = 40 shaft along x by the box
// u in [-5, 45], v in [v0, v1] on the plane z = 7, 20 mm tall: the D-flat
// (v in [-20, 20]) or the keyway (v in [-3, 3]) of
// apitest/revolve_shaft_class_b_test.go.
func shaftCut(t *testing.T, v0, v1 float64) breprecord.Payload {
	t.Helper()
	yz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1}
	shaft := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Frame:   yz, Z1: 40, Xform: r3.Identity(),
	}
	xy, err := r3.NewFrame(r3.Vec{Z: 7}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: -5, V: v0}, {U: 45, V: v0}, {U: 45, V: v1}, {U: -5, V: v1}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	tool := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   xy, Z1: 20, Xform: r3.Identity(),
	}
	out, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpCut, shaft, tool, r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok, `the pair takes class B`)
	bp, ok := out.(breprecord.Payload)
	require.True(t, ok, `the crossing reach builds a brep record`)
	require.NotZero(t, bp.SectionDelta(), `the crossings displace the section`)
	return bp
}

// TestCrossingCutRecordsWallCarrier is general-boolean §4.1's wallCarrier: a
// crossing cut's cylinder piece records the operand circle it lies on, the
// shaft's (0, 0) and 10 in the piece's frame, and no planar face records one.
// Shown-to-fail: without the assignment in the crossing build the wall's
// carrier is zero, and apitest's class-B blends refuse brep-modify PD2.
func TestCrossingCutRecordsWallCarrier(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		v0, v1 float64
	}{{"D-flat", -20, 20}, {"keyway", -3, 3}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			walls := 0
			for _, f := range shaftCut(t, tc.v0, tc.v1).Faces {
				if f.Planar() {
					require.Zero(t, f.WallCarrier, `a planar face records no wall carrier`)
					continue
				}
				walls++
				require.Equal(t, brepgeom.CircleCarrier{Radius: 10}, f.WallCarrier)
			}
			require.Equal(t, 1, walls)
		})
	}
}
