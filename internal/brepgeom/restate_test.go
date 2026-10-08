package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func xyFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// TestPlanarFrame pins docs/brep-modify-design.md §5.2's frame: for every
// reference axis and both signs, the normal is that axis scaled by the sign,
// the embed lands the local normal there, and AxisFrame and StackedWallFrame
// build the same frame for the cases they name. Shown to fail with the
// negative branch's U and V left in the positive order (r3.NewFrame then
// rebuilt a normal of the wrong sign and PlanarFrame refused).
func TestPlanarFrame(t *testing.T) {
	t.Parallel()
	ref := xyFrame(t)
	axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	for k := range 3 {
		for _, sign := range []float64{1, -1} {
			frame, e, err := brepgeom.PlanarFrame(ref, k, sign)
			require.NoError(t, err)
			require.Equal(t, axes[k].Scale(sign), frame.N(), "axis %d sign %g", k, sign)
			var want [3]float64
			want[k] = sign
			require.Equal(t, want, e.Canon(0, 0, 1))
			if sign > 0 {
				af, ae, err := brepgeom.AxisFrame(ref, k)
				require.NoError(t, err)
				require.Equal(t, [2]any{frame, e}, [2]any{af, ae})
			}
			if k < 2 {
				sf, se, err := brepgeom.StackedWallFrame(ref, brepgeom.StackedWallKey{Axis: k, Sign: sign})
				require.NoError(t, err)
				require.Equal(t, [2]any{frame, e}, [2]any{sf, se})
			}
		}
	}
}

// TestRestate pins docs/brep-modify-design.md §5.2 on S1's x = 0 wall: the
// XY-frame wall walking (0, 20) → (0, 0) over z ∈ [0, 20], material on its
// left at x > 0. Its rectangle lies in the plane x = 0 with the outward
// normal −x (u along z, v along y), through (0, 20), (0, 0), (20, 0),
// (20, 20) counter-clockwise, and the same wall recorded over the reversed
// range restates alike. An oblique, split, narrowed, displaced, curved or
// planar face is ErrRestate. Shown to fail with the outward normal read as
// the walk's left-hand normal (the rectangle then turned clockwise and every
// wall refused), and with the split, displaced and narrowed arms each deleted
// (that wall then restated). With the oblique arm deleted the oblique wall
// still refuses: its corners land on two levels of the new frame.
func TestRestate(t *testing.T) {
	t.Parallel()
	ref := xyFrame(t)
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	wall := brepgeom.FaceRecord{Frame: ref, Wall: sectionrecord.LineSeg{Start: p(0, 20), End: p(0, 0), TStart: 0, TEnd: 1},
		Z0: 0, Z1: 20, Role: "wall(1)"}

	got, e, err := brepgeom.Restate(wall, ref)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(-1, 0, 0), got.Frame.N())
	require.Equal(t, brepgeom.Embed{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, -1}}, e)
	require.Nil(t, got.Wall)
	require.Equal(t, [4]float64{0, 0, 0, 0}, [4]float64{got.Z0, got.Z1, got.Z0Delta, got.Z1Delta})
	require.Equal(t, "wall(1)", got.Role)
	pts := []sectionrecord.Point2{p(0, 20), p(0, 0), p(20, 0), p(20, 20)}
	var want sectionrecord.LoopRecord
	for i, q := range pts {
		want.Segments = append(want.Segments, sectionrecord.LineSeg{Start: q, End: pts[(i+1)%4], TStart: 0, TEnd: 1})
	}
	require.Equal(t, &brepgeom.Profile{Outer: want}, got.Region)

	reversed := wall
	reversed.Wall = sectionrecord.LineSeg{Start: p(0, 0), End: p(0, 20), TStart: 1, TEnd: 0}
	again, _, err := brepgeom.Restate(reversed, ref)
	require.NoError(t, err)
	require.Equal(t, got, again)

	for name, mutate := range map[string]func(*brepgeom.FaceRecord){
		"oblique": func(f *brepgeom.FaceRecord) {
			f.Wall = sectionrecord.LineSeg{Start: p(0, 20), End: p(5, 0), TStart: 0, TEnd: 1}
		},
		"split":     func(f *brepgeom.FaceRecord) { f.Side0 = []brepgeom.Split{{Z: 10}} },
		"displaced": func(f *brepgeom.FaceRecord) { f.Z1Delta = 1e-9 },
		"narrowed": func(f *brepgeom.FaceRecord) {
			f.Wall = sectionrecord.LineSeg{Start: p(0, 20), End: p(0, 0), TStart: 0, TEnd: 0.5}
		},
		"curved": func(f *brepgeom.FaceRecord) {
			f.Wall = sectionrecord.CircleSeg{Center: p(20, 10), Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}
		},
		"planar": func(f *brepgeom.FaceRecord) { f.Wall, f.Region = nil, got.Region },
	} {
		f := wall
		mutate(&f)
		_, _, err := brepgeom.Restate(f, ref)
		require.ErrorIs(t, err, brepgeom.ErrRestate, name)
	}
}
