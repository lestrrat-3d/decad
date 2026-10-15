package breprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// rectProfile is the 10x20 counter-clockwise section boxRecord states.
func rectProfile(t *testing.T) (r3.Frame, momentinput.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 20}, {U: 0, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	return frame, momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func requireBoxFaceView(t *testing.T, bp breprecord.Payload) {
	t.Helper()
	require.Len(t, bp.Faces, 6)
	for i, f := range bp.Faces[:4] {
		require.Nil(t, f.Region)
		require.Equal(t, 0.0, f.Z0)
		require.Equal(t, 30.0, f.Z1)
		require.Equal(t, "wall("+string(rune('0'+i))+")", f.Role)
	}
	start, end := bp.Faces[4], bp.Faces[5]
	require.NotNil(t, start.Region)
	require.False(t, start.Outward)
	require.Equal(t, 0.0, start.Z1)
	require.NotNil(t, end.Region)
	require.True(t, end.Outward)
	require.Equal(t, 30.0, end.Z0)
	require.Equal(t, 30.0, end.Z1)
}

func TestOfPrism(t *testing.T) {
	t.Parallel()
	frame, profile := rectProfile(t)
	pp := prismrecord.Payload{Profile: profile, Frame: frame, Z0: 0, Z1: 30, Xform: r3.Identity()}
	bp, roles, err := breprecord.OfPrismWithRoles(pp)
	require.NoError(t, err)
	requireBoxFaceView(t, bp)
	require.Equal(t, []string{"side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)", "capStart", "capEnd"}, roles)
	require.Equal(t, 0.0, bp.SectionDelta())

	plain, err := breprecord.OfPrism(pp)
	require.NoError(t, err)
	requireBoxFaceView(t, plain)

	pp.SurfaceResult = true
	_, err = breprecord.OfPrism(pp)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a surface-result prism has no closed face view")
}

func TestOfStacked(t *testing.T) {
	t.Parallel()
	frame, profile := rectProfile(t)
	single := stackedrecord.Payload{
		Slabs: []stackedrecord.Slab{
			{Regions: []momentinput.Profile{profile}, Z0: 0, Z1: 10},
			{Regions: []momentinput.Profile{profile}, Z0: 10, Z1: 30},
		},
		Interfaces: []stackedrecord.Interface{{}},
		Frame:      frame,
		Xform:      r3.Identity(),
	}
	bp, roles, err := breprecord.OfStackedWithRoles(t.Context(), single)
	require.NoError(t, err)
	requireBoxFaceView(t, bp)
	require.Equal(t, []string{
		"slab(0).region(0).side(0,0)", "slab(0).region(0).side(0,1)",
		"slab(0).region(0).side(0,2)", "slab(0).region(0).side(0,3)",
		"capStart", "capEnd",
	}, roles)

	group := single
	group.Slabs = []stackedrecord.Slab{{Regions: []momentinput.Profile{profile, profile}, Z0: 0, Z1: 30}}
	_, err = breprecord.OfStacked(t.Context(), group)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a prism group of 2 disjoint regions has no face view")
}

func TestRequireExactSection(t *testing.T) {
	t.Parallel()
	bp := boxRecord(t)
	require.NoError(t, breprecord.RequireExactSection(bp, "chamfers"))
	bp.Faces[2].Delta = 0.5
	err := breprecord.RequireExactSection(bp, "chamfers")
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "this evaluator chamfers a brep or stacked receiver")
	require.ErrorContains(t, err, "section displacement of 0.5 mm")
}

func TestThroughCapsCount(t *testing.T) {
	t.Parallel()
	bp := boxRecord(t)
	embeds := make([]brepgeom.Embed, len(bp.Faces))
	// Walls are swept faces; the two caps are the only planar faces.
	embeds[4].Axis = [3]int{0, 1, 2}
	embeds[5].Axis = [3]int{0, 1, 2}
	require.Equal(t, 2, breprecord.ThroughCapsCount(bp, embeds, 2))
	require.Equal(t, 0, breprecord.ThroughCapsCount(bp, embeds, 1))
	embeds[5].Axis = [3]int{0, 2, 1}
	require.Equal(t, 1, breprecord.ThroughCapsCount(bp, embeds, 2))
	require.Equal(t, 1, breprecord.ThroughCapsCount(bp, embeds, 1))
}

func TestIsP8RoundedRejectsOtherRecords(t *testing.T) {
	t.Parallel()
	require.False(t, breprecord.IsP8Rounded(breprecord.Payload{}))
	require.False(t, breprecord.IsP8Rounded(boxRecord(t)))
}
