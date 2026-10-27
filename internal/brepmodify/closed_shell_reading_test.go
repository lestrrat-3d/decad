package brepmodify_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// rectRegion is the counter-clockwise rectangle [0, w] × [0, h] in a face's
// own frame.
func rectRegion(w, h float64) *momentinput.Profile {
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: w, V: 0}, {U: w, V: h}, {U: 0, V: h}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	return &momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

// brepBox is the box [0, 10] × [0, 20] × [0, 30] stated as a brep record of
// six planar region faces, which route P reads as a prism along x, y and z
// alike.
func brepBox(t *testing.T, env topo.Env) *topo.Body {
	t.Helper()
	size := [3]float64{10, 20, 30}
	axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	bp := breprecord.Payload{Xform: r3.Identity()}
	for k := range 3 {
		// The frame's normal is axis k; its u and v are the next two axes.
		u, v := (k+1)%3, (k+2)%3
		frame, err := r3.NewFrame(r3.Vec{}, axes[u], axes[v])
		require.NoError(t, err)
		region := rectRegion(size[u], size[v])
		bp.Faces = append(bp.Faces,
			breprecord.Face{Frame: frame, Region: region, Z0: 0, Z1: 0},
			breprecord.Face{Frame: frame, Region: region, Outward: true, Z0: size[k], Z1: size[k]})
	}
	bp.AssignRoles()
	body, err := breprecord.Build(t.Context(), env, bp)
	require.NoError(t, err)
	return body
}

// TestClosedShellPrismReadingAxes is modify-general §3.2's axis rule for a
// closed shell of a brep record: inward, every reading of a box denotes the
// same eroded box, so the first one builds; outward, a reading rounds the
// edges along its own axis only, so a record that reads along several axes
// refuses (SG2).
func TestClosedShellPrismReadingAxes(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	box := brepBox(t, env)
	require.Equal(t, 6000.0, box.Volume.Value.Base())
	call := brepmodify.ShellCall{NoOpenings: true, Inward: true, T: units.Millimeters(1), Tmm: 1}

	closed, err := brepmodify.Shell(t.Context(), env, box, call)
	require.NoError(t, err)
	_, ok := closed.Payload.(stackedrecord.Payload)
	require.True(t, ok, "the closed shell is the prism's three-slab stack")
	require.InDelta(t, 6000-8*18*28, closed.Volume.Value.Base(), closed.Volume.Bound.Base()+1e-9)
	require.Len(t, closed.Lumps, 1)
	require.Len(t, closed.Lumps[0].Shells, 2)

	call.Inward = false
	_, err = brepmodify.Shell(t.Context(), env, box, call)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "reads as a prism along 3 reference axes")
	require.ErrorContains(t, err, "modify-general SG2")
}
