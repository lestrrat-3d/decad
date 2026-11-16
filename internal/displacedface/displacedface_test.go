package displacedface_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/displacedface"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func loop(pts ...sectionrecord.Point2) sectionrecord.LoopRecord {
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func square(s float64) sectionrecord.LoopRecord {
	return loop(sectionrecord.Point2{U: 0, V: 0}, sectionrecord.Point2{U: s, V: 0},
		sectionrecord.Point2{U: s, V: s}, sectionrecord.Point2{U: 0, V: s})
}

func hole(u, v, s float64) sectionrecord.LoopRecord {
	return loop(sectionrecord.Point2{U: u, V: v}, sectionrecord.Point2{U: u, V: v + s},
		sectionrecord.Point2{U: u + s, V: v + s}, sectionrecord.Point2{U: u + s, V: v})
}

// roles are the roles of the faces of b that the set holds, by loop.
func wallLoops(t *testing.T, b *topo.Body, walls map[*topo.Face]struct{}) map[int]int {
	t.Helper()
	out := map[int]int{}
	for _, f := range topobuild.Faces(b) {
		if _, ok := walls[f]; !ok {
			continue
		}
		li, side, _ := displacedface.FaceLoop(f)
		if !side {
			li = -1
		}
		out[li]++
	}
	return out
}

func TestWallsOfADisplacedCapBlend(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	set := capblend.Setback{Dc: 0.5, Ds: 0.5}
	cbp := capblend.Payload{
		Profile: momentinput.Profile{Outer: square(20), Holes: []sectionrecord.LoopRecord{hole(8, 8, 4)}},
		Frame:   frame, Z1: 10, Xform: r3.Identity(), Start: set, End: set,
		StartLoops: map[int]bool{1: true}, EndLoops: map[int]bool{1: true},
	}
	body, err := capblend.BuildDisplaced(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}, cbp, []float64{0.01, 0})
	require.NoError(t, err)
	walls, err := displacedface.Walls(proofbound.NewWorkBudget(t.Context()), body)
	require.NoError(t, err)
	require.Equal(t, map[int]int{0: 4}, wallLoops(t, body, walls), "the four outer walls only: no cap, no band patch, no hole wall")
}

func TestWallsOfADisplacedPrism(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: square(20), Holes: []sectionrecord.LoopRecord{hole(8, 8, 4)}}
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	cr := prismrecord.ZeroCarrierRecord(profile)
	cr.SectionDelta = 1e-6
	for j := range cr.Junction[0] {
		cr.Junction[0][j] = 1e-9
	}
	pp := prismrecord.Payload{Profile: profile, Frame: frame, Z1: 5, Xform: r3.Identity(), SectionDelta: 1e-6, Carriers: &cr}
	body, err := prismrecord.BuildWithRecord(t.Context(), env, pp, cr)
	require.NoError(t, err)
	untag, kept, err := displacedface.PrismSideFaces(proofbound.NewWorkBudget(t.Context()), body, pp, &cr)
	require.NoError(t, err)
	require.True(t, kept, "the exact hole loop keeps its walls")
	require.Len(t, untag, 4, "the displaced outer loop's four walls")
	walls, err := displacedface.Walls(proofbound.NewWorkBudget(t.Context()), body)
	require.NoError(t, err)
	require.Empty(t, walls, "a displaced prism's walls are not read yet")
}
