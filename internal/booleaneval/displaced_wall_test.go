package booleaneval_test

import (
	"context"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Rule DW (agent-docs/prism-boolean-design.md Part K §8.7) declines the retained tag
// of a displaced wall, Plane and Cylinder alike, and keeps every other tag.
func TestDisplacedWall(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	wall, other := &topo.Face{}, &topo.Face{}
	walls := map[*topo.Face]struct{}{wall: {}}
	for _, g := range []facetedrecord.Group{
		{Surface: surfacegeom.Plane{Frame: frame}, Retained: true, AxialDelta: 1, HasAxialDelta: true, NormalBound: 0.5},
		{Surface: surfacegeom.Cylinder{}, Retained: true},
	} {
		got := booleaneval.DisplacedWall(g, wall, walls)
		require.Nil(t, got.Surface)
		require.False(t, got.Retained)
		require.False(t, got.HasAxialDelta)
		require.Zero(t, got.NormalBound)
		require.Equal(t, g, booleaneval.DisplacedWall(g, other, walls), "a face outside the set keeps its tag")
		require.Equal(t, g, booleaneval.DisplacedWall(g, wall, nil))
	}
}

type cutMesher struct{}

func (cutMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	mesh, err := tessdispatch.Body(ctx, body, chord, tessdispatch.All, tessdispatch.Options{Alloc: topotest.Alloc{}})
	if err != nil {
		return nil, false, err
	}
	if err := topobuild.RequireShellWinding(ctx, mesh, body.Lumps); err != nil {
		return nil, false, err
	}
	mesh.BoundaryOK = true
	return mesh, false, nil
}

func dispSquare(s float64, hole bool, u, v float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: u, V: v}, {U: u + s, V: v}, {U: u + s, V: v + s}, {U: u, V: v + s}}
	if hole {
		pts[1], pts[3] = pts[3], pts[1]
	}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// A mesh Cut of a DisplacedPayload by a box (rule DW end to end): the facets of
// the displaced outer loop's walls carry no retained tag, while the exact
// hole's walls and the selected hole's band patches keep theirs.
func TestDisplacedWallsInAMeshCut(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	set := capblend.Setback{Dc: 0.5, Ds: 0.5}
	target, err := capblend.BuildDisplaced(t.Context(), env, capblend.Payload{
		Profile: momentinput.Profile{Outer: dispSquare(20, false, 0, 0), Holes: []sectionrecord.LoopRecord{dispSquare(4, true, 8, 8)}},
		Frame:   frame, Z1: 10, Xform: r3.Identity(), Start: set, End: set,
		StartLoops: map[int]bool{1: true}, EndLoops: map[int]bool{1: true},
	}, []float64{0.01, 0})
	require.NoError(t, err)
	env.Ref = 4
	tool, err := prismrecord.Build(t.Context(), env, prismrecord.Payload{
		Profile: momentinput.Profile{Outer: dispSquare(4, false, 17, 3)}, Frame: frame, Z0: -1, Z1: 11, Xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	require.NoError(t, err)

	ev, err := booleaneval.Evaluate(t.Context(), meshbool.OpCut, target, tool, cutMesher{}, 1, nil, nil)
	require.NoError(t, err)
	seen := map[string]int{}
	faces := topobuild.FacesByRole(target)
	for _, g := range ev.Payload.Groups {
		for _, o := range g.Origins {
			if o.Producer != target.Origin.Producer {
				continue
			}
			switch {
			case strings.HasPrefix(o.Role, "side(0,"):
				base, _ := facetedrecord.SourceTag(faces[o.Role], false)
				require.True(t, base.Retained, "without rule DW the wall's tag is retained: "+o.Role)
				require.False(t, g.Retained, o.Role)
				require.Nil(t, g.Surface, o.Role)
				seen["displaced wall"]++
			case strings.HasPrefix(o.Role, "side(1,"):
				base, _ := facetedrecord.SourceTag(faces[o.Role], false)
				require.Equal(t, base.Retained, g.Retained, o.Role)
				require.True(t, g.Retained, o.Role)
				seen["exact wall"]++
			case strings.HasPrefix(o.Role, "chamferCap("):
				base, _ := facetedrecord.SourceTag(faces[o.Role], false)
				require.Equal(t, base.Retained, g.Retained, o.Role)
				if g.Retained {
					seen["band patch"]++
				}
			}
		}
	}
	require.Equal(t, 4, seen["displaced wall"])
	require.Equal(t, 4, seen["exact wall"])
	require.Positive(t, seen["band patch"])
}

// tagOf is what a group says of its face's tag: the plain-data reading the
// two callers of rule DW are compared by.
type tagOf struct {
	Retained bool
	Surface  bool
}

// tagsByRole reads each group's tag by the role of the target face it came
// from, for the faces whose role starts with one of prefixes.
func tagsByRole(t *testing.T, target *topo.Body, groups []facetedrecord.Group, prefixes ...string) map[string]tagOf {
	t.Helper()
	out := map[string]tagOf{}
	for _, g := range groups {
		for _, o := range g.Origins {
			if o.Producer != target.Origin.Producer {
				continue
			}
			for _, p := range prefixes {
				if strings.HasPrefix(o.Role, p) {
					out[o.Role] = tagOf{Retained: g.Retained, Surface: g.Surface != nil}
				}
			}
		}
	}
	return out
}

func displacedCutOperands(t *testing.T) (target, tool *topo.Body) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	set := capblend.Setback{Dc: 0.5, Ds: 0.5}
	target, err = capblend.BuildDisplaced(t.Context(), env, capblend.Payload{
		Profile: momentinput.Profile{Outer: dispSquare(20, false, 0, 0), Holes: []sectionrecord.LoopRecord{dispSquare(4, true, 8, 8)}},
		Frame:   frame, Z1: 10, Xform: r3.Identity(), Start: set, End: set,
		StartLoops: map[int]bool{1: true}, EndLoops: map[int]bool{1: true},
	}, []float64{0.01, 0})
	require.NoError(t, err)
	env.Ref = 4
	tool, err = prismrecord.Build(t.Context(), env, prismrecord.Payload{
		Profile: momentinput.Profile{Outer: dispSquare(4, false, 17, 3)}, Frame: frame, Z0: -1, Z1: 11, Xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	return target, tool
}

// TestDisplacedWallViewMatchesEvaluator: rule DW runs at both of
// sourceGroup's callers, so booleaneval.View, the faceted view a retained
// edge blend reads, declines the same faces the evaluator's mesh path does and
// keeps the same tags on every other face of the target.
func TestDisplacedWallViewMatchesEvaluator(t *testing.T) {
	t.Parallel()
	target, tool := displacedCutOperands(t)
	ev, err := booleaneval.Evaluate(t.Context(), meshbool.OpCut, target, tool, cutMesher{}, 1, nil, nil)
	require.NoError(t, err)
	view, _, err := booleaneval.View(t.Context(), target, cutMesher{})
	require.NoError(t, err)

	prefixes := []string{"side(", "chamferCap(", "cap", "start", "end"}
	fromView := tagsByRole(t, target, view.Groups, prefixes...)
	fromEval := tagsByRole(t, target, ev.Payload.Groups, prefixes...)
	require.NotEmpty(t, fromEval)
	declined, kept := 0, 0
	for role, got := range fromEval {
		require.Equal(t, got, fromView[role], "the view and the evaluator read %s alike", role)
		if strings.HasPrefix(role, "side(0,") {
			require.Equal(t, tagOf{}, got, "the displaced outer wall %s keeps no tag", role)
			declined++
		}
		if strings.HasPrefix(role, "side(1,") {
			require.True(t, got.Retained, role)
			kept++
		}
	}
	require.Equal(t, 4, declined)
	require.Equal(t, 4, kept)
}

// TestDisplacedPrismKeepsItsTagsInAMeshCut pins that rule DW covers a
// DisplacedPayload only: a displaced prismrecord.Payload reaches the mesh
// path as on main, and its walls keep their retained tags (§8.7, "Displaced
// prisms keep today's tags"). The same Cut of a DisplacedPayload declines
// the displaced outer wall's tag, so the two answers differ on the rule alone.
func TestDisplacedPrismKeepsItsTagsInAMeshCut(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	profile := momentinput.Profile{Outer: dispSquare(20, false, 0, 0), Holes: []sectionrecord.LoopRecord{dispSquare(4, true, 8, 8)}}
	cr := prismrecord.UniformCarrierRecord(profile, 1e-13)
	cr.SectionDelta = 1e-12
	pp := prismrecord.Payload{Profile: profile, Frame: frame, Z1: 10, Xform: r3.Identity(), SectionDelta: 1e-12, Carriers: &cr}
	target, err := prismrecord.BuildWithRecord(t.Context(), env, pp, cr)
	require.NoError(t, err)
	require.NotZero(t, target.Payload.(prismrecord.Payload).SectionDelta, "the receiver is a displaced prism")
	_, tool := displacedCutOperands(t)

	ev, err := booleaneval.Evaluate(t.Context(), meshbool.OpCut, target, tool, cutMesher{}, 1, nil, nil)
	require.NoError(t, err)
	tags := tagsByRole(t, target, ev.Payload.Groups, "side(")
	require.NotEmpty(t, tags)
	faces := topobuild.FacesByRole(target)
	for role, got := range tags {
		base, _ := facetedrecord.SourceTag(faces[role], false)
		require.True(t, base.Retained, role)
		require.True(t, got.Retained, "the displaced prism's wall %s keeps its tag", role)
		require.True(t, got.Surface, role)
	}
	require.Contains(t, tags, "side(0,0)")
	require.Contains(t, tags, "side(1,0)")
}

func obliqueGroup(t *testing.T) facetedrecord.Group {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return facetedrecord.Group{Surface: surfacegeom.Plane{Frame: frame}, Retained: true,
		NormalBound: 3e-14, ExtentDelta: 5e-15}
}

// A displaced wall's mesh-Boolean tag is declined whole: the extent drops with the tag and its tilt.
func TestDisplacedWallDropsExtentDelta(t *testing.T) {
	t.Parallel()
	f := &topo.Face{}
	g := obliqueGroup(t)
	kept := booleaneval.DisplacedWall(g, f, map[*topo.Face]struct{}{})
	require.Equal(t, g, kept, "a face that is not a displaced wall keeps its group")
	dropped := booleaneval.DisplacedWall(g, f, map[*topo.Face]struct{}{f: {}})
	require.Nil(t, dropped.Surface)
	require.Zero(t, dropped.ExtentDelta)
	require.Zero(t, dropped.NormalBound)
}
