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

// Rule DW (agent-docs/carrier-record-design.md §8.7) declines the retained tag
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
