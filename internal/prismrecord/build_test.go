package prismrecord_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func squareLoop(side float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: 0, V: 0}, {U: side, V: 0}, {U: side, V: side}, {U: 0, V: side}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// TestBuildBox builds a 4x4x4 box through the node builder with bare nodes and
// reads its topology and measurements back off the nodes alone.
func TestBuildBox(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	doc := &topo.Document{}
	env := topo.Env{Doc: doc, Ref: 7, Alloc: topotest.Alloc{}}
	p := prismrecord.Payload{
		Profile: momentinput.Profile{Outer: squareLoop(4)},
		Frame:   frame,
		Z0:      0,
		Z1:      4,
		Xform:   r3.Identity(),
	}
	body, err := prismrecord.Build(t.Context(), env, p, freeform.NewFreeformWork())
	require.NoError(t, err)

	require.Same(t, doc, body.Doc)
	require.Equal(t, provenance.Ref{Producer: 7, Role: provenance.RoleBody}, body.Origin)
	require.Equal(t, topo.BodySolid, body.Kind)
	require.True(t, body.Solid)
	require.Nil(t, body.Owner, `a bare allocator sets no Owner`)
	require.Len(t, body.Lumps, 1)
	require.Len(t, body.Lumps[0].Shells, 1)
	require.False(t, body.Lumps[0].Shells[0].Open)

	faces := body.Faces()
	edges := body.Edges()
	vertices := body.Vertices()
	require.Len(t, faces, 6)
	require.Len(t, edges, 12)
	require.Len(t, vertices, 8)

	var roles []string
	for _, f := range faces {
		require.Same(t, body, f.Body)
		require.Equal(t, 16.0, f.Area)
		require.Zero(t, f.AreaBound, `an axis-aligned 4 mm face is exact`)
		for _, r := range f.Origins {
			require.Equal(t, provenance.Producer(7), r.Producer)
			roles = append(roles, r.Role)
		}
	}
	slices.Sort(roles)
	require.Equal(t, []string{"capEnd", "capStart", "side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)"}, roles)

	for _, e := range edges {
		require.Len(t, e.Faces, 2, `every edge of a closed box joins two faces`)
		require.Equal(t, 4.0, e.Length)
		require.NotNil(t, e.Start)
		require.NotNil(t, e.End)
		for _, f := range e.Faces {
			inLoop := false
			for _, l := range f.Loops {
				for _, ce := range l.CoEdges {
					inLoop = inLoop || ce.Edge == e
				}
			}
			require.True(t, inLoop, `an edge appears in the loops of each face it names`)
		}
	}

	require.Equal(t, units.CubicMillimeters(64), body.Volume.Value)
	require.Equal(t, measurement.Exact, body.Volume.Exactness)
	require.Equal(t, units.SquareMillimeters(96), body.Area.Value)
	require.Equal(t, measurement.Exact, body.Area.Exactness)
	require.Equal(t, r3.NewVec(2, 2, 2), body.Centroid.Value)
	require.Equal(t, r3.Vec{}, body.Bounds.Min)
	require.Equal(t, r3.NewVec(4, 4, 4), body.Bounds.Max)

	rec, ok := body.Payload.(prismrecord.Payload)
	require.True(t, ok)
	require.NotNil(t, rec.Walks, `the build publishes its walk resolution`)
	require.Equal(t, r3.Identity(), body.Payload.Transform())
}

// TestBuildRefusesBlendRoles keeps blend roles with root's builder.
func TestBuildRefusesBlendRoles(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	p := prismrecord.Payload{
		Profile:   momentinput.Profile{Outer: squareLoop(4)},
		Frame:     frame,
		Z1:        4,
		Xform:     r3.Identity(),
		BlendSegs: []map[int]struct{}{{0: {}}},
	}
	_, err = prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Alloc: topotest.Alloc{}}, p, freeform.NewFreeformWork())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
