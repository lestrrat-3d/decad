package prismrecord_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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

func boxPayload(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: squareLoop(4)},
		Frame:   frame,
		Z0:      0,
		Z1:      4,
		Xform:   r3.Identity(),
	}
}

func bodyVertices(b *topo.Body) []*topo.Vertex {
	var out []*topo.Vertex
	for _, e := range topobuild.Edges(b) {
		for _, v := range [2]*topo.Vertex{e.Start, e.End} {
			if v != nil && !slices.Contains(out, v) {
				out = append(out, v)
			}
		}
	}
	return out
}

// TestBuildBox builds a 4x4x4 box through the node builder on bare nodes and
// reads its topology, provenance and measurements back off the nodes alone.
func TestBuildBox(t *testing.T) {
	t.Parallel()
	doc := &topo.Document{}
	env := topo.Env{Doc: doc, Ref: 7, Alloc: topotest.Alloc{}}
	body, err := prismrecord.Build(t.Context(), env, boxPayload(t), freeform.NewFreeformWork())
	require.NoError(t, err)

	require.Same(t, doc, body.Doc)
	require.Equal(t, provenance.Ref{Producer: 7, Role: provenance.RoleBody}, body.Origin)
	require.Equal(t, topo.BodySolid, body.Kind)
	require.True(t, body.Solid)
	require.Nil(t, body.Owner, `a bare allocator sets no Owner`)
	require.Len(t, body.Lumps, 1)
	require.Len(t, body.Lumps[0].Shells, 1)
	require.False(t, body.Lumps[0].Shells[0].Open)

	faces := topobuild.Faces(body)
	edges := topobuild.Edges(body)
	require.Len(t, faces, 6)
	require.Len(t, edges, 12)
	require.Len(t, bodyVertices(body), 8)

	var roles []string
	for _, f := range faces {
		require.Same(t, body, f.Body)
		require.Equal(t, 16.0, f.Area)
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

	// Two cap levels and one curve identity per rim vertex and rim edge.
	require.Equal(t, 2, int(doc.NextLevel))
	require.Equal(t, 16, int(doc.NextCurve))

	rec, ok := body.Payload.(prismrecord.Payload)
	require.True(t, ok)
	require.NotNil(t, rec.Walks, `the build publishes its walk resolution`)
	require.Equal(t, r3.Identity(), body.Payload.Transform())
}

// TestBuildSurfaceResult builds the same section as a sheet: no caps, one open
// shell, and every rim edge free.
func TestBuildSurfaceResult(t *testing.T) {
	t.Parallel()
	p := boxPayload(t)
	p.SurfaceResult = true
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Alloc: topotest.Alloc{}}, p, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Equal(t, topo.BodySheet, body.Kind)
	require.False(t, body.Solid)
	require.Len(t, body.Lumps, 1)
	require.True(t, body.Lumps[0].Shells[0].Open)
	require.Len(t, topobuild.Faces(body), 4)
	free := 0
	for _, e := range topobuild.Edges(body) {
		if len(e.Faces) == 1 {
			free++
		}
	}
	require.Equal(t, 8, free, `both rims of the four walls are free`)
	require.Equal(t, units.SquareMillimeters(64), body.Area.Value)
}

// TestBuildMintsBlendRoles re-mints a blend wall's second role under the
// build's own producer (modify §9).
func TestBuildMintsBlendRoles(t *testing.T) {
	t.Parallel()
	p := boxPayload(t)
	p.BlendSegs = []map[int]struct{}{{1: {}}}
	p.BlendKind = "fillet"
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}, p, freeform.NewFreeformWork())
	require.NoError(t, err)
	var blended [][]provenance.Ref
	for _, f := range topobuild.Faces(body) {
		if len(f.Origins) > 1 {
			blended = append(blended, f.Origins)
		}
	}
	require.Equal(t, [][]provenance.Ref{{{Producer: 3, Role: "side(0,1)"}, {Producer: 3, Role: "fillet(0,1)"}}}, blended)
}

// TestPlacedMovesTheRecord re-evaluates the box under a translation: the
// record keeps its section and its walk resolution, and the body moves.
func TestPlacedMovesTheRecord(t *testing.T) {
	t.Parallel()
	doc := &topo.Document{}
	env := topo.Env{Doc: doc, Ref: 1, Alloc: topotest.Alloc{}}
	first, err := prismrecord.Build(t.Context(), env, boxPayload(t), freeform.NewFreeformWork())
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	env.Ref = 2
	moved, err := first.Payload.Placed(t.Context(), env, shift)
	require.NoError(t, err)
	require.Equal(t, units.CubicMillimeters(64), moved.Volume.Value)
	require.Equal(t, r3.NewVec(10, 0, 0), moved.Bounds.Min)
	require.Equal(t, r3.NewVec(14, 4, 4), moved.Bounds.Max)
	require.Equal(t, shift, moved.Payload.Transform())
	require.Equal(t, provenance.Producer(2), moved.Origin.Producer)
}
