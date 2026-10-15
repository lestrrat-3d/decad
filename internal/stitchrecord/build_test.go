package stitchrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cubeFaces builds a closed cube of six free-standing squares of side 3,
// each with its own vertices and edges (every edge Convex false), walked
// counter-clockwise about its outward normal, all reading one operand body
// whose box is the cube. inward reverses every face; offset moves the cube.
func cubeFaces(t *testing.T, inward bool, offset r3.Vec) []*topo.Face {
	t.Helper()
	const size = 3.0
	alloc := topotest.Alloc{}
	owner := &topo.Body{Bounds: measurement.Box{Min: offset, Max: offset.Add(r3.NewVec(size, size, size))}}
	quads := [6][4]r3.Vec{
		{{}, {Y: size}, {X: size, Y: size}, {X: size}},
		{{Z: size}, {X: size, Z: size}, {X: size, Y: size, Z: size}, {Y: size, Z: size}},
		{{}, {X: size}, {X: size, Z: size}, {Z: size}},
		{{Y: size}, {Y: size, Z: size}, {X: size, Y: size, Z: size}, {X: size, Y: size}},
		{{}, {Z: size}, {Y: size, Z: size}, {Y: size}},
		{{X: size}, {X: size, Y: size}, {X: size, Y: size, Z: size}, {X: size, Z: size}},
	}
	var faces []*topo.Face
	for _, c := range quads {
		ex, ok := c[1].Sub(c[0]).Normalize()
		require.True(t, ok)
		ey, ok := c[3].Sub(c[0]).Normalize()
		require.True(t, ok)
		frame, err := r3.NewFrame(c[0].Add(offset), ex, ey)
		require.NoError(t, err)
		var verts [4]*topo.Vertex
		for i := range 4 {
			verts[i] = topobuild.NewVertex(alloc, topo.Vertex{Position: c[i].Add(offset)})
		}
		coedges := make([]topo.CoEdge, 4)
		for i := range 4 {
			e := topobuild.NewEdge(alloc, topo.Edge{Curve: surfacegeom.Line3{}, Length: size, Start: verts[i], End: verts[(i+1)%4]})
			coedges[i] = topo.CoEdge{Edge: e, Forward: true}
		}
		f := topobuild.NewFace(alloc, topo.Face{
			Area: size * size, Surface: surfacegeom.Plane{Frame: frame}, Body: owner,
			Loops: []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: coedges})},
		})
		for _, ce := range coedges {
			ce.Edge.Faces = []*topo.Face{f}
		}
		if inward {
			stitchrecord.ReverseFaceOrientation(f)
		}
		faces = append(faces, f)
	}
	return faces
}

func env() topo.Env { return topo.Env{Ref: 7, Alloc: topotest.Alloc{}} }

// TestBuildClosesACube welds the twelve shared edges of six free squares
// into a solid whose volume, area, box and welded-edge convexity are the
// cube's, whichever way the operands were wound.
func TestBuildClosesACube(t *testing.T) {
	t.Parallel()
	for _, inward := range []bool{false, true} {
		faces := cubeFaces(t, inward, r3.Vec{})
		plan := stitchrecord.BuildPlan(faces)
		require.Equal(t, 12, plan.Groups)
		require.Len(t, plan.Table.Vertices, 8)

		body, err := stitchrecord.Build(t.Context(), env(), faces, plan, r3.Identity())
		require.NoError(t, err)
		require.Equal(t, topo.BodySolid, body.Kind)
		require.True(t, body.Solid)
		require.Equal(t, 27.0, body.Volume.Value.Base(), "inward=%v", inward)
		require.Equal(t, measurement.Exact, body.Volume.Exactness)
		require.Equal(t, 54.0, body.Area.Value.Base())
		require.Equal(t, r3.NewVec(1.5, 1.5, 1.5), body.Centroid.Value)
		require.Equal(t, measurement.Box{Max: r3.NewVec(3, 3, 3), Bound: units.Millimeters(0)}, body.Bounds)

		edges := topobuild.Edges(body)
		require.Len(t, edges, 12)
		for _, e := range edges {
			require.Len(t, e.Faces, 2)
			require.True(t, e.Convex, "every welded cube edge is convex")
		}
		p, ok := body.Payload.(stitchrecord.Payload)
		require.True(t, ok)
		require.True(t, p.AuditClean)
		require.True(t, p.ProvesSimple(t.Context()))
		require.Len(t, p.Tris, 12)
		require.Len(t, p.TriFaces, 12)
		require.Len(t, p.LiveFaces, 6)
	}
}

// TestBuildPlacedCubeMovesAndCharges replays the record under a translation:
// the box moves by the offset and both it and the volume carry the
// placement's nonzero rounding.
func TestBuildPlacedCubeMovesAndCharges(t *testing.T) {
	t.Parallel()
	faces := cubeFaces(t, false, r3.Vec{})
	move, err := r3.Translation(r3.NewVec(10, 20, 30))
	require.NoError(t, err)
	body, err := stitchrecord.Build(t.Context(), env(), faces, stitchrecord.BuildPlan(faces), move)
	require.NoError(t, err)
	require.InDelta(t, 27.0, body.Volume.Value.Base(), 1e-9)
	require.Equal(t, measurement.Approximate, body.Volume.Exactness)
	require.Positive(t, body.Bounds.Bound.Base())
	require.Equal(t, r3.NewVec(10, 20, 30), body.Bounds.Min)
	require.Equal(t, r3.NewVec(13, 23, 33), body.Bounds.Max)
	p, ok := body.Payload.(stitchrecord.Payload)
	require.True(t, ok)
	require.Positive(t, p.Delta)
	for _, b := range p.VertBound {
		require.GreaterOrEqual(t, b, p.Delta, "every zero-bound class widens by the placement")
	}
}

// TestBuildOpenSheet leaves one square out: five welded faces publish a
// sheet with no volume and a recorded triangle set.
func TestBuildOpenSheet(t *testing.T) {
	t.Parallel()
	faces := cubeFaces(t, false, r3.Vec{})[:5]
	body, err := stitchrecord.Build(t.Context(), env(), faces, stitchrecord.BuildPlan(faces), r3.Identity())
	require.NoError(t, err)
	require.Equal(t, topo.BodySheet, body.Kind)
	require.False(t, body.Solid)
	require.Equal(t, 45.0, body.Area.Value.Base())
	p, ok := body.Payload.(stitchrecord.Payload)
	require.True(t, ok)
	require.Len(t, p.Tris, 10)
	require.True(t, p.AuditClean)
}

// TestBuildSeparatesTwoCubes stitches two cubes at once: apart, they publish
// one two-lump solid of twice the volume; overlapping, the two closed lumps
// are not proven separate and refuse as R20.
func TestBuildSeparatesTwoCubes(t *testing.T) {
	t.Parallel()
	apart := append(cubeFaces(t, false, r3.Vec{}), cubeFaces(t, false, r3.NewVec(5, 0, 0))...)
	body, err := stitchrecord.Build(t.Context(), env(), apart, stitchrecord.BuildPlan(apart), r3.Identity())
	require.NoError(t, err)
	require.Len(t, body.Lumps, 2)
	require.Equal(t, 54.0, body.Volume.Value.Base())

	overlapping := append(cubeFaces(t, false, r3.Vec{}), cubeFaces(t, false, r3.NewVec(1, 1, 1))...)
	_, err = stitchrecord.Build(t.Context(), env(), overlapping, stitchrecord.BuildPlan(overlapping), r3.Identity())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "Table R row R20")
}

// TestBuildUnstitchFaceBoundsTheSquare copies one square: its box is the
// square's own, exact at the identity and moved, with the placement's bound,
// under a translation.
func TestBuildUnstitchFaceBoundsTheSquare(t *testing.T) {
	t.Parallel()
	src := cubeFaces(t, false, r3.Vec{})[2]
	whole := measurement.Box{Max: r3.NewVec(3, 3, 3)}
	body, err := stitchrecord.BuildUnstitchFace(t.Context(), env(), src, whole, r3.Identity())
	require.NoError(t, err)
	require.Equal(t, topo.BodySheet, body.Kind)
	require.Equal(t, measurement.Box{Max: r3.NewVec(3, 0, 3), Bound: units.Millimeters(0)}, body.Bounds)
	require.Equal(t, 9.0, body.Area.Value.Base())
	f := topobuild.Faces(body)
	require.Len(t, f, 1)
	require.NotSame(t, src, f[0])
	require.Len(t, topobuild.Edges(body), 4)

	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	placed, err := stitchrecord.BuildUnstitchFace(t.Context(), env(), src, whole, move)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(1, 2, 3), placed.Bounds.Min)
	require.Equal(t, r3.NewVec(4, 2, 6), placed.Bounds.Max)
	p, ok := placed.Payload.(stitchrecord.UnstitchPayload)
	require.True(t, ok)
	require.Positive(t, p.Delta)
	require.GreaterOrEqual(t, placed.Bounds.Bound.Base(), p.Delta, "the box carries the copied vertices' placement bound")
}
