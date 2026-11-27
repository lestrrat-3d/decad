package booleaneval

// CI: isolated-tests

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDirectLoftSharedEdge(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	check := func() (bool, error) {
		return directLoftSharedEdge(t.Context(), f.target, f.targetBool, 0, 0, f.body, f.loft, f.loftBool,
			f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(t.Context()))
	}
	got, err := check()
	require.NoError(t, err)
	require.True(t, got)
	tri := f.loftBool.Tris[f.loftFacet]
	v := tri[f.loftEdge]
	original := f.loftBool.Verts[v]
	f.loftBool.Verts = append([]r3.Vec(nil), f.loftBool.Verts...)
	f.loftBool.Verts[v].X = math.Nextafter(f.loftBool.Verts[v].X, math.Inf(1))
	got, err = check()
	require.NoError(t, err)
	require.False(t, got)
	f.loftBool.Verts[v] = original
	f.target.BoundaryOK = false
	got, err = check()
	require.NoError(t, err)
	require.False(t, got)
	f.target.BoundaryOK = true
	//nolint:staticcheck // Exercise the nil-context refusal.
	_, err = directLoftSharedEdge(nil, f.target, f.targetBool, 0, 0, f.body, f.loft, f.loftBool,
		f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(t.Context()))
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = directLoftSharedEdge(ctx, f.target, f.targetBool, 0, 0, f.body, f.loft, f.loftBool,
		f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(ctx))
	require.ErrorIs(t, err, context.Canceled)
	_, err = directLoftSharedEdge(t.Context(), f.target, f.targetBool, 0, 0, f.body, f.loft, f.loftBool,
		f.loftFacet, f.loftEdge, nil)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	got, err = directLoftSharedEdge(t.Context(), f.target, f.targetBool, 0, 0, new(topo.Body), f.loft, f.loftBool,
		f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)

	// Both held edges lie on the same retained source edge, but their ranges do
	// not overlap. The common source edge alone cannot certify this contact.
	a, b := f.target.Vertices[0], f.target.Vertices[1]
	point := func(scale float64) r3.Vec {
		return r3.Vec{X: a.X + scale*(b.X-a.X), Y: a.Y + scale*(b.Y-a.Y), Z: a.Z + scale*(b.Z-a.Z)}
	}
	shared := f.target.Source[0].Loops[0].CoEdges[0].Edge
	shared.End.Position = point(3)
	f.target.Vertices = append([]r3.Vec(nil), f.target.Vertices...)
	f.target.Vertices[0], f.target.Vertices[1] = point(1.5), point(2)
	f.targetBool, err = meshbool.PrepContext(t.Context(), f.target.Vertices, f.target.Triangles, []int{0, 1})
	require.NoError(t, err)
	got, err = check()
	require.NoError(t, err)
	require.False(t, got)
}

func TestCollinearSegmentsOverlap(t *testing.T) {
	for _, tc := range []struct {
		name string
		a, b [2]r3.Vec
		want bool
	}{
		{"equal", [2]r3.Vec{{}, {X: 2}}, [2]r3.Vec{{}, {X: 2}}, true},
		{"contained", [2]r3.Vec{{}, {X: 3}}, [2]r3.Vec{{X: 1}, {X: 2}}, true},
		{"reversed partial", [2]r3.Vec{{X: 2}, {}}, [2]r3.Vec{{X: 3}, {X: 1}}, true},
		{"touch endpoint", [2]r3.Vec{{}, {X: 1}}, [2]r3.Vec{{X: 1}, {X: 2}}, false},
		{"disjoint", [2]r3.Vec{{}, {X: 1}}, [2]r3.Vec{{X: 2}, {X: 3}}, false},
	} {
		t.Run(tc.name, func(t *testing.T) { require.Equal(t, tc.want, collinearSegmentsOverlap(tc.a, tc.b)) })
	}
}

func TestDirectLoftTargetFaceMeetsCapAtEdge(t *testing.T) {
	for _, tc := range []struct {
		name string
		edit func(*directLoftEdgeFixture)
		want bool
	}{
		{"exact face", nil, true},
		{"uncertain face", func(f *directLoftEdgeFixture) { f.target.FaceBound[f.target.Source[0]] = 0.001 }, false},
		{"missing face bound", func(f *directLoftEdgeFixture) { delete(f.target.FaceBound, f.target.Source[0]) }, false},
		{"third vertex in cap plane", func(f *directLoftEdgeFixture) {
			f.target.Vertices[2].Z = 0
			f.targetBool.Verts[2].Z = 0
		}, false},
		{"altered prepared vertex", func(f *directLoftEdgeFixture) {
			f.targetBool.Verts = append([]r3.Vec(nil), f.targetBool.Verts...)
			f.targetBool.Verts[2].Z = 2
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := makeDirectLoftEdgeFixture(t)
			if tc.edit != nil {
				tc.edit(&f)
			}
			got, err := directLoftTargetFaceMeetsCapAtEdge(t.Context(), f.target, f.targetBool, 0, 0,
				f.body, f.loft, f.loftBool, f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestDirectLoftCapEdgeSupport(t *testing.T) {
	for _, tc := range []struct {
		name string
		edit func(*directLoftEdgeFixture)
		want bool
	}{
		{"complete source cap", nil, true},
		{"missing cap source", func(f *directLoftEdgeFixture) {
			lp := f.body.Payload.(loftrecord.Payload)
			start, _, _ := lp.DirectCapFacetRange(f.body, 0)
			f.loft.Source = append([]*topo.Face(nil), f.loft.Source...)
			f.loft.Source[start] = nil
		}, false},
		{"altered cap facet", func(f *directLoftEdgeFixture) {
			lp := f.body.Payload.(loftrecord.Payload)
			start, _, _ := lp.DirectCapFacetRange(f.body, 0)
			f.loft.Triangles = append([][3]int(nil), f.loft.Triangles...)
			f.loft.Triangles[start][0], f.loft.Triangles[start][1] =
				f.loft.Triangles[start][1], f.loft.Triangles[start][0]
		}, false},
		{"uncertain target face", func(f *directLoftEdgeFixture) {
			f.target.FaceBound[f.target.Source[0]] = 0.001
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := makeDirectLoftEdgeFixture(t)
			if tc.edit != nil {
				tc.edit(&f)
			}
			got, err := directLoftCapEdgeSupport(t.Context(), f.target, f.targetBool, 0, 0,
				f.body, f.loft, f.loftBool, f.loftFacet, f.loftEdge, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestDirectLoftCapContactEvidence(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	lp := f.body.Payload.(loftrecord.Payload)
	start, _, ok := lp.DirectCapFacetRange(f.body, 0)
	require.True(t, ok)
	evidence := directLoftCapContactEvidence(t.Context(), f.target, f.targetBool, f.body, f.loft, f.loftBool)
	require.NotNil(t, evidence)
	got, err := evidence(f.target.Source[0], f.loft.Source[start])
	require.NoError(t, err)
	require.True(t, got)
	got, err = evidence(f.target.Source[0], new(topo.Face))
	require.NoError(t, err)
	require.False(t, got)
	f.loftBool.Verts = append([]r3.Vec(nil), f.loftBool.Verts...)
	vertex := f.loftBool.Tris[f.loftFacet][f.loftEdge]
	f.loftBool.Verts[vertex].X = math.Nextafter(f.loftBool.Verts[vertex].X, math.Inf(1))
	got, err = evidence(f.target.Source[0], f.loft.Source[start])
	require.NoError(t, err)
	require.False(t, got)
}
