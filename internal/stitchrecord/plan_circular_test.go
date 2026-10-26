package stitchrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// vertexAt is a vertex at (x, 0, 0) with the given bound and curve token.
func vertexAt(x float64, bound units.Value, tok denotation.CurveToken) *topo.Vertex {
	return &topo.Vertex{Position: r3.NewVec(x, 0, 0), Bound: bound, Denot: tok}
}

// faceOf returns a face whose only loop walks e, and links e to it.
func faceOf(e *topo.Edge) *topo.Face {
	f := &topo.Face{Loops: []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{{Edge: e, Forward: true}}}}}
	e.Faces = []*topo.Face{f}
	return f
}

func arcEdge(a, b *topo.Vertex, bounded bool, bound float64) *topo.Edge {
	return &topo.Edge{
		Curve:        surfacegeom.Arc3{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(10)},
		Start:        a,
		End:          b,
		CurveBounded: bounded,
		CurveBound:   bound,
	}
}

// TestBuildPlanZeroBoundArcs checks both halves of the zero-bound arm: the
// edge's own curve bound and its end vertices' bounds.
func TestBuildPlanZeroBoundArcs(t *testing.T) {
	t.Parallel()
	zero := units.Millimeters(0)
	small := units.Millimeters(1e-3)
	tok := denotation.NewCurve(7)
	tests := []struct {
		name    string
		bounded bool
		bound   float64
		vBound  units.Value
		vTok    denotation.CurveToken
		groups  int
	}{
		{"zero bound welds", true, 0, zero, denotation.CurveToken{}, 1},
		{"curve bound unstated", false, 0, zero, denotation.CurveToken{}, 0},
		{"nonzero curve bound", true, 1e-3, zero, denotation.CurveToken{}, 0},
		{"bounded vertices sharing a token", true, 0, small, tok, 0},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			var faces []*topo.Face
			for range 2 {
				a, b := vertexAt(10, tc.vBound, tc.vTok), vertexAt(-10, tc.vBound, tc.vTok)
				faces = append(faces, faceOf(arcEdge(a, b, tc.bounded, tc.bound)))
			}
			require.Equal(t, tc.groups, stitchrecord.BuildPlan(faces).Groups)
		})
	}
}

// TestBuildPlanLineKeyClaims checks that a circular pair claims weld keys
// apart from straight edges, and that any other curved claim still counts
// against a straight pair.
func TestBuildPlanLineKeyClaims(t *testing.T) {
	t.Parallel()
	zero := units.Millimeters(0)
	build := func(extra func(a, b *topo.Vertex) *topo.Edge) *stitchrecord.Plan {
		var faces []*topo.Face
		for range 2 {
			a, b := vertexAt(10, zero, denotation.CurveToken{}), vertexAt(-10, zero, denotation.CurveToken{})
			faces = append(faces, faceOf(&topo.Edge{Curve: surfacegeom.Line3{}, Start: a, End: b}))
			faces = append(faces, faceOf(extra(a, b)))
		}
		return stitchrecord.BuildPlan(faces)
	}
	t.Run("circular pair beside a straight pair", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, 2, build(func(a, b *topo.Vertex) *topo.Edge { return arcEdge(a, b, true, 0) }).Groups)
	})
	t.Run("ellipse claim beside a straight pair", func(t *testing.T) {
		t.Parallel()
		p := build(func(a, b *topo.Vertex) *topo.Edge {
			return &topo.Edge{Curve: surfacegeom.Ellipse3{}, Start: a, End: b, Denot: denotation.NewCurve(3)}
		})
		require.Equal(t, 0, p.Groups)
	})
}
