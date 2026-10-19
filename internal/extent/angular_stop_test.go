package extent_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// radialFace is a unit-square face in the half-plane at angle phi about the
// Z axis, spanning radius 1..2 and height 0..1, the way a revolve's radial
// cap is built: a Plane through the axis and four Line3 edges.
func radialFace(t *testing.T, phi float64) *topo.Face {
	t.Helper()
	dir := r3.Vec{X: math.Cos(phi), Y: math.Sin(phi)}
	frame, err := r3.NewFrame(r3.Vec{}, dir, r3.Vec{Z: 1})
	require.NoError(t, err)
	at := func(r, z float64) *topo.Vertex { return &topo.Vertex{Position: dir.Scale(r).Add(r3.Vec{Z: z})} }
	corners := []*topo.Vertex{at(1, 0), at(2, 0), at(2, 1), at(1, 1)}
	loop := &topo.Loop{}
	for i, v := range corners {
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{
			Edge: &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: corners[(i+1)%len(corners)]}})
	}
	return &topo.Face{Surface: surfacegeom.Plane{Frame: frame}, Loops: []*topo.Loop{loop}}
}

func TestToFaceAngularStop(t *testing.T) {
	t.Parallel()

	// The axis is Z through the origin and the profile's own half-plane
	// points along X, so E1 = W × R0 = Y.
	stop := extent.AngularStop{W: r3.Vec{Z: 1}, R0: r3.Vec{X: 1}, E1: r3.Vec{Y: 1}}

	t.Run("senses", func(t *testing.T) {
		t.Parallel()
		tests := []struct {
			name   string
			phi    float64
			travel float64
			want   float64
		}{
			{"nearer way is Along", math.Pi / 2, 0, math.Pi / 2},
			{"nearer way is Against", 3 * math.Pi / 2, 0, -math.Pi / 2},
			{"half-turn tie is Along", math.Pi, 0, math.Pi},
			{"Along side", 3 * math.Pi / 2, 1, 3 * math.Pi / 2},
			{"Against side", math.Pi / 2, -1, math.Pi/2 - 2*math.Pi},
		}
		for _, tc := range tests {
			t.Run(tc.name, func(t *testing.T) {
				got, err := extent.ToFaceAngularStop(radialFace(t, tc.phi), stop, tc.travel)
				require.NoError(t, err)
				require.InDelta(t, tc.want, got, 1e-12)
			})
		}
	})

	t.Run("plane tolerance scales with its position", func(t *testing.T) {
		t.Parallel()
		face := radialFace(t, math.Pi/2)
		pl := face.Surface.(surfacegeom.Plane)
		// Far along the axis, a 1e-5 offset from the axis is within the
		// position-scaled tolerance of 1e-9 * 1e5.
		origin := r3.Vec{Z: 1e5}.Add(pl.Frame.N().Scale(1e-5))
		frame, err := r3.NewFrame(origin, pl.Frame.U(), pl.Frame.V())
		require.NoError(t, err)
		face.Surface = surfacegeom.Plane{Frame: frame}
		got, err := extent.ToFaceAngularStop(face, stop, 0)
		require.NoError(t, err)
		require.InDelta(t, math.Pi/2, got, 1e-12)
	})

	refusals := []struct {
		name string
		face func(*testing.T) *topo.Face
		stop extent.AngularStop
		want error
		msg  string
	}{
		{name: "faceted surface", want: decaderr.ErrUnsupported, msg: "surface is decad.Faceted,", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Surface = surfacegeom.Faceted{}
				return f
			}},
		{name: "plane not parallel to the axis", want: decaderr.ErrUnsupported, msg: "not parallel to the axis",
			stop: extent.AngularStop{W: r3.Vec{X: 1}, R0: r3.Vec{Z: 1}, E1: r3.Vec{Y: 1}},
			face: func(t *testing.T) *topo.Face { return radialFace(t, math.Pi/2) }},
		{name: "plane offset from the axis", want: decaderr.ErrUnsupported, msg: "offset from it",
			stop: extent.AngularStop{A3: r3.Vec{X: 1}, W: r3.Vec{Z: 1}, R0: r3.Vec{X: 1}, E1: r3.Vec{Y: 1}},
			face: func(t *testing.T) *topo.Face { return radialFace(t, math.Pi/2) }},
		{name: "face in the profile's own half-plane", want: decaderr.ErrDegenerate, msg: "own half-plane", stop: stop,
			face: func(t *testing.T) *topo.Face { return radialFace(t, 0) }},
		{name: "face just short of a full turn", want: decaderr.ErrDegenerate, msg: "own half-plane", stop: stop,
			face: func(t *testing.T) *topo.Face { return radialFace(t, 2*math.Pi-1e-10) }},
		{name: "face spanning both half-planes", want: decaderr.ErrDegenerate, msg: "both sides", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Loops[0].CoEdges[0].Edge.Start.Position = r3.Vec{Y: -1}
				return f
			}},
		{name: "face on the axis", want: decaderr.ErrDegenerate, msg: "no material off the revolve axis", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				for _, ce := range f.Loops[0].CoEdges {
					ce.Edge.Start.Position.X, ce.Edge.Start.Position.Y = 0, 0
				}
				return f
			}},
		{name: "unsupported boundary curve", want: decaderr.ErrUnsupported, msg: "for a decad.Ellipse3 boundary edge", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Loops[0].CoEdges[0].Edge.Curve = surfacegeom.Ellipse3{}
				return f
			}},
		{name: "start vertex counts", want: decaderr.ErrDegenerate, msg: "both sides", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Loops = []*topo.Loop{{CoEdges: []topo.CoEdge{
					{Edge: &topo.Edge{Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: r3.Vec{Y: 1}}}},
					{Edge: &topo.Edge{Curve: surfacegeom.Line3{}, End: &topo.Vertex{Position: r3.Vec{Y: -1}}}},
				}}}
				return f
			}},
		{name: "last edge counts", want: decaderr.ErrDegenerate, msg: "both sides", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Loops[0].CoEdges[3].Edge.End = &topo.Vertex{Position: r3.Vec{Y: -1}}
				return f
			}},
		{name: "inner loop counts", want: decaderr.ErrDegenerate, msg: "both sides", stop: stop,
			face: func(t *testing.T) *topo.Face {
				f := radialFace(t, math.Pi/2)
				f.Loops = append(f.Loops, &topo.Loop{CoEdges: []topo.CoEdge{
					{Edge: &topo.Edge{Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: r3.Vec{Y: -1}}}}}})
				return f
			}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := extent.ToFaceAngularStop(tc.face(t), tc.stop, 0)
			require.ErrorIs(t, err, tc.want)
			require.ErrorContains(t, err, tc.msg)
		})
	}
}
