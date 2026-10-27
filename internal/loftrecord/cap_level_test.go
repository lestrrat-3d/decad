package loftrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capLevelFace is a cap on the plane z = level: a square loop of four Line3
// edges with corners (+-5, +-5, level), a whole Circle3 of radius 1 about
// (0, 0, level) on the z axis, a Plane tag through the origin of tag, and the
// record {xform(O + u*X + v*Y)} of the world frame at origin.
func capLevelFace(t *testing.T, level float64, tag r3.Frame, origin r3.Vec, xform r3.Transform) *topo.Face {
	t.Helper()
	record, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []*topo.Vertex{
		{Position: r3.NewVec(-5, -5, level)}, {Position: r3.NewVec(5, -5, level)},
		{Position: r3.NewVec(5, 5, level)}, {Position: r3.NewVec(-5, 5, level)},
	}
	outer := &topo.Loop{Outer: true}
	for i, v := range corners {
		edge := &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: corners[(i+1)%len(corners)]}
		outer.CoEdges = append(outer.CoEdges, topo.CoEdge{Edge: edge, Forward: true})
	}
	rim := &topo.Vertex{Position: r3.NewVec(1, 0, level)}
	hole := &topo.Loop{CoEdges: []topo.CoEdge{{Edge: &topo.Edge{
		Curve: surfacegeom.Circle3{Center: r3.NewVec(0, 0, level), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)},
		Start: rim, End: rim,
	}}}}
	return &topo.Face{
		Surface:       surfacegeom.Plane{Frame: tag},
		CapRecord:     &topo.CapRecord{Frame: record, Xform: xform},
		AxialDelta:    1e-15,
		HasAxialDelta: true,
		Loops:         []*topo.Loop{outer, hole},
	}
}

func capLevelFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return frame
}

// TestCapOnRecord pins that a loft cap is admitted as lying on its section
// plane only when its tag and every loop point it holds lie on that plane in
// exact rationals, and that every departure, however small, refuses. Shown to
// fail with each refusal leg of CapOnRecord deleted in turn: the vertex, tag
// origin, tag N, circle centre, circle axis and curve-kind legs. The tag N
// leg alone refuses a tilted cap whose tag is its own record frame: U and V
// are exactly orthogonal to the record normal, but N is their rounded cross
// product. The U and V orthogonality legs are shown to fail only together
// with the N leg, on the tilted tag: no case here has a tag whose N is exactly
// parallel to the record normal while U or V is off the plane.
func TestCapOnRecord(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	lift, err := r3.Translation(r3.NewVec(0, 0, 0.1))
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.NewVec(1, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	up := math.Nextafter(0, 1)

	t.Run("exact caps", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			name string
			face *topo.Face
		}{
			{"world plane", capLevelFace(t, 0, capLevelFrame(t, r3.Vec{}, x, y), r3.Vec{}, r3.Identity())},
			{"tag origin moved in plane", capLevelFace(t, 0, capLevelFrame(t, r3.NewVec(1e9, 0.3, 0), x, y), r3.Vec{}, r3.Identity())},
			{"tag flipped", capLevelFace(t, 0, capLevelFrame(t, r3.Vec{}, y, x), r3.Vec{}, r3.Identity())},
			{"record origin off the axis", capLevelFace(t, 2, capLevelFrame(t, r3.NewVec(0, 0, 2), x, y), r3.NewVec(1e9, 0.427, 2), r3.Identity())},
			{"exact placement", capLevelFace(t, 0.1, capLevelFrame(t, r3.NewVec(0, 0, 0.1), x, y), r3.Vec{}, lift)},
		} {
			require.True(t, loftrecord.CapOnRecord(tc.face), tc.name)
		}
	})

	t.Run("departed caps", func(t *testing.T) {
		t.Parallel()
		face := func() *topo.Face {
			return capLevelFace(t, 0, capLevelFrame(t, r3.Vec{}, x, y), r3.Vec{}, r3.Identity())
		}
		for _, tc := range []struct {
			name   string
			mutate func(f *topo.Face)
		}{
			{"vertex one subnormal off the plane", func(f *topo.Face) {
				f.Loops[0].CoEdges[2].Edge.Start.Position.Z = up
			}},
			{"tag origin off the plane", func(f *topo.Face) {
				f.Surface = surfacegeom.Plane{Frame: capLevelFrame(t, r3.NewVec(0, 0, up), x, y)}
			}},
			{"tag N rounded off a tilted record normal", func(f *topo.Face) {
				tilted := capLevelFrame(t, r3.Vec{}, r3.NewVec(1, 2, 3), r3.NewVec(-2, 1, 0.5))
				f.Surface = surfacegeom.Plane{Frame: tilted}
				f.CapRecord = &topo.CapRecord{Frame: tilted, Xform: r3.Identity()}
				f.Loops = nil
			}},
			{"tag tilted", func(f *topo.Face) {
				f.Surface = surfacegeom.Plane{Frame: capLevelFrame(t, r3.Vec{}, r3.NewVec(1, 0, 1e-12), y)}
			}},
			{"circle centre off the plane", func(f *topo.Face) {
				c := f.Loops[1].CoEdges[0].Edge.Curve.(surfacegeom.Circle3)
				c.Center.Z = up
				f.Loops[1].CoEdges[0].Edge.Curve = c
			}},
			{"circle axis tilted", func(f *topo.Face) {
				c := f.Loops[1].CoEdges[0].Edge.Curve.(surfacegeom.Circle3)
				c.Axis = r3.NewVec(1e-12, 0, 1)
				f.Loops[1].CoEdges[0].Edge.Curve = c
			}},
			{"ellipse edge", func(f *topo.Face) {
				f.Loops[1].CoEdges[0].Edge.Curve = surfacegeom.Ellipse3{
					Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Major: x,
					SemiMajor: units.Millimeters(1), SemiMinor: units.Millimeters(1),
				}
			}},
			{"no record", func(f *topo.Face) { f.CapRecord = nil }},
			{"record placed elsewhere", func(f *topo.Face) { f.CapRecord.Xform = lift }},
			{"record turned off the plane", func(f *topo.Face) { f.CapRecord.Xform = turn }},
		} {
			f := face()
			require.True(t, loftrecord.CapOnRecord(f), "the unmutated cap is admitted")
			tc.mutate(f)
			require.False(t, loftrecord.CapOnRecord(f), tc.name)
		}
	})
}
