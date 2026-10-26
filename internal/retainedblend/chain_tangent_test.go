package retainedblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// junctionAt builds the top-front line and the top corner arc of the rounded
// box (agent-docs/retained-modify-reach-design.md §1), with the corner
// cylinder of the given radius, whose origin stays at (3, 3).
func junctionAt(t *testing.T, radius float64) (line, arc *topo.Edge, v *topo.Vertex) {
	t.Helper()
	frame := func(o, u, w r3.Vec) r3.Frame {
		f, err := r3.NewFrame(o, u, w)
		require.NoError(t, err)
		return f
	}
	retained := func(s surfacegeom.Surface) *topo.Face { return &topo.Face{Surface: s, RetainedTag: true} }
	top := retained(surfacegeom.Plane{Frame: frame(r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))})
	front := retained(surfacegeom.Plane{Frame: frame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))})
	cyl := retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 3, 0), Axis: r3.NewVec(0, 0, 1),
		Radius: units.Millimeters(radius)})
	v = &topo.Vertex{Position: r3.NewVec(3, 0, 20), Bound: units.Millimeters(1e-9)}
	far := &topo.Vertex{Position: r3.NewVec(37, 0, 20)}
	side := &topo.Vertex{Position: r3.NewVec(0, 3, 20)}
	line = &topo.Edge{Curve: surfacegeom.Line3{}, RetainedCurve: true, Start: far, End: v,
		Faces: []*topo.Face{top, front}}
	arc = &topo.Edge{Curve: surfacegeom.Arc3{Center: r3.NewVec(3, 3, 20), Axis: r3.NewVec(0, 0, 1),
		Radius: units.Millimeters(3)}, RetainedCurve: true, Start: side, End: v, Faces: []*topo.Face{top, cyl}}
	return line, arc, v
}

func TestTangentJunction(t *testing.T) {
	t.Parallel()

	t.Run("exactly tangent tags prove tangent", func(t *testing.T) {
		t.Parallel()
		line, arc, v := junctionAt(t, 3)
		require.True(t, retainedblend.Chainable(line))
		require.Equal(t, clearance.DegYes, retainedblend.Tangent(line, arc, v))
		require.Equal(t, clearance.DegYes, retainedblend.Tangent(arc, line, v))
	})

	t.Run("tags one float step off are undecided", func(t *testing.T) {
		t.Parallel()
		// The cylinder's centre is one float step from the tangent position.
		line, arc, v := junctionAt(t, math.Nextafter(3, 4))
		require.Equal(t, clearance.DegUnknown, retainedblend.Tangent(line, arc, v))
	})

	t.Run("a clear miss is not tangent", func(t *testing.T) {
		t.Parallel()
		line, arc, v := junctionAt(t, 3.5)
		require.Equal(t, clearance.DegNo, retainedblend.Tangent(line, arc, v))
	})

	t.Run("a face with a stated departure is undecided", func(t *testing.T) {
		t.Parallel()
		line, arc, v := junctionAt(t, 3)
		arc.Faces[1].TagDeparture = 1e-3
		require.Equal(t, clearance.DegUnknown, retainedblend.Tangent(line, arc, v))
	})
}
