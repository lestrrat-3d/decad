package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// wallWork is a work whose face 0 is f.
func wallWork(f *topo.Face) *work {
	return &work{faceIx: map[*topo.Face]int{f: 0}, r: rim{scale: 10}}
}

func tagged(s surfacegeom.Surface) *topo.Face { return &topo.Face{Surface: s, RetainedTag: true} }

// TestWallSine reads the sine of the wall's slant to an end plane from the wall's
// retained tag, and has none where the tag gives no bound.
func TestWallSine(t *testing.T) {
	t.Parallel()

	g := plane{n: r3.NewVec(0, 1, 0), o: r3.NewVec(0, 1.5, 0)}

	t.Run("a plane wall meets the plane at the sine of its normal's angle", func(t *testing.T) {
		t.Parallel()
		fr, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0.6, 0.8))
		require.NoError(t, err)
		// The wall's normal is (0, -0.8, 0.6) about: sin with g's normal (0, 1, 0).
		sine := wallWork(tagged(surfacegeom.Plane{Frame: fr})).wallSine(0, g)
		require.NotNil(t, sine)
		n := fr.N()
		want := n.Cross(g.n).Len()
		require.InDelta(t, want, sine(1), 1e-12)
		require.LessOrEqual(t, sine(1), want)
	})
	t.Run("a cylinder wall parallel to the plane, the axis on either side", func(t *testing.T) {
		t.Parallel()
		// R = 3, the axis 1.5 from the plane: sin = √(R²−h²)/R = 0.866 at the plane, less within reach.
		for _, y := range []float64{0, 3} {
			cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, y, 3), Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3)}
			sine := wallWork(tagged(cyl)).wallSine(0, g)
			require.NotNil(t, sine)
			require.InDelta(t, math.Sqrt(1-0.25), sine(0), 1e-9, "axis at y = %v", y)
			require.Less(t, sine(0.3), sine(0))
			require.InDelta(t, math.Sqrt(1-(1.8/3)*(1.8/3)), sine(0.3), 1e-9)
		}
	})
	t.Run("a bore wall is the same surface", func(t *testing.T) {
		t.Parallel()
		cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, 1.5-0.4, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)}
		sine := wallWork(tagged(cyl)).wallSine(0, g)
		require.NotNil(t, sine)
		require.InDelta(t, math.Sqrt(1-0.16), sine(0), 1e-9)
		require.Zero(t, sine(0.7), "the reach leaves the cylinder's width")
	})
	t.Run("no bound", func(t *testing.T) {
		t.Parallel()
		tilted := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0.1, math.Sqrt(0.99)), Radius: units.Millimeters(3)}
		require.Nil(t, wallWork(tagged(tilted)).wallSine(0, g), "an axis not parallel to the plane")
		cone := surfacegeom.Cone{Origin: r3.Vec{}, Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3), HalfAngle: units.Radians(0.1)}
		require.Nil(t, wallWork(tagged(cone)).wallSine(0, g))
		cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, 0, 3), Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3)}
		for _, mod := range []func(f *topo.Face){
			func(f *topo.Face) { f.RetainedTag = false },
			func(f *topo.Face) { f.TagDeparture = 1e-9 },
			func(f *topo.Face) { f.NormalBound = -1 },
			func(f *topo.Face) { f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
		} {
			f := tagged(cyl)
			mod(f)
			require.Nil(t, wallWork(f).wallSine(0, g))
		}
		require.Nil(t, wallWork(tagged(cyl)).wallSine(1, g), "no such face")
	})
}
