package surfacegeom_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestTransformCurveEllipse3 places and mirrors the quarter ellipse of a
// 90-degree convex loop corner at r = 2 (docs/loop-fillet-design.md LF4): a
// semi-major axis 2·sqrt(2) along the bisector and a semi-minor axis 2, swept
// counter-clockwise about +z. The arc runs from Center + a·Major (angle 0) to
// Center + b·(Axis × Major) (angle pi/2), so a placed curve is right exactly
// when its own start and end are the images of those two points.
func TestTransformCurveEllipse3(t *testing.T) {
	t.Parallel()
	a, b := 2*math.Sqrt2, 2.0
	major := r3.NewVec(1, 1, 0).Scale(1 / math.Sqrt2)
	src := surfacegeom.Ellipse3{
		Center: r3.NewVec(2, 2, 18), Axis: r3.NewVec(0, 0, 1), Major: major,
		SemiMajor: units.Millimeters(a), SemiMinor: units.Millimeters(b),
	}
	endpoints := func(c surfacegeom.Ellipse3) (r3.Vec, r3.Vec) {
		start := c.Center.Add(c.Major.Scale(c.SemiMajor.Base()))
		end := c.Center.Add(c.Axis.Cross(c.Major).Scale(c.SemiMinor.Base()))
		return start, end
	}
	srcStart, srcEnd := endpoints(src)

	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(math.Pi/3))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(5, -7, 11))
	require.NoError(t, err)
	placement, err := rot.Then(move)
	require.NoError(t, err)

	mirrorFrame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirror, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	require.True(t, mirror.IsReflection())

	for name, xf := range map[string]r3.Transform{"identity": r3.Identity(), "placed": placement, "mirrored": mirror} {
		t.Run(name, func(t *testing.T) {
			got, err := surfacegeom.TransformCurve(src, xf)
			require.NoError(t, err)
			c, ok := got.(surfacegeom.Ellipse3)
			require.True(t, ok, "an Ellipse3 stays an Ellipse3")

			// Centre, semi-axes and plane survive the placement.
			require.True(t, c.Center.Equal(xf.Apply(src.Center), 1e-12))
			require.InDelta(t, a, c.SemiMajor.Base(), 1e-15)
			require.InDelta(t, b, c.SemiMinor.Base(), 1e-15)
			require.InDelta(t, 1, c.Axis.Len(), 1e-12)
			require.InDelta(t, 1, c.Major.Len(), 1e-12)
			require.InDelta(t, 0, c.Axis.Dot(c.Major), 1e-12)
			require.True(t, c.Major.Equal(xf.ApplyDir(major), 1e-12))
			// The plane's normal is the image of +z up to sign.
			require.InDelta(t, 1, math.Abs(c.Axis.Dot(xf.ApplyDir(src.Axis))), 1e-12)

			// The arc still runs counter-clockwise from the image of its start
			// to the image of its end.
			start, end := endpoints(c)
			require.True(t, start.Equal(xf.Apply(srcStart), 1e-12), "start %v", start)
			require.True(t, end.Equal(xf.Apply(srcEnd), 1e-12), "end %v", end)
		})
	}
}
