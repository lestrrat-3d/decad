package surfacegeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/stretchr/testify/require"
)

func TestSurfaceTypeName(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		surface surfacegeom.Surface
		want    string
	}{
		{nil, "<nil>"},
		{surfacegeom.Plane{}, "decad.Plane"},
		{surfacegeom.Cylinder{}, "decad.Cylinder"},
		{surfacegeom.Cone{}, "decad.Cone"},
		{surfacegeom.Sphere{}, "decad.Sphere"},
		{surfacegeom.Torus{}, "decad.Torus"},
		{surfacegeom.NURBSSurface{}, "decad.NURBSSurface"},
		{surfacegeom.Faceted{}, "decad.Faceted"},
	} {
		require.Equal(t, tc.want, surfacegeom.SurfaceTypeName(tc.surface))
	}
}

func TestCurveTypeName(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		curve surfacegeom.Curve
		want  string
	}{
		{nil, "<nil>"},
		{surfacegeom.Line3{}, "decad.Line3"},
		{surfacegeom.Circle3{}, "decad.Circle3"},
		{surfacegeom.Arc3{}, "decad.Arc3"},
		{surfacegeom.Ellipse3{}, "decad.Ellipse3"},
		{surfacegeom.FilletMiter3{}, "decad.FilletMiter3"},
		{surfacegeom.NURBSCurve{}, "decad.NURBSCurve"},
		{surfacegeom.FacetedCurve{}, "decad.FacetedCurve"},
	} {
		require.Equal(t, tc.want, surfacegeom.CurveTypeName(tc.curve))
	}
}
