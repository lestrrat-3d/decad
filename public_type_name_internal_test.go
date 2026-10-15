package decad

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/stretchr/testify/require"
)

// TestReportvocabNamesMatchPublicTypes pins reportvocab's surface and curve
// names to what %T prints for the public root variant of every surfacegeom
// variant, and for nil: internal/stitchrecord's flux refusals print these
// names where the root code printed %T.
//
// Shown to fail first: with SurfaceTypeName returning "decad.NURBSSurfaceX"
// for a NURBSSurface, the NURBSSurface row failed; with CurveTypeName
// returning "decad.Line3X" for a Line3, the Line3 row failed.
func TestReportvocabNamesMatchPublicTypes(t *testing.T) {
	t.Parallel()
	for _, s := range []surfacegeom.Surface{
		nil, surfacegeom.Plane{}, surfacegeom.Cylinder{}, surfacegeom.Cone{}, surfacegeom.Sphere{},
		surfacegeom.Torus{}, surfacegeom.NURBSSurface{}, surfacegeom.Faceted{},
	} {
		require.Equal(t, fmt.Sprintf("%T", publicSurface(s)), reportvocab.SurfaceTypeName(s))
	}
	for _, c := range []surfacegeom.Curve{
		nil, surfacegeom.Line3{}, surfacegeom.Circle3{}, surfacegeom.Arc3{}, surfacegeom.Ellipse3{},
		surfacegeom.FilletMiter3{}, surfacegeom.NURBSCurve{}, surfacegeom.FacetedCurve{},
	} {
		require.Equal(t, fmt.Sprintf("%T", publicCurve(c)), reportvocab.CurveTypeName(c))
	}
}
