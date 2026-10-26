package raster_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPerspectiveProjectionKeepsFittedEdgePixels(t *testing.T) {
	t.Parallel()
	var verts []r3.Vec
	for _, z := range []float64{0, 10} {
		verts = append(verts, r3.Vec{Z: z}, r3.Vec{X: 20, Z: z},
			r3.Vec{X: 20, Y: 20, Z: z}, r3.Vec{Y: 20, Z: z})
	}
	cam := topView
	cam.FieldOfView = math.Pi / 3
	proj, err := raster.NewProjection(verts, cam, 200, 200)
	require.NoError(t, err)
	// The top square determines the fit and leaves the 10-pixel margin.
	// Its plane coordinates follow from the 15 mm bounding-sphere radius
	// and the eye's distance above the square, as PNG design V2 specifies.
	distance := 15/math.Sin(cam.FieldOfView/2) - 5
	for i, expected := range [][2]int{{10, 190}, {190, 190}, {190, 10}, {10, 10}} {
		v := verts[4+i]
		x, y, _ := proj.Project(v)
		px, py := proj.Plane((v.X-10)/distance, (v.Y-10)/distance)
		require.Equal(t, expected[0], int(math.Floor(x)), "world corner %d x = %.17g", i, x)
		require.Equal(t, expected[1], int(math.Floor(y)), "world corner %d y = %.17g", i, y)
		require.Equal(t, expected[0], int(math.Floor(px)), "plane corner %d x = %.17g", i, px)
		require.Equal(t, expected[1], int(math.Floor(py)), "plane corner %d y = %.17g", i, py)
	}
}
