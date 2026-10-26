package raster_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// topView looks down the -Z axis with +Y up the image.
var topView = raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}

// square returns two triangles for the 10 x 10 square at height z, wound
// counter-clockwise seen from +Z, so its normal is +Z.
func square(z float64) ([]r3.Vec, [][3]int) {
	const s = 10
	return []r3.Vec{r3.NewVec(0, 0, z), r3.NewVec(s, 0, z), r3.NewVec(s, s, z), r3.NewVec(0, s, z)},
		[][3]int{{0, 1, 2}, {0, 2, 3}}
}

func TestRenderFillsTheMarginedImage(t *testing.T) {
	t.Parallel()
	v, tris := square(0)
	img, err := raster.Render(t.Context(), v, tris, topView, 100, 100)
	require.NoError(t, err)
	front, err := raster.Shaded(topView, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	// A 5% margin on a 100 px image leaves the square 90 px wide, at 5 to 95.
	require.Equal(t, front, img.RGBAAt(50, 50))
	require.Equal(t, raster.Background, img.RGBAAt(2, 50), "inside the margin")
	require.Equal(t, raster.Background, img.RGBAAt(97, 50))
	require.Equal(t, front, img.RGBAAt(6, 6), "just inside the corner")
	require.Equal(t, front, img.RGBAAt(93, 93))
	require.Equal(t, raster.Background, img.RGBAAt(4, 4))
}

func TestRenderOrientsTheImageByTheCamera(t *testing.T) {
	t.Parallel()
	// A triangle with its right angle at the origin, its +X leg 10 long and its
	// +Y leg 4: in the image +X runs right and +Y runs up.
	v := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 4, 0)}
	img, err := raster.Render(t.Context(), v, [][3]int{{0, 1, 2}}, topView, 100, 100)
	require.NoError(t, err)
	front, err := raster.Shaded(topView, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	// The 10 x 4 box scales by 9 to 90 x 36 px, centred: x from 5 to 95, y from
	// 32 to 68, with the right angle at the lower left (5, 68) and the other
	// corners at (95, 68) and (5, 32).
	require.Equal(t, front, img.RGBAAt(10, 60), "near the right angle, left and low")
	require.Equal(t, front, img.RGBAAt(10, 41), "up the +Y leg, which runs up the image")
	require.Equal(t, front, img.RGBAAt(60, 66), "along the +X leg, which runs right")
	require.Equal(t, raster.Background, img.RGBAAt(90, 41), "past the hypotenuse, upper right")
	require.Equal(t, raster.Background, img.RGBAAt(90, 60))
}

func TestShadedFacesTheLightAndTheCamera(t *testing.T) {
	t.Parallel()
	toward, err := raster.Shaded(topView, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	away, err := raster.Shaded(topView, r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	require.Greater(t, toward.B, toward.R, "a face toward the camera is bluish")
	require.Greater(t, away.R, away.B, "a face away from it is orange")
	edgeOn, err := raster.Shaded(topView, r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	require.Less(t, edgeOn.B, toward.B, "a face turned from the light is darker")
	// The key light is raised and to the left, so a face turned up the image
	// is lit more than one turned down it, and one turned left more than right.
	up, _ := raster.Shaded(topView, r3.NewVec(0, 1, 1))
	down, _ := raster.Shaded(topView, r3.NewVec(0, -1, 1))
	left, _ := raster.Shaded(topView, r3.NewVec(-1, 0, 1))
	right, _ := raster.Shaded(topView, r3.NewVec(1, 0, 1))
	require.Greater(t, up.B, down.B)
	require.Greater(t, left.B, right.B)
}

func TestRenderKeepsTheNearerTriangle(t *testing.T) {
	t.Parallel()
	far, farTris := square(0)
	near, nearTris := square(5)
	// The far square faces the camera. The near square is wound the other way,
	// so it faces away and draws in the back colour.
	for i := range nearTris {
		nearTris[i] = [3]int{nearTris[i][0] + 4, nearTris[i][2] + 4, nearTris[i][1] + 4}
	}
	verts := append(append([]r3.Vec(nil), far...), near...)
	tris := append(append([][3]int(nil), farTris...), nearTris...)
	img, err := raster.Render(t.Context(), verts, tris, topView, 100, 100)
	require.NoError(t, err)
	back, err := raster.Shaded(topView, r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	require.Equal(t, back, img.RGBAAt(50, 50), "the nearer back face hides the far front face")
	// Drawn in the other order the result is the same: depth, not order, decides.
	rev := append(append([][3]int(nil), nearTris...), farTris...)
	img2, err := raster.Render(t.Context(), verts, rev, topView, 100, 100)
	require.NoError(t, err)
	require.Equal(t, img.Pix, img2.Pix)
}

func TestRenderRefusals(t *testing.T) {
	t.Parallel()
	v, tris := square(0)
	cases := map[string]func() error{
		"zero direction": func() error {
			_, err := raster.Render(t.Context(), v, tris, raster.Camera{Up: r3.NewVec(0, 1, 0)}, 100, 100)
			return err
		},
		"up parallel to direction": func() error {
			_, err := raster.Render(t.Context(), v, tris, raster.Camera{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 0, 2)}, 100, 100)
			return err
		},
		"image too small": func() error {
			_, err := raster.Render(t.Context(), v, tris, topView, 8, 100)
			return err
		},
		"image too large": func() error {
			_, err := raster.Render(t.Context(), v, tris, topView, 100, raster.MaxSize+1)
			return err
		},
		"no triangles": func() error {
			_, err := raster.Render(t.Context(), v, nil, topView, 100, 100)
			return err
		},
		"index out of range": func() error {
			_, err := raster.Render(t.Context(), v, [][3]int{{0, 1, 9}}, topView, 100, 100)
			return err
		},
	}
	for name, call := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			require.ErrorIs(t, call(), raster.ErrDegenerate)
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := raster.Render(ctx, v, tris, topView, 100, 100)
	require.ErrorIs(t, err, context.Canceled)
}
