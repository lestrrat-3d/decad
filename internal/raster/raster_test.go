package raster_test

import (
	"context"
	"image"
	"image/color"
	"math"
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

func TestDrawSegmentsDrawsALineBetweenItsEnds(t *testing.T) {
	t.Parallel()
	// A 10 x 10 mm square seen from above fills the image but for its margin, so
	// the image plane's origin maps to the middle of the 100-pixel image.
	verts := []r3.Vec{{X: -5, Y: -5}, {X: 5, Y: -5}, {X: 5, Y: 5}, {X: -5, Y: 5}}
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}}
	proj, err := raster.NewProjection(verts, cam, 100, 100)
	require.NoError(t, err)
	px, py := proj.Plane(0, 0)
	require.InDelta(t, 50, px, 1e-9)
	require.InDelta(t, 50, py, 1e-9)
	img := image.NewRGBA(image.Rect(0, 0, 100, 100))
	red := color.RGBA{R: 255, A: 255}
	// The scale is 9 px per mm (a 5% margin each side leaves 90 px for 10 mm).
	raster.DrawSegments(img, proj, [][2][2]float64{{{-5, 0}, {5, 0}}}, red)
	for x := 5; x <= 95; x++ {
		require.Equal(t, red, img.RGBAAt(x, 50), "pixel %d of the horizontal line", x)
	}
	require.Equal(t, color.RGBA{}, img.RGBAAt(50, 49), "the line is one pixel wide")
	require.Equal(t, color.RGBA{}, img.RGBAAt(50, 51))
	// A vertical segment up the image runs toward smaller y.
	raster.DrawSegments(img, proj, [][2][2]float64{{{0, 0}, {0, 5}}}, red)
	require.Equal(t, red, img.RGBAAt(50, 10))
	// A segment past the image is clipped without a panic.
	raster.DrawSegments(img, proj, [][2][2]float64{{{-500, 0}, {500, 500}}}, red)
}

// twoSquares are two 4 x 4 squares facing +Z, one at z = 10 on the left (x from
// -6 to -2) and one at z = 0 on the right (x from 2 to 6), seen from above.
func twoSquares() ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		{X: -6, Y: -2, Z: 10}, {X: -2, Y: -2, Z: 10}, {X: -2, Y: 2, Z: 10}, {X: -6, Y: 2, Z: 10},
		{X: 2, Y: -2, Z: 0}, {X: 6, Y: -2, Z: 0}, {X: 6, Y: 2, Z: 0}, {X: 2, Y: 2, Z: 0},
	}
	return v, [][3]int{{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}}
}

// coveredColumns counts the columns of img in [from, to) with a non-white pixel
// in the middle row.
func coveredColumns(img *image.RGBA, from, to int) int {
	n := 0
	for x := from; x < to; x++ {
		if img.RGBAAt(x, img.Bounds().Dy()/2) != raster.Background {
			n++
		}
	}
	return n
}

func TestPerspectiveMakesANearerSquareLarger(t *testing.T) {
	t.Parallel()
	v, tris := twoSquares()
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}}
	ortho, err := raster.Render(t.Context(), v, tris, cam, 200, 200)
	require.NoError(t, err)
	// Orthographic: both squares are 4 mm wide, so they cover the same columns.
	require.InDelta(t, coveredColumns(ortho, 0, 100), coveredColumns(ortho, 100, 200), 1)

	cam.FieldOfView = math.Pi / 3
	persp, err := raster.Render(t.Context(), v, tris, cam, 200, 200)
	require.NoError(t, err)
	// The eye is 16.12 mm above the middle of the bounds at z = 5 (the bounding
	// sphere's radius is √65 and sin 30° is a half), so z = 21.12. The near square
	// is 11.12 mm from it and the far one 21.12 mm. The projected x spans are
	// -0.5396 to -0.1799 and 0.0947 to 0.2841, fitted into 180 px at 218.5 px per unit.
	near, far := coveredColumns(persp, 0, 100), coveredColumns(persp, 100, 200)
	require.InDelta(t, 78.6, near, 1.5)
	require.InDelta(t, 41.4, far, 1.5)
	require.Greater(t, float64(near)/float64(far), 1.8)
}

func TestPerspectiveNearerTriangleHidesAFarOne(t *testing.T) {
	t.Parallel()
	// Two squares on the same line of sight: the near one, at z = 10, is smaller
	// than the far one at z = 0, so it shows against it as a frame of the far one.
	// The near square faces the camera (blue) and the far one faces away (orange),
	// so which is in front shows in the colour, in either drawing order.
	v := []r3.Vec{
		{X: -4, Y: -4, Z: 0}, {X: 4, Y: -4, Z: 0}, {X: 4, Y: 4, Z: 0}, {X: -4, Y: 4, Z: 0},
		{X: -1, Y: -1, Z: 10}, {X: 1, Y: -1, Z: 10}, {X: 1, Y: 1, Z: 10}, {X: -1, Y: 1, Z: 10},
	}
	far := [][3]int{{0, 2, 1}, {0, 3, 2}}
	near := [][3]int{{4, 5, 6}, {4, 6, 7}}
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: math.Pi / 3}
	for _, order := range [][][3]int{append(append([][3]int{}, far...), near...), append(append([][3]int{}, near...), far...)} {
		img, err := raster.Render(t.Context(), v, order, cam, 200, 200)
		require.NoError(t, err)
		centre, frame := img.RGBAAt(100, 100), img.RGBAAt(30, 100)
		require.Greater(t, centre.B, centre.R, "the middle shows the near, blue face")
		require.Greater(t, frame.R, frame.B, "the rim shows the far, orange face")
	}
}

func TestPerspectiveFieldOfViewLimits(t *testing.T) {
	t.Parallel()
	v, tris := twoSquares()
	for _, fov := range []float64{-0.5, math.Pi, 4, math.NaN()} {
		_, err := raster.Render(t.Context(), v, tris, raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: fov}, 100, 100)
		require.ErrorIs(t, err, raster.ErrDegenerate, "field of view %v", fov)
	}
}

func TestPerspectiveBackFacesFollowTheLineOfSight(t *testing.T) {
	t.Parallel()
	// A square seen from above, facing down: its normal points away from the
	// camera, so it is a back face, orange.
	v := []r3.Vec{{X: -2, Y: -2}, {X: -2, Y: 2}, {X: 2, Y: 2}, {X: 2, Y: -2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	img, err := raster.Render(t.Context(), v, tris, raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: math.Pi / 3}, 100, 100)
	require.NoError(t, err)
	got := img.RGBAAt(50, 50)
	require.Greater(t, got.R, got.B, "a back face is orange")
}

func TestDrawDashedSkipsAfterEachDash(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: -5, Y: -5}, {X: 5, Y: -5}, {X: 5, Y: 5}, {X: -5, Y: 5}}
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}}
	proj, err := raster.NewProjection(verts, cam, 100, 100)
	require.NoError(t, err)
	img := image.NewRGBA(image.Rect(0, 0, 100, 100))
	red := color.RGBA{R: 255, A: 255}
	// A horizontal line from pixel 5 to pixel 95, drawn 4 pixels and skipped 3, from its start.
	raster.DrawDashed(img, proj, [][2][2]float64{{{-5, 0}, {5, 0}}}, red, 4, 3)
	// A dash's end pixels can fall one pixel either way as the step positions round, so the
	// pixels inside each dash and inside each gap are the ones checked.
	for x := 5; x <= 95; x++ {
		switch (x - 5) % 7 {
		case 1, 2:
			require.Equal(t, red, img.RGBAAt(x, 50), "pixel %d inside a dash", x)
		case 5:
			require.Equal(t, color.RGBA{}, img.RGBAAt(x, 50), "pixel %d inside a gap", x)
		}
	}
	// Each segment restarts its dash, so a second one drawn down from its start begins on.
	raster.DrawDashed(img, proj, [][2][2]float64{{{0, 5}, {0, -5}}}, red, 4, 3)
	require.Equal(t, red, img.RGBAAt(50, 5))
	require.Equal(t, color.RGBA{}, img.RGBAAt(50, 9))
	// A segment past the image is clipped without a panic, and an off of zero is solid.
	raster.DrawDashed(img, proj, [][2][2]float64{{{-500, 0}, {500, 500}}}, red, 4, 0)
}

// boxCorners are the corners of a 4 x 4 x 10 mm box on the origin. Its bounding
// sphere has radius √132/2, so a 60 degree perspective eye sits √132 mm from the
// middle, z = 5, straight above it.
func boxCorners() []r3.Vec {
	return []r3.Vec{{X: -2, Y: -2}, {X: 2, Y: 2, Z: 10}}
}

func TestProjectionPlanePointDividesByTheDistanceFromTheEye(t *testing.T) {
	t.Parallel()
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: math.Pi / 3}
	persp, err := raster.NewProjection(boxCorners(), cam, 200, 200)
	require.NoError(t, err)
	ortho, err := raster.NewProjection(boxCorners(), topView, 200, 200)
	require.NoError(t, err)
	eyeZ := 5 + math.Sqrt(132)/math.Sin(math.Pi/6)/2
	for _, v := range []r3.Vec{{X: 2, Y: -1, Z: 10}, {X: -1.5, Y: 2, Z: 0}, {X: 0.5, Y: 0.25, Z: 4}} {
		x, y := persp.PlanePoint(v)
		require.InDelta(t, v.X/(eyeZ-v.Z), x, 1e-12)
		require.InDelta(t, v.Y/(eyeZ-v.Z), y, 1e-12)
		ox, oy := ortho.PlanePoint(v)
		require.InDelta(t, v.X, ox, 1e-12, "an orthographic camera keeps the right component")
		require.InDelta(t, v.Y, oy, 1e-12)
		// Plane maps the point to the pixel Project does.
		px, py := persp.Plane(x, y)
		qx, qy, _ := persp.Project(v)
		require.InDelta(t, qx, px, 1e-9)
		require.InDelta(t, qy, py, 1e-9)
	}
}

func TestProjectionFacingAtReadsTheLineOfSight(t *testing.T) {
	t.Parallel()
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: math.Pi / 3}
	persp, err := raster.NewProjection(boxCorners(), cam, 200, 200)
	require.NoError(t, err)
	ortho, err := raster.NewProjection(boxCorners(), topView, 200, 200)
	require.NoError(t, err)
	eyeZ := 5 + math.Sqrt(132)/math.Sin(math.Pi/6)/2
	wall, at := r3.Vec{X: 1}, r3.Vec{X: 2, Z: 5}
	require.Zero(t, persp.Facing(wall), "the axis sees a side wall edge on")
	require.InDelta(t, -2, persp.FacingAt(wall, at), 1e-12, "the eye over the middle sees +X's wall from behind: -n·(at-eye) = -2")
	require.InDelta(t, 0, ortho.FacingAt(wall, at), 1e-12, "orthographic is Facing")
	top := r3.Vec{Z: 1}
	require.InDelta(t, eyeZ-5, persp.FacingAt(top, r3.Vec{X: 2, Z: 5}), 1e-12)
	require.InDelta(t, 1, ortho.FacingAt(top, r3.Vec{X: 2, Z: 5}), 1e-12)
	require.Negative(t, persp.FacingAt(r3.Vec{Z: -1}, r3.Vec{Z: 0}), "a face turned down looks away")
}

func TestProjectionDepthToleranceIsTwoPixelsAtTheDepth(t *testing.T) {
	t.Parallel()
	cam := raster.Camera{Direction: r3.Vec{Z: -1}, Up: r3.Vec{Y: 1}, FieldOfView: math.Pi / 3}
	persp, err := raster.NewProjection(boxCorners(), cam, 200, 200)
	require.NoError(t, err)
	ortho, err := raster.NewProjection(boxCorners(), topView, 200, 200)
	require.NoError(t, err)
	require.InDelta(t, 2*ortho.PixelSize(), ortho.DepthTolerance(7), 1e-15, "the same everywhere")
	// Depth is minus the reciprocal of the distance, so two pixels' length d·2·PixelSize at distance d is that over d².
	for _, distance := range []float64{8, 20} {
		_, _, depth := persp.Project(r3.Vec{Z: 5 + math.Sqrt(132)/math.Sin(math.Pi/6)/2 - distance})
		require.InDelta(t, 2*persp.PixelSize()/distance, persp.DepthTolerance(depth), 1e-12)
	}
}
