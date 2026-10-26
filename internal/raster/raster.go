// Package raster draws a triangle mesh as an orthographic, flat-shaded image
// with a depth buffer (agent-docs/png-export-design.md). It works on plain
// millimetre data and touches no body or document.
package raster

import (
	"context"
	"errors"
	"fmt"
	"image"
	"image/color"
	"math"

	"github.com/lestrrat-3d/r3"
)

// ErrDegenerate marks a camera or image size that draws nothing: a zero or
// non-finite direction, an up vector parallel to the direction, or a size
// outside the limits.
var ErrDegenerate = errors.New("raster: degenerate view")

// Image size limits, in pixels along each side.
const (
	MinSize = 16
	MaxSize = 8192
)

// Colours of the three surface classes and the background.
var (
	// Front is the base colour of a face that looks toward the camera.
	Front = color.RGBA{R: 120, G: 160, B: 200, A: 255}
	// Back is the base colour of a face that looks away from the camera, which
	// shows only where the surface is open or wound inside out.
	Back = color.RGBA{R: 220, G: 130, B: 60, A: 255}
	// Background is the colour of a pixel no triangle covers.
	Background = color.RGBA{R: 255, G: 255, B: 255, A: 255}
)

const (
	// ambient is the shade of a face the light does not reach.
	ambient = 0.3
	// marginFraction is the border left clear on each side, as a fraction of
	// the image side.
	marginFraction = 0.05
)

// Camera is an orthographic camera. Direction is the way it looks, from the eye
// into the scene, and Up sets the image's vertical. Neither need be a unit
// vector.
type Camera struct {
	Direction, Up r3.Vec
}

// Light offsets of the key light from the camera, in the camera's own axes: a
// headlight raised 0.5 and moved 0.35 to the left, so faces turned differently
// from the camera take different shades even when they make equal angles with
// the view direction.
const (
	lightUp   = 0.5
	lightLeft = 0.35
)

// axes returns the unit direction, right and up of the camera.
func (c Camera) axes() (dir, right, up r3.Vec, err error) {
	dir, ok := c.Direction.Normalize()
	if !ok {
		return dir, right, up, fmt.Errorf("%w: the view direction has no length", ErrDegenerate)
	}
	right, ok = dir.Cross(c.Up).Normalize()
	if !ok {
		return dir, right, up, fmt.Errorf("%w: the up vector is parallel to the view direction", ErrDegenerate)
	}
	return dir, right, right.Cross(dir), nil
}

type screenVertex struct{ x, y, depth float64 }

// Shaded returns the colour a face with the given outward normal takes under
// the camera's light: the Front colour for a face looking toward the camera and
// the Back colour for one looking away, scaled between ambient and full by the
// share of the light the face catches. normal need not be a unit vector.
func Shaded(cam Camera, normal r3.Vec) (color.RGBA, error) {
	dir, right, up, err := cam.axes()
	if err != nil {
		return color.RGBA{}, err
	}
	n, ok := normal.Normalize()
	if !ok {
		return color.RGBA{}, fmt.Errorf("%w: the face normal has no length", ErrDegenerate)
	}
	return shade(dir, right, up, n), nil
}

// shade colours a face with the unit normal n.
func shade(dir, right, up, n r3.Vec) color.RGBA {
	base := Front
	if n.Dot(dir) > 0 { // the normal points away from the camera: a back face
		base = Back
		n = n.Scale(-1)
	}
	light, _ := dir.Scale(-1).Add(up.Scale(lightUp)).Sub(right.Scale(lightLeft)).Normalize()
	level := ambient + (1-ambient)*math.Max(0, n.Dot(light))
	return color.RGBA{
		R: uint8(math.Round(float64(base.R) * level)),
		G: uint8(math.Round(float64(base.G) * level)),
		B: uint8(math.Round(float64(base.B) * level)),
		A: 255,
	}
}

// Render draws the triangles. The camera's frame decides the image's axes; the
// mesh is centred and scaled to fit with a margin, keeping its proportions. A
// triangle takes the colour Shaded gives for its normal.
func Render(ctx context.Context, verts []r3.Vec, tris [][3]int, cam Camera, width, height int) (*image.RGBA, error) {
	if width < MinSize || width > MaxSize || height < MinSize || height > MaxSize {
		return nil, fmt.Errorf("%w: an image side must lie in [%d, %d] pixels, got %d x %d",
			ErrDegenerate, MinSize, MaxSize, width, height)
	}
	dir, right, up, err := cam.axes()
	if err != nil {
		return nil, err
	}
	if len(verts) == 0 || len(tris) == 0 {
		return nil, fmt.Errorf("%w: there are no triangles to draw", ErrDegenerate)
	}
	for _, t := range tris {
		for _, i := range t {
			if i < 0 || i >= len(verts) {
				return nil, fmt.Errorf("%w: a triangle names vertex %d of %d", ErrDegenerate, i, len(verts))
			}
		}
	}
	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, v := range verts {
		x, y := right.Dot(v), up.Dot(v)
		minX, maxX = math.Min(minX, x), math.Max(maxX, x)
		minY, maxY = math.Min(minY, y), math.Max(maxY, y)
	}
	margin := marginFraction * float64(min(width, height))
	spanX, spanY := maxX-minX, maxY-minY
	scale := math.Inf(1)
	if spanX > 0 {
		scale = math.Min(scale, (float64(width)-2*margin)/spanX)
	}
	if spanY > 0 {
		scale = math.Min(scale, (float64(height)-2*margin)/spanY)
	}
	if math.IsInf(scale, 1) {
		scale = 1
	}
	centreX, centreY := (minX+maxX)/2, (minY+maxY)/2
	screen := make([]screenVertex, len(verts))
	for i, v := range verts {
		screen[i] = screenVertex{
			x:     float64(width)/2 + (right.Dot(v)-centreX)*scale,
			y:     float64(height)/2 - (up.Dot(v)-centreY)*scale,
			depth: dir.Dot(v),
		}
	}

	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for i := range img.Pix {
		img.Pix[i] = 0xff // opaque white
	}
	depths := make([]float64, width*height)
	for i := range depths {
		depths[i] = math.Inf(1)
	}
	for ti, t := range tris {
		if ti%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		normal, ok := verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Normalize()
		if !ok {
			continue // a triangle of no area draws nothing
		}
		fill := shade(dir, right, up, normal)
		fillTriangle(img, depths, screen[t[0]], screen[t[1]], screen[t[2]], fill)
	}
	return img, nil
}

// fillTriangle colours every pixel whose centre lies in the triangle and that
// is nearer than what is drawn there.
func fillTriangle(img *image.RGBA, depths []float64, a, b, c screenVertex, fill color.RGBA) {
	area := (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
	if area == 0 {
		return
	}
	bounds := img.Bounds()
	x0 := max(bounds.Min.X, int(math.Floor(min(a.x, b.x, c.x))))
	x1 := min(bounds.Max.X-1, int(math.Ceil(max(a.x, b.x, c.x))))
	y0 := max(bounds.Min.Y, int(math.Floor(min(a.y, b.y, c.y))))
	y1 := min(bounds.Max.Y-1, int(math.Ceil(max(a.y, b.y, c.y))))
	width := bounds.Dx()
	for y := y0; y <= y1; y++ {
		py := float64(y) + 0.5
		for x := x0; x <= x1; x++ {
			px := float64(x) + 0.5
			// Barycentric weights from the signed areas of the sub-triangles.
			wa := ((b.x-px)*(c.y-py) - (b.y-py)*(c.x-px)) / area
			wb := ((c.x-px)*(a.y-py) - (c.y-py)*(a.x-px)) / area
			wc := 1 - wa - wb
			if wa < 0 || wb < 0 || wc < 0 {
				continue
			}
			depth := wa*a.depth + wb*b.depth + wc*c.depth
			at := y*width + x
			if depth >= depths[at] {
				continue
			}
			depths[at] = depth
			img.SetRGBA(x, y, fill)
		}
	}
}
