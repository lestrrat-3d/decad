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

// Camera is a camera. Direction is the way it looks, from the eye into the
// scene, and Up sets the image's vertical. Neither need be a unit vector.
// FieldOfView, in radians, is zero for an orthographic camera. A positive one,
// below π, makes the camera perspective: its eye sits on the line through the
// middle of the mesh's bounds along Direction, as near as keeps the mesh's
// bounding sphere inside the field of view.
type Camera struct {
	Direction, Up r3.Vec
	FieldOfView   float64
}

// Light offsets of the key light from the camera, in the camera's own axes: a
// headlight raised 0.5 and moved 0.35 to the left, so faces turned differently
// from the camera take different shades even when they make equal angles with
// the view direction.
const (
	lightUp   = 0.5
	lightLeft = 0.35
)

// Axes returns the camera's unit right and up vectors and its unit direction.
// A camera that draws nothing, with no direction or an up vector parallel to
// it, returns the zero vectors.
func (c Camera) Axes() (right, up, dir r3.Vec) {
	dir, right, up, err := c.axes()
	if err != nil {
		return r3.Vec{}, r3.Vec{}, r3.Vec{}
	}
	return right, up, dir
}

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
	return shade(Front, dir, dir, right, up, n), nil
}

// shade colours a face with the unit normal n. view is the direction from the
// eye to the face: dir for an orthographic camera and the line of sight to the
// face for a perspective one. A face whose normal points along view looks away.
func shade(front color.RGBA, view, dir, right, up, n r3.Vec) color.RGBA {
	base := front
	if n.Dot(view) > 0 { // the normal points away from the camera: a back face
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

// Projection maps world points to image pixels and depth for one camera and
// image size. The mesh it was built from is centred and scaled to fit with a
// margin, keeping its proportions.
type Projection struct {
	dir, right, up r3.Vec
	scale          float64
	centreX        float64
	centreY        float64
	Width, Height  int
	// eye is the camera's position when it is perspective, and perspective
	// says so. A perspective projection works in the units of the plane at
	// distance 1 from the eye, where a point maps to its camera x and y over its
	// distance along the view direction.
	eye         r3.Vec
	perspective bool
}

// NewProjection builds the projection that fits verts into a width by height
// image under cam.
func NewProjection(verts []r3.Vec, cam Camera, width, height int) (Projection, error) {
	if width < MinSize || width > MaxSize || height < MinSize || height > MaxSize {
		return Projection{}, fmt.Errorf("%w: an image side must lie in [%d, %d] pixels, got %d x %d",
			ErrDegenerate, MinSize, MaxSize, width, height)
	}
	dir, right, up, err := cam.axes()
	if err != nil {
		return Projection{}, err
	}
	if len(verts) == 0 {
		return Projection{}, fmt.Errorf("%w: there is nothing to project", ErrDegenerate)
	}
	if cam.FieldOfView != 0 && !(cam.FieldOfView > 0 && cam.FieldOfView < math.Pi) {
		return Projection{}, fmt.Errorf("%w: a field of view must lie strictly between 0 and π radians, got %g", ErrDegenerate, cam.FieldOfView)
	}
	p := Projection{dir: dir, right: right, up: up, Width: width, Height: height}
	if cam.FieldOfView > 0 {
		lo, hi := verts[0], verts[0]
		for _, v := range verts[1:] {
			lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
			hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
		}
		centre := lo.Add(hi).Scale(0.5)
		radius := hi.Sub(lo).Len() / 2
		// A mesh of no extent still needs an eye away from it.
		distance := math.Max(radius, 1e-9) / math.Sin(cam.FieldOfView/2)
		p.eye = centre.Sub(dir.Scale(distance))
		p.perspective = true
	}
	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, v := range verts {
		x, y, _ := p.plane(v)
		minX, maxX = math.Min(minX, x), math.Max(maxX, x)
		minY, maxY = math.Min(minY, y), math.Max(maxY, y)
	}
	margin := marginFraction * float64(min(width, height))
	scale := math.Inf(1)
	if span := maxX - minX; span > 0 {
		scale = math.Min(scale, (float64(width)-2*margin)/span)
	}
	if span := maxY - minY; span > 0 {
		scale = math.Min(scale, (float64(height)-2*margin)/span)
	}
	if math.IsInf(scale, 1) {
		scale = 1
	}
	p.scale, p.centreX, p.centreY = scale, (minX+maxX)/2, (minY+maxY)/2
	return p, nil
}

// plane returns the position of v in the image plane, before fitting, and its
// depth. For an orthographic camera that is its right and up components and its
// distance along the view direction. For a perspective one it is those
// components over the distance from the eye along the view direction, and the
// depth is minus the reciprocal of that distance: still larger for a farther
// point, and linear in screen space across a triangle.
func (p Projection) plane(v r3.Vec) (x, y, depth float64) {
	if !p.perspective {
		return p.right.Dot(v), p.up.Dot(v), p.dir.Dot(v)
	}
	rel := v.Sub(p.eye)
	z := p.dir.Dot(rel)
	return p.right.Dot(rel) / z, p.up.Dot(rel) / z, -1 / z
}

// Project returns the pixel position of v, with y growing downward, and its
// depth: a larger depth is farther from the camera. The depth is the distance
// along the view direction for an orthographic camera, and only its order
// carries meaning for a perspective one.
func (p Projection) Project(v r3.Vec) (x, y, depth float64) {
	px, py, depth := p.plane(v)
	return float64(p.Width)/2 + (px-p.centreX)*p.scale, float64(p.Height)/2 - (py-p.centreY)*p.scale, depth
}

// Plane returns the pixel position of the point (x, y) of the image plane,
// measured in millimetres from the world origin's projection with x to the
// right and y up, as [Projection.Project] places a world point there. y grows
// downward in the result.
func (p Projection) Plane(x, y float64) (px, py float64) {
	return float64(p.Width)/2 + (x-p.centreX)*p.scale, float64(p.Height)/2 - (y-p.centreY)*p.scale
}

// PixelSize is the length in millimetres one pixel spans in the image plane.
func (p Projection) PixelSize() float64 { return 1 / p.scale }

// Facing is positive for a triangle with unit normal n that looks toward the
// camera, negative for one that looks away and zero for one seen edge on.
func (p Projection) Facing(n r3.Vec) float64 { return -n.Dot(p.dir) }

// checkMesh refuses an empty mesh or a triangle naming a missing vertex.
func checkMesh(verts []r3.Vec, tris [][3]int) error {
	if len(verts) == 0 || len(tris) == 0 {
		return fmt.Errorf("%w: there are no triangles to draw", ErrDegenerate)
	}
	for _, t := range tris {
		for _, i := range t {
			if i < 0 || i >= len(verts) {
				return fmt.Errorf("%w: a triangle names vertex %d of %d", ErrDegenerate, i, len(verts))
			}
		}
	}
	return nil
}

// Render draws the triangles. The camera's frame decides the image's axes; the
// mesh is centred and scaled to fit with a margin, keeping its proportions. A
// triangle takes the colour Shaded gives for its normal.
func Render(ctx context.Context, verts []r3.Vec, tris [][3]int, cam Camera, width, height int) (*image.RGBA, error) {
	return RenderTinted(ctx, verts, tris, nil, cam, width, height)
}

// RenderTinted is Render with a front colour for each triangle: tints[i] is the
// base colour of triangle i when it looks toward the camera, in place of Front.
// A back face keeps the Back colour. tints is nil, or as long as tris.
func RenderTinted(ctx context.Context, verts []r3.Vec, tris [][3]int, tints []color.RGBA, cam Camera, width, height int) (*image.RGBA, error) {
	proj, err := NewProjection(verts, cam, width, height)
	if err != nil {
		return nil, err
	}
	if err := checkMesh(verts, tris); err != nil {
		return nil, err
	}
	if tints != nil && len(tints) != len(tris) {
		return nil, fmt.Errorf("%w: %d tints for %d triangles", ErrDegenerate, len(tints), len(tris))
	}
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for i := range img.Pix {
		img.Pix[i] = 0xff // opaque white
	}
	if _, err := draw(ctx, verts, tris, tints, proj, img); err != nil {
		return nil, err
	}
	return img, nil
}

// DepthBuffer draws the triangles into a depth buffer and returns it, row by
// row from the top of the image: the depth of the nearest triangle at each
// pixel, or +Inf where none covers it.
func DepthBuffer(ctx context.Context, verts []r3.Vec, tris [][3]int, proj Projection) ([]float64, error) {
	if err := checkMesh(verts, tris); err != nil {
		return nil, err
	}
	return draw(ctx, verts, tris, nil, proj, nil)
}

// draw rasterizes the triangles, colouring img when it is not nil, and returns
// the depth buffer.
func draw(ctx context.Context, verts []r3.Vec, tris [][3]int, tints []color.RGBA, proj Projection, img *image.RGBA) ([]float64, error) {
	screen := make([]screenVertex, len(verts))
	for i, v := range verts {
		x, y, depth := proj.Project(v)
		screen[i] = screenVertex{x: x, y: y, depth: depth}
	}
	depths := make([]float64, proj.Width*proj.Height)
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
		fill := color.RGBA{}
		if img != nil {
			view := proj.dir
			if proj.perspective {
				centre := verts[t[0]].Add(verts[t[1]]).Add(verts[t[2]]).Scale(1.0 / 3)
				view = centre.Sub(proj.eye)
			}
			front := Front
			if tints != nil {
				front = tints[ti]
			}
			fill = shade(front, view, proj.dir, proj.right, proj.up, normal)
		}
		fillTriangle(img, depths, proj.Width, screen[t[0]], screen[t[1]], screen[t[2]], fill)
	}
	return depths, nil
}

// fillTriangle records every pixel whose centre lies in the triangle and that
// is nearer than what is drawn there, colouring img when it is not nil.
func fillTriangle(img *image.RGBA, depths []float64, width int, a, b, c screenVertex, fill color.RGBA) {
	area := (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
	if area == 0 {
		return
	}
	height := len(depths) / width
	x0 := max(0, int(math.Floor(min(a.x, b.x, c.x))))
	x1 := min(width-1, int(math.Ceil(max(a.x, b.x, c.x))))
	y0 := max(0, int(math.Floor(min(a.y, b.y, c.y))))
	y1 := min(height-1, int(math.Ceil(max(a.y, b.y, c.y))))
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
			if img != nil {
				img.SetRGBA(x, y, fill)
			}
		}
	}
}

// DrawSegments draws each segment, given in the image plane as for
// [Projection.Plane], one pixel wide in colour c over img. A segment's pixels are
// taken at steps of about half a pixel along it, each rounded down to the pixel
// that holds the point, and those outside the image are skipped.
func DrawSegments(img *image.RGBA, proj Projection, segments [][2][2]float64, c color.RGBA) {
	for _, s := range segments {
		ax, ay := proj.Plane(s[0][0], s[0][1])
		bx, by := proj.Plane(s[1][0], s[1][1])
		steps := int(math.Ceil(2 * math.Max(math.Abs(bx-ax), math.Abs(by-ay))))
		for i := 0; i <= max(steps, 1); i++ {
			t := float64(i) / float64(max(steps, 1))
			x, y := int(math.Floor(ax+(bx-ax)*t)), int(math.Floor(ay+(by-ay)*t))
			if x >= 0 && y >= 0 && x < proj.Width && y < proj.Height {
				img.SetRGBA(x, y, c)
			}
		}
	}
}
