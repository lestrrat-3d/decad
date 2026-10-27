package raster

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// RenderTranslucent is [RenderBacked] with an opacity for each triangle: alpha[i], from above 0 to 1, is how much of
// triangle i's colour a pixel takes and how much of what lies behind it is kept (agent-docs/png-export-design.md
// §16). The triangles of opacity 1 are drawn as RenderBacked draws them, with a depth buffer. The rest are blended
// over that image from the farthest to the nearest, each where it is nearer than the opaque surface at the pixel, so
// a translucent body shows what is behind it, its own far side included. A triangle is placed by the mean depth of
// its vertices, so two translucent triangles that cross each other are blended in one order throughout.
func RenderTranslucent(ctx context.Context, verts []r3.Vec, tris [][3]int, tints []color.RGBA, alpha []float64,
	back color.RGBA, cam Camera, width, height int) (*image.RGBA, error) {
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
	if len(alpha) != len(tris) {
		return nil, fmt.Errorf("%w: %d opacities for %d triangles", ErrDegenerate, len(alpha), len(tris))
	}
	var opaque [][3]int
	var opaqueTints []color.RGBA
	var glass []int
	for i, a := range alpha {
		if !(a > 0 && a <= 1) {
			return nil, fmt.Errorf("%w: an opacity must lie in (0, 1], got %g", ErrDegenerate, a)
		}
		if a == 1 {
			opaque = append(opaque, tris[i])
			if tints != nil {
				opaqueTints = append(opaqueTints, tints[i])
			}
			continue
		}
		glass = append(glass, i)
	}
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for i := range img.Pix {
		img.Pix[i] = 0xff // opaque white
	}
	depths, err := draw(ctx, verts, opaque, opaqueTints, back, proj, img, nil)
	if err != nil {
		return nil, err
	}
	screen := make([]screenVertex, len(verts))
	for i, v := range verts {
		x, y, depth := proj.Project(v)
		screen[i] = screenVertex{x: x, y: y, depth: depth}
	}
	meanDepth := func(i int) float64 {
		t := tris[i]
		return (screen[t[0]].depth + screen[t[1]].depth + screen[t[2]].depth) / 3
	}
	sort.SliceStable(glass, func(a, b int) bool { return meanDepth(glass[a]) > meanDepth(glass[b]) })
	for n, i := range glass {
		if n%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		t := tris[i]
		normal, ok := verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Normalize()
		if !ok {
			continue
		}
		view := proj.dir
		if proj.perspective {
			centre := verts[t[0]].Add(verts[t[1]]).Add(verts[t[2]]).Scale(1.0 / 3)
			view = centre.Sub(proj.eye)
		}
		front := Front
		if tints != nil {
			front = tints[i]
		}
		fill := shade(front, back, view, proj.dir, proj.right, proj.up, normal, proj.gloss)
		blendTriangle(img, depths, proj.Width, screen[t[0]], screen[t[1]], screen[t[2]], fill, alpha[i])
	}
	return img, nil
}

// blendTriangle mixes fill into every pixel whose centre lies in the triangle and that is nearer than the opaque
// surface drawn there: alpha of the fill and the rest of what is already in the pixel.
func blendTriangle(img *image.RGBA, depths []float64, width int, a, b, c screenVertex, fill color.RGBA, alpha float64) {
	area := (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)
	if area == 0 {
		return
	}
	if area < 0 {
		b, c, area = c, b, -area // wind the triangle one way, so one rule settles a pixel on an edge
	}
	height := len(depths) / width
	x0 := max(0, int(math.Floor(min(a.x, b.x, c.x))))
	x1 := min(width-1, int(math.Ceil(max(a.x, b.x, c.x))))
	y0 := max(0, int(math.Floor(min(a.y, b.y, c.y))))
	y1 := min(height-1, int(math.Ceil(max(a.y, b.y, c.y))))
	mix := func(over, under uint8) uint8 {
		return uint8(math.Round(alpha*float64(over) + (1-alpha)*float64(under)))
	}
	// A pixel whose centre lies on an edge belongs to the triangle on one side of it: the one for which the edge,
	// run the way the triangle winds, points up or, level, to the left. A pixel on the edge two triangles share is
	// then blended once, where the opaque draw's depth test would have hidden a second blend.
	owns := func(from, to screenVertex) bool {
		dx, dy := to.x-from.x, to.y-from.y
		return dy < 0 || (dy == 0 && dx > 0)
	}
	ownA, ownB, ownC := owns(b, c), owns(c, a), owns(a, b)
	for y := y0; y <= y1; y++ {
		py := float64(y) + 0.5
		for x := x0; x <= x1; x++ {
			px := float64(x) + 0.5
			wa := ((b.x-px)*(c.y-py) - (b.y-py)*(c.x-px)) / area
			wb := ((c.x-px)*(a.y-py) - (c.y-py)*(a.x-px)) / area
			wc := 1 - wa - wb
			if wa < 0 || wb < 0 || wc < 0 || (wa == 0 && !ownA) || (wb == 0 && !ownB) || (wc == 0 && !ownC) {
				continue
			}
			depth := wa*a.depth + wb*b.depth + wc*c.depth
			if depth >= depths[y*width+x] {
				continue
			}
			under := img.RGBAAt(x, y)
			img.SetRGBA(x, y, color.RGBA{R: mix(fill.R, under.R), G: mix(fill.G, under.G), B: mix(fill.B, under.B), A: 255})
		}
	}
}
