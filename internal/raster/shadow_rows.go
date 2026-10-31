package raster

import (
	"context"
	"image/color"
	"math"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/r3"
)

const minShadowPixels = 256 * 256

// shadowRows reads one completed camera and light depth buffer. Each row owns
// its shaded counts and shadow colours; samples run in order after rows join.
type shadowRows struct {
	verts                     []r3.Vec
	tris                      [][3]int
	tints                     []color.RGBA
	back                      color.RGBA
	proj, lproj               Projection
	toLight                   r3.Vec
	width, height, shadowSize int
	ids                       []int32
	depths, shadowMap         []float64
	shaded                    []uint16
	shadow                    []color.RGBA
}

// Keep the escaping method value in this parallel-only helper, so the serial
// path keeps its row state on the stack.
func shadeParallelRows(ctx context.Context, rows shadowRows, workers int) error {
	_, err := orderedwork.Each(rows.height, workers, 8, ctx.Err, rows.shadeRow)
	return err
}

func (s *shadowRows) shadeRow(y int) error {
	verts, tris, tints, back := s.verts, s.tris, s.tints, s.back
	proj, lproj, toLight := s.proj, s.lproj, s.toLight
	width, height, shadowSize := s.width, s.height, s.shadowSize
	ids, depths, shadowMap := s.ids, s.depths, s.shadowMap
	shaded, shadow := s.shaded, s.shadow
	for x := range width {
		at := y*width + x
		if ids[at] < 0 {
			continue
		}
		t := tris[ids[at]]
		normal, ok := verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Normalize()
		if !ok {
			continue
		}
		// The pixel's world point: the image plane's place of the pixel and its depth.
		px := proj.centreX + (float64(x)+0.5-float64(width)/2)/proj.scale
		py := proj.centreY - (float64(y)+0.5-float64(height)/2)/proj.scale
		world := proj.World(px, py, depths[at])
		view := proj.Sight(world)
		if normal.Dot(view) > 0 {
			normal = normal.Scale(-1) // seen from behind: the side that faces the camera
		}
		cos := normal.Dot(toLight)
		if cos <= 0 {
			continue // turned from the light: already at the ambient level
		}
		lx, ly, ld := lproj.Project(world)
		ix, iy := int(math.Floor(lx)), int(math.Floor(ly))
		if ix < 0 || iy < 0 || ix >= shadowSize || iy >= shadowSize {
			continue
		}
		slope := math.Sqrt(math.Max(0, 1-cos*cos)) / math.Max(cos, 0.1)
		bias := lproj.PixelSize() * (1.5 + 2*slope)
		if ld > shadowMap[iy*shadowSize+ix]+bias {
			shaded[at]++
			if shaded[at] == 1 {
				front := Front
				if tints != nil {
					front = tints[ids[at]]
				}
				base := front
				if cosView := view.Dot(verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]]))); cosView > 0 {
					base = back
				}
				shadow[at] = color.RGBA{
					R: uint8(math.Round(float64(base.R) * ambient)),
					G: uint8(math.Round(float64(base.G) * ambient)),
					B: uint8(math.Round(float64(base.B) * ambient)),
					A: 255,
				}
			}
		}
	}
	return nil
}
