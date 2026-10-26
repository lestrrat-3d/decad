package export

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// View is an orthographic camera and the size of the image it draws
// (agent-docs/png-export-design.md). Direction is the way the camera looks,
// from the eye into the scene, and Up sets the image's vertical. Neither need
// be a unit vector. The image's right is Direction × Up. Width and Height are
// pixels, each from 16 to 8192.
//
// Outlines draws the body's visible feature edges over the shading, one dark
// pixel wide (agent-docs/png-export-design.md §6). FieldOfView, an angle, makes
// the camera perspective with that field of view across the smaller image side
// of the scene's bounding sphere; the zero units.Value is the orthographic camera
// (agent-docs/png-export-design.md §7). Outlines and a perspective camera are not
// combined.
type View struct {
	Direction, Up r3.Vec
	Width, Height int
	Outlines      bool
	FieldOfView   units.Value
}

// IsometricView looks at the model along the diagonal from above its +X, +Y,
// +Z corner, with +Z up the image, so the model's top, front and right faces
// show. It draws width by height pixels.
func IsometricView(width, height int) View {
	return View{Direction: r3.NewVec(-1, -1, -1), Up: r3.NewVec(0, 0, 1), Width: width, Height: height}
}

// PNG draws body as a PNG image of its tessellation at the requested chord
// tolerance, so a program can look at a part (agent-docs/png-export-design.md).
// The camera is orthographic and the model is centred and scaled to fill the
// image with a margin, keeping its proportions. A face is lit by a headlight on
// the camera and shaded flat by how squarely it faces it: a face turned to the
// camera is light blue, shaded toward 30% as it turns edge on. A face that looks
// away from the camera, which shows where a sheet is open or a mesh is wound
// inside out, is orange. A pixel no face covers is opaque white. No outlines,
// axes or labels are drawn.
//
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. The output is deterministic for a platform: equal body,
// tolerance, view and options give equal bytes. ctx, w and body MUST NOT be nil,
// and view MUST have a direction, an up vector not parallel to it and sides in
// range, each otherwise decad.ErrDegenerate.
func PNG(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, view View, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: PNG: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: PNG: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	return writePNG(ctx, w, mesh, view)
}

// writePNG draws the mesh as a PNG image.
func writePNG(ctx context.Context, w io.Writer, mesh meshSource, view View) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	cam := raster.Camera{Direction: view.Direction, Up: view.Up}
	if view.FieldOfView != (units.Value{}) {
		fov, err := sectionrecord.MagnitudeIn(view.FieldOfView, units.Angle, units.Radian, "the field of view")
		if err != nil {
			return fmt.Errorf("export: PNG: %w", err)
		}
		if fov == 0 {
			return fmt.Errorf("export: PNG: %w: a field of view of zero draws nothing; leave it unset for an orthographic camera", decad.ErrDegenerate)
		}
		if view.Outlines {
			return fmt.Errorf("export: PNG: %w: outlines are not drawn with a perspective camera", decad.ErrDegenerate)
		}
		cam.FieldOfView = fov
	}
	img, err := raster.Render(ctx, mesh.Vertices(), mesh.Triangles(), cam, view.Width, view.Height)
	if errors.Is(err, raster.ErrDegenerate) {
		return fmt.Errorf("export: PNG: %w: %s", decad.ErrDegenerate, err)
	}
	if err != nil {
		return err
	}
	if view.Outlines {
		if err := drawOutlines(ctx, img, mesh, view); err != nil {
			return err
		}
	}
	var encoded bytes.Buffer
	if err := png.Encode(&encoded, img); err != nil {
		return fmt.Errorf("export: PNG encoding failed: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if n, err := w.Write(encoded.Bytes()); err != nil {
		return fmt.Errorf("export: PNG write failed: %w", err)
	} else if n != encoded.Len() {
		return fmt.Errorf("export: PNG write failed: %w", io.ErrShortWrite)
	}
	return nil
}

// outlineColor is the dark grey of an outline.
var outlineColor = color.RGBA{R: 32, G: 32, B: 32, A: 255}

// drawOutlines draws the visible feature edges of mesh over img.
func drawOutlines(ctx context.Context, img *image.RGBA, mesh meshSource, view View) error {
	cam := raster.Camera{Direction: view.Direction, Up: view.Up}
	lines, err := drawing.Project(ctx, mesh.Vertices(), mesh.Triangles(), cam)
	if err != nil {
		return err
	}
	proj, err := raster.NewProjection(mesh.Vertices(), cam, view.Width, view.Height)
	if err != nil {
		return err
	}
	segments := make([][2][2]float64, len(lines.Visible))
	for i, s := range lines.Visible {
		segments[i] = [2][2]float64{s.A, s.B}
	}
	raster.DrawSegments(img, proj, segments, outlineColor)
	return nil
}
