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
// pixels, each from 16 to 8192. Colors gives each body its own colour
// (agent-docs/png-export-design.md §8).
//
// Outlines draws the body's visible feature edges over the shading, one dark
// pixel wide (agent-docs/png-export-design.md §6). FieldOfView, an angle, makes
// the camera perspective with that field of view across the smaller image side
// of the scene's bounding sphere; the zero units.Value is the orthographic camera
// (agent-docs/png-export-design.md §7). Outlines, HiddenEdges and Shadows take the perspective
// camera too (agent-docs/png-export-design.md §12).
type View struct {
	Direction, Up r3.Vec
	Width, Height int
	Outlines      bool
	FieldOfView   units.Value
	// Colors, when not empty, are the colours of the bodies' faces that look
	// toward the camera, one for each body drawn, in order: [PNG] and [MeshPNG]
	// draw one body and so take one, and [PNGBodies] takes one for each body.
	// The alpha channel is ignored. Faces that look away stay orange.
	Colors []color.RGBA
	// HiddenEdges draws the feature edges behind surfaces as dashed light grey
	// lines under the outlines (agent-docs/png-export-design.md §9).
	HiddenEdges bool
	// Shadows casts the key light's shadows: a face a nearer one hides from the
	// light is drawn at the ambient level (agent-docs/png-export-design.md §10).
	Shadows bool
	// BackColor, when set, is the colour of the faces that look away from the
	// camera, in place of orange: they show where a sheet is open or a mesh is
	// wound inside out, and a part's far side through an open shell. Its alpha is
	// ignored (agent-docs/png-export-design.md §11).
	BackColor *color.RGBA
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
	return writePNG(ctx, w, mesh, view, []int{len(mesh.Triangles())})
}

// bodyTints is the front colour of each triangle for bodies whose triangle
// counts are counts, or nil when the view sets no colours: view.Colors is one
// colour for each body.
func bodyTints(view View, counts []int) ([]color.RGBA, error) {
	if len(view.Colors) == 0 {
		return nil, nil
	}
	if len(view.Colors) != len(counts) {
		return nil, fmt.Errorf("export: PNG: %w: %d colours for %d bodies; give one for each body or none",
			decad.ErrDegenerate, len(view.Colors), len(counts))
	}
	var tints []color.RGBA
	for i, n := range counts {
		c := view.Colors[i]
		c.A = 255
		for range n {
			tints = append(tints, c)
		}
	}
	return tints, nil
}

// writePNG draws the mesh as a PNG image. counts are the triangle counts of
// the bodies the mesh is made of, in order, which View.Colors is read against.
func writePNG(ctx context.Context, w io.Writer, mesh meshSource, view View, counts []int) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	tints, err := bodyTints(view, counts)
	if err != nil {
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
		cam.FieldOfView = fov
	}
	var img *image.RGBA
	back := raster.Back
	if view.BackColor != nil {
		back = *view.BackColor
		back.A = 255
	}
	if view.Shadows {
		img, err = raster.RenderShadowed(ctx, mesh.Vertices(), mesh.Triangles(), tints, back, cam, view.Width, view.Height,
			min(max(2*max(view.Width, view.Height), 512), 4096))
	} else {
		img, err = raster.RenderBacked(ctx, mesh.Vertices(), mesh.Triangles(), tints, back, cam, view.Width, view.Height)
	}
	if errors.Is(err, raster.ErrDegenerate) {
		return fmt.Errorf("export: PNG: %w: %s", decad.ErrDegenerate, err)
	}
	if err != nil {
		return err
	}
	if view.Outlines || view.HiddenEdges {
		if err := drawOutlines(ctx, img, mesh, view, cam.FieldOfView); err != nil {
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

// outlineColor is the dark grey of an outline, and hiddenColor the light grey of
// a hidden edge.
var (
	outlineColor = color.RGBA{R: 32, G: 32, B: 32, A: 255}
	hiddenColor  = color.RGBA{R: 150, G: 150, B: 150, A: 255}
)

// hiddenDash is the dash of a hidden edge in pixels: 4 drawn, 3 skipped.
const (
	hiddenOn  = 4
	hiddenOff = 3
)

// drawOutlines draws the visible feature edges of mesh over img when the view
// asks for outlines, and the hidden ones, dashed and under them, when it asks
// for hidden edges.
func drawOutlines(ctx context.Context, img *image.RGBA, mesh meshSource, view View, fieldOfView float64) error {
	cam := raster.Camera{Direction: view.Direction, Up: view.Up, FieldOfView: fieldOfView}
	lines, err := drawing.Project(ctx, mesh.Vertices(), mesh.Triangles(), cam)
	if err != nil {
		return err
	}
	proj, err := raster.NewProjection(mesh.Vertices(), cam, view.Width, view.Height)
	if err != nil {
		return err
	}
	plane := func(in []drawing.Segment) [][2][2]float64 {
		out := make([][2][2]float64, len(in))
		for i, s := range in {
			out[i] = [2][2]float64{s.A, s.B}
		}
		return out
	}
	if view.HiddenEdges {
		raster.DrawDashed(img, proj, plane(lines.Hidden), hiddenColor, hiddenOn, hiddenOff)
	}
	if view.Outlines {
		raster.DrawSegments(img, proj, plane(lines.Visible), outlineColor)
	}
	return nil
}
