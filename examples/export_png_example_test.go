package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// export.PNG draws a body as a picture, so a program or a person can look at a
// part without a CAD viewer. The camera is orthographic: it looks along a
// direction you give, and the model is centred and scaled to fill the image.
func Example_export_png() {
	// Make a body to draw: sketch a rectangle and extrude it into a box (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Draw the box from the isometric corner, 320 by 240 pixels. Write to any
	// io.Writer: a file, a network connection or, here, memory.
	var out bytes.Buffer
	if err := export.PNG(context.Background(), &out, box, units.Millimeters(0.1), export.IsometricView(320, 240)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	bounds := img.Bounds()
	fmt.Printf("image: %d x %d pixels\n", bounds.Dx(), bounds.Dy())
	// A corner of the image lies outside the box's outline and stays white.
	r, g, b, _ := img.At(0, 0).RGBA()
	fmt.Printf("corner is white: %v\n", r == 0xffff && g == 0xffff && b == 0xffff)
	// Output:
	// image: 320 x 240 pixels
	// corner is white: true
}

// export.PNGBodies draws several bodies in one picture, such as the parts of an
// assembly laid out side by side. The scene is centred and scaled as a whole.
func Example_export_png_bodies() {
	doc := decad.New()
	var parts []*decad.Body
	for _, x := range []float64{0, 30} {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(x, 0, x+10, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		part, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		parts = append(parts, part)
	}

	var out bytes.Buffer
	if err := export.PNGBodies(context.Background(), &out, parts, units.Millimeters(0.1), export.IsometricView(320, 240)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	bounds := img.Bounds()
	fmt.Printf("image: %d x %d pixels\n", bounds.Dx(), bounds.Dy())
	// Output:
	// image: 320 x 240 pixels
}

// View.Colors gives each body its own colour, so the parts of an assembly read
// apart. Here two blocks are drawn red and green from above, and the pixel over
// each is checked.
func Example_export_png_colors() {
	doc := decad.New()
	var parts []*decad.Body
	for _, x := range []float64{0, 30} {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(x, 0, x+10, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		part, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		parts = append(parts, part)
	}
	view := export.View{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Width: 400, Height: 400}
	view.Colors = []color.RGBA{{R: 200, G: 40, B: 40}, {R: 40, G: 200, B: 40}}
	var out bytes.Buffer
	if err := export.PNGBodies(context.Background(), &out, parts, units.Millimeters(0.1), view); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	left := color.RGBAModel.Convert(img.At(50, 200)).(color.RGBA)
	right := color.RGBAModel.Convert(img.At(340, 200)).(color.RGBA)
	fmt.Printf("left block is red: %t, right block is green: %t\n", left.R > left.G, right.G > right.R)
	// Output:
	// left block is red: true, right block is green: true
}

// Set Outlines on the view to draw the part's visible edges in dark grey over
// the shading, so the shape reads even where two faces share a shade.
func Example_export_png_outlines() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Look straight down at the box, 200 by 200 pixels, with outlines on. The
	// top face spans pixels 10 to 190, so its left edge lies in column 10.
	view := export.View{
		Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0),
		Width: 200, Height: 200, Outlines: true,
	}
	var out bytes.Buffer
	if err := export.PNG(context.Background(), &out, box, units.Millimeters(0.1), view); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	r, g, b, _ := img.At(10, 100).RGBA()
	fmt.Printf("edge pixel: %d %d %d\n", r>>8, g>>8, b>>8)
	// Output:
	// edge pixel: 32 32 32
}

// Set FieldOfView on the view for a perspective camera, which makes nearer parts
// look larger as an eye or a lens sees them. The zero value is the orthographic
// camera of the other examples. Here two equal 4 mm blocks side by side, one 10 mm
// tall and one 2 mm, are drawn from above: the taller block's top is nearer the
// camera and takes more of the picture.
func Example_export_png_perspective() {
	doc := decad.New()
	var blocks []*decad.Body
	for _, spec := range []struct{ x, height float64 }{{-6, 10}, {2, 2}} {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(spec.x, -2, spec.x+4, 2)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(spec.height), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		blocks = append(blocks, block)
	}

	view := export.View{
		Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0),
		Width: 200, Height: 200, FieldOfView: units.Degrees(60),
	}
	var out bytes.Buffer
	if err := export.PNGBodies(context.Background(), &out, blocks, units.Millimeters(0.1), view); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	// Count the columns across the middle row that each block covers.
	count := func(from, to int) (n int) {
		for x := from; x < to; x++ {
			if r, g, b, _ := img.At(x, 100).RGBA(); r != 0xffff || g != 0xffff || b != 0xffff {
				n++
			}
		}
		return n
	}
	fmt.Printf("the nearer block is wider: %t\n", count(0, 100) > count(100, 200))
	// Output:
	// the nearer block is wider: true
}

// HiddenEdges draws the edges behind the part's surfaces as grey dashes, so the
// far side shows through the shading as in an engineering view. Here a cube's
// three hidden edges meet at the far corner, which the isometric camera draws at
// the middle of the image.
func Example_export_png_hidden_edges() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	cube, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	view := export.IsometricView(300, 300)
	view.HiddenEdges = true
	var out bytes.Buffer
	if err := export.PNG(context.Background(), &out, cube, units.Millimeters(0.1), view); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	grey := 0
	for y := 100; y < 200; y++ {
		for x := 100; x < 200; x++ {
			if c := color.RGBAModel.Convert(img.At(x, y)).(color.RGBA); c.R == 150 && c.G == 150 && c.B == 150 {
				grey++
			}
		}
	}
	fmt.Printf("dashed hidden edges in the middle: %t\n", grey > 60)
	// Output:
	// dashed hidden edges in the middle: true
}

// Shadows casts the key light's shadows, so a tall part visibly shades the plate
// beside it. Here a 20 mm pillar stands on a flat plate, seen from above, and the
// image is drawn with and without shadows.
func Example_export_png_shadows() {
	doc := decad.New()
	block := func(x0, y0, x1, y1, height float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	plate, err := block(0, 0, 60, 40, 2)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	pillar, err := block(25, 15, 35, 25, 20)
	if err != nil {
		fmt.Printf("failed to build the pillar: %s\n", err)
		return
	}
	draw := func(shadows bool) (image.Image, error) {
		view := export.View{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Width: 400, Height: 400, Shadows: shadows}
		var out bytes.Buffer
		if err := export.PNGBodies(context.Background(), &out, []*decad.Body{plate, pillar}, units.Millimeters(0.1), view); err != nil {
			return nil, err
		}
		return png.Decode(&out)
	}
	plain, err := draw(false)
	if err != nil {
		fmt.Printf("failed to draw: %s\n", err)
		return
	}
	shaded, err := draw(true)
	if err != nil {
		fmt.Printf("failed to draw: %s\n", err)
		return
	}
	darker := 0
	for y := range 400 {
		for x := range 400 {
			a, b := color.RGBAModel.Convert(plain.At(x, y)).(color.RGBA), color.RGBAModel.Convert(shaded.At(x, y)).(color.RGBA)
			if int(b.R)+int(b.G)+int(b.B) < int(a.R)+int(a.G)+int(a.B) {
				darker++
			}
		}
	}
	fmt.Printf("shadow pixels: %t\n", darker > 200)
	// Output:
	// shadow pixels: true
}

// BackColor sets the colour of the faces that look away from the camera. They are
// orange by default, which is how an open shell or an inside-out mesh shows; pick
// another colour to match a palette or to stand out against the part's own.
func Example_export_png_back_color() {
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(`v 0 0 0
v 10 0 0
v 10 10 0
v 0 10 0
v 0 0 10
v 10 0 10
v 10 10 10
v 0 10 10
f 1 2 3
f 1 3 4
f 5 7 6
f 5 8 7
f 1 6 2
f 1 5 6
f 2 7 3
f 2 6 7
f 3 8 4
f 3 7 8
f 4 5 1
f 4 8 5
`))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	// The cube's triangles wind clockwise seen from outside, so every face the camera sees looks away.
	view := export.View{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Width: 100, Height: 100}
	view.BackColor = &color.RGBA{R: 30, G: 200, B: 60}
	var out bytes.Buffer
	if err := export.MeshPNG(context.Background(), &out, mesh, view); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to read the image: %s\n", err)
		return
	}
	c := color.RGBAModel.Convert(img.At(50, 50)).(color.RGBA)
	fmt.Printf("green dominates: %t\n", c.G > c.R && c.G > c.B)
	// Output:
	// green dominates: true
}
