package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"image/png"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
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
