package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"image/color"
	"image/png"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// View.Opacity makes a body translucent: one value from above 0 to 1 for each body drawn. Here a plate floats over a
// block, drawn half clear, so the block shows through it.
func Example_png_opacity() {
	doc := decad.New()
	block, err := opacityBlock(doc, 0, 0, 40, 40, 10)
	if err != nil {
		fmt.Printf("failed to build the block: %s\n", err)
		return
	}
	plate, err := opacityBlock(doc, 15, 15, 25, 25, 2)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	lift, err := r3.Translation(r3.Vec{Z: 11})
	if err != nil {
		fmt.Printf("failed to build the lift: %s\n", err)
		return
	}
	plate, err = decad.Transform(context.Background(), plate, lift)
	if err != nil {
		fmt.Printf("failed to lift the plate: %s\n", err)
		return
	}
	view := export.View{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0), Width: 200, Height: 200,
		Colors:  []color.RGBA{{R: 200}, {G: 200}},
		Opacity: []units.Value{units.Scalar(1), units.Scalar(0.5)}}
	var out bytes.Buffer
	if err := export.PNGBodies(context.Background(), &out, []*decad.Body{block, plate}, units.Millimeters(0.1), view); err != nil {
		fmt.Printf("failed to draw: %s\n", err)
		return
	}
	img, err := png.Decode(&out)
	if err != nil {
		fmt.Printf("failed to decode: %s\n", err)
		return
	}
	r, g, _, _ := img.At(100, 100).RGBA()
	fmt.Printf("over the plate both the block's red and the plate's green show: %t\n", r>>8 > 0 && g>>8 > 0)
	// Output:
	// over the plate both the block's red and the plate's green show: true
}

// opacityBlock extrudes the rectangle from (x0, y0) to (x1, y1) up by height millimetres.
func opacityBlock(doc *decad.Document, x0, y0, x1, y1, height float64) (*decad.Body, error) {
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
