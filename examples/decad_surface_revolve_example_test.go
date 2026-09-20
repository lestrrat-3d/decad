package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// WithSurfaceResult on Revolve omits the two caps a partial sweep would
// otherwise close with, publishing a sheet instead of a solid
// (docs/surface-design.md §4). A full revolution mints no closing face to
// omit in the first place, so there the option changes no face at all: the
// result is a CLOSED sheet — no free edge — rather than a refusal.
func Example_decad_surfaceRevolve() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// An annular section clear of the axis: u ∈ [0, 10], v ∈ [5, 15].
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own u axis, stated in plane coordinates.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

	doc := decad.New()
	sheet, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	_, freeErr := decad.Edges(decad.Free()).SelectEdges(sheet)
	_, volErr := sheet.Volume()

	fmt.Printf("is sheet: %v\n", sheet.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(sheet.Faces()))
	fmt.Printf("shell open: %v\n", sheet.Shells()[0].IsOpen())
	fmt.Printf("free edges error: %v\n", freeErr)
	fmt.Printf("volume error: %v\n", volErr)
	// Output:
	// is sheet: true
	// faces: 4
	// shell open: false
	// free edges error: decad: selector matched nothing: edges(free) on body 0:"body" matched 0 edges, expected any; the clause free matched none
	// volume error: decad: body is not a solid
}
