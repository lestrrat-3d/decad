package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A screw thread is made by cutting a helical groove. Coil builds the groove
// as a helical solid, and Cut removes it from the shaft. Use this recipe to
// model a threaded rod or bolt. The same two steps cut an internal thread into
// a bore, with the groove drawn on the other side of the bore wall.
func Example_thread_cut() {
	w := sketch.NewWorld()
	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

	// The shaft is a plain rod, 5 mm in radius and 20 mm long, made by
	// revolving a rectangle about the V axis.
	shaftSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	half := shaftSketch.CreateRectangle(0, 0, 5, 20)
	shaftSketch.Fix(half.A)
	shaftSketch.Fix(half.C)
	if _, err := shaftSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	shaft, err := decad.Revolve(context.Background(), doc, shaftSketch, shaftSketch.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve shaft: %s\n", err)
		return
	}

	// The groove's cross-section is a V with a 60 degree opening, the usual
	// thread shape. Its point is 0.9 mm below the shaft surface (radius 4.1)
	// and its mouth opens 0.3 mm past the surface (radius 5.3). The half-width
	// at the mouth is 1.2 x tan(30 degrees) = 0.6928 mm. The groove is
	// centered 3 mm up the shaft.
	grooveSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	const halfWidth = 0.6928203230275509
	corners := [][2]float64{{4.1, 3}, {5.3, 3 - halfWidth}, {5.3, 3 + halfWidth}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = grooveSketch.CreatePoint(c[0], c[1])
		grooveSketch.Fix(points[i])
	}
	for i := range points {
		grooveSketch.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := grooveSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Coil the groove one turn at a 1.5 mm pitch. That makes a single turn
	// of thread; use more turns for a longer thread.
	groove, err := decad.Coil(context.Background(), doc, grooveSketch, grooveSketch.Profiles()[0], axis,
		units.Millimeters(1.5), units.Scalar(1))
	if err != nil {
		fmt.Printf("failed to coil groove: %s\n", err)
		return
	}

	// Cut the groove out of the shaft.
	threaded, err := decad.Cut(context.Background(), shaft, groove)
	if err != nil {
		fmt.Printf("failed to cut thread: %s\n", err)
		return
	}
	volume, err := threaded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plain shaft held π x 5² x 20 = 1570.8 mm^3. The thread removes
	// about 14 mm^3 of it.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1557 mm^3
}
