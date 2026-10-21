package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Remove takes a body back out of the document. Use it to undo something you
// added, for example a fixture that turned out to collide with the part. The
// removed body can still be measured, but it no longer takes part in Verify.
func Example_remove_body() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The part is the thing we are building: a cube.
	partSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	partRect := partSketch.CreateRectangle(0, 0, 10, 10)
	partSketch.Fix(partRect.A)
	if _, err := partSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	if _, err := decad.Extrude(context.Background(), doc, partSketch, partSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude part: %s\n", err)
		return
	}

	// The fixture is a second cube shifted 5 mm in X, Y and Z, so it overlaps the
	// part's corner. It stands in for a support or jig that collides with the part.
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	fixtureSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	fixtureRect := fixtureSketch.CreateRectangle(5, 5, 15, 15)
	fixtureSketch.Fix(fixtureRect.A)
	if _, err := fixtureSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	fixture, err := decad.Extrude(context.Background(), doc, fixtureSketch, fixtureSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude fixture: %s\n", err)
		return
	}

	// Verify sees the overlap.
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("with fixture: %s\n", report.Status)

	// Take the fixture out and verify again.
	if err := decad.Remove(fixture); err != nil {
		fmt.Printf("failed to remove: %s\n", err)
		return
	}
	report, err = doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("without fixture: %s\n", report.Status)
	// Output:
	// with fixture: Interfering
	// without fixture: Sound
}
