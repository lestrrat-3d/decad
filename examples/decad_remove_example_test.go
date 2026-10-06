package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Document.Remove takes a body back out of the model. A caller that adds a
// body to someone else's document — a support, a fixture — and then sees
// Verify fail can undo its own addition. The removed body is retired: it
// leaves Bodies() and Verify, still answers its measurements, and no
// operation takes it.
func Example_decad_remove() {
	buildCube := func(doc *decad.Document, x, y, z float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(0, 0, 10, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		cube, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		if err != nil {
			return nil, err
		}
		shift, err := r3.Translation(r3.NewVec(x, y, z))
		if err != nil {
			return nil, err
		}
		return cube.Placed(context.Background(), shift)
	}

	doc := decad.New()
	if _, err := buildCube(doc, 0, 0, 0); err != nil {
		fmt.Printf("failed to build part: %s\n", err)
		return
	}
	// A second body that turns out to overlap the part by a 5 mm corner block.
	added, err := buildCube(doc, 5, 5, 5)
	if err != nil {
		fmt.Printf("failed to build addition: %s\n", err)
		return
	}

	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("with addition: %s, passed: %v\n", report.Status, report.Passed())

	if err := doc.Remove(added); err != nil {
		fmt.Printf("failed to remove: %s\n", err)
		return
	}
	report, err = doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("after remove: %s, passed: %v, live bodies: %d\n", report.Status, report.Passed(), len(doc.Bodies()))

	// The removed body still reads, but it is out of the model for good.
	vol, err := added.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	fmt.Printf("removed body volume: %s\n", vol.Value)
	fmt.Printf("remove again refused as retired: %v\n", errors.Is(doc.Remove(added), decad.ErrRetiredBody))
	// Output:
	// with addition: Interfering, passed: false
	// after remove: Sound, passed: true, live bodies: 1
	// removed body volume: 1000 mm^3
	// remove again refused as retired: true
}
