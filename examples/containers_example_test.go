package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A decad.Container groups the sketches, planes and bodies of one document. It
// moves nothing and instances nothing: a consumer that maps a model onto a tool
// with its own component tree reads the grouping from it. An extrude lands in its
// sketch's container, a boolean in its target's, and an operation that reads two
// containers refuses with decad.ErrCrossContainer until MoveBody puts a finished
// body next to the other.
func Example_containers() {
	ctx := context.Background()
	doc := decad.New()
	driver, err := doc.Root().NewChild("driver")
	if err != nil {
		fmt.Printf("failed to create container: %s\n", err)
		return
	}
	pinion, err := doc.Root().NewChild("pinion")
	if err != nil {
		fmt.Printf("failed to create container: %s\n", err)
		return
	}

	// box extrudes a rectangle from a sketch placed in c.
	box := func(c *decad.Container, x0, y0, x1, y1, h float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		if err := c.AddSketch(s); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	}
	plate, err := box(driver, 0, 0, 4, 4, 2)
	if err != nil {
		fmt.Printf("failed to build plate: %s\n", err)
		return
	}
	pin, err := box(pinion, 1, 1, 2, 2, 2)
	if err != nil {
		fmt.Printf("failed to build pin: %s\n", err)
		return
	}

	// The two bodies sit in different containers, so a cut refuses.
	if _, err := decad.Cut(ctx, plate, pin); err != nil {
		fmt.Println("cut across containers refuses:", err)
	}

	// MoveBody puts the finished pin in the driver container; nothing else changes.
	if err := doc.MoveBody(pin, driver); err != nil {
		fmt.Printf("failed to move: %s\n", err)
		return
	}
	cut, err := decad.Cut(ctx, plate, pin)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	home, _ := doc.ContainerOf(cut)
	fmt.Printf("the cut lands in %q with %d body\n", home.Name(), len(home.Bodies()))

	// Output:
	// cut across containers refuses: decad: operands sit in different containers: "/driver" and "/pinion"
	// the cut lands in "driver" with 1 body
}
