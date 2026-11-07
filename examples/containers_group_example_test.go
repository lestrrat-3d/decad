package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ChildNamed gathers bodies built in a loop into one container, NewUniqueChild
// keeps a second group of the same name apart, and VerifyContainers reports a
// body left in the root and a container that holds nothing.
func Example_containers_group() {
	ctx := context.Background()
	doc := decad.New()

	box := func(c *decad.Container, x float64) error {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return err
		}
		rect := s.CreateRectangle(x, 0, x+1, 1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return err
		}
		if err := c.AddSketch(s); err != nil {
			return err
		}
		_, err = decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		return err
	}

	// Every pass of the loop lands in the same "Planet Gears" container.
	for i := range 3 {
		planets, err := doc.Root().ChildNamed("Planet Gears")
		if err != nil {
			fmt.Printf("failed to get container: %s\n", err)
			return
		}
		if err := box(planets, float64(i)*2); err != nil {
			fmt.Printf("failed to build body: %s\n", err)
			return
		}
	}
	planets, _ := doc.Root().ChildNamed("Planet Gears")
	fmt.Println("planet gears:", len(planets.Bodies()))

	// A second group with the same name gets the next free suffix.
	for range 2 {
		c, err := doc.Root().NewUniqueChild("Spare")
		if err != nil {
			fmt.Printf("failed to add container: %s\n", err)
			return
		}
		fmt.Println("container:", c.Path())
	}

	// A stray body in the root and the empty "Spare" containers are reported.
	if err := box(doc.Root(), 10); err != nil {
		fmt.Printf("failed to build body: %s\n", err)
		return
	}
	rep := doc.VerifyContainers()
	fmt.Println("passed:", rep.Passed())
	fmt.Println(rep.Err())

	// Output:
	// planet gears: 3
	// container: /Spare
	// container: /Spare 2
	// passed: false
	// decad: container layout is not clean: live bodies #4 sit in the root container; containers "/Spare", "/Spare 2" hold no bodies
}
