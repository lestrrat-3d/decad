package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithNoOpenings makes Shell hollow a body without removing any face. The
// result is a sealed hollow part with a void inside, such as a lightweight
// printed block.
func Example_shell_closed() {
	// A solid 100 x 60 x 20 mm block.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// A nil selector names no faces to open. The 5 mm walls enclose a cavity.
	sealed, err := decad.Shell(context.Background(), block, nil, units.Millimeters(5), decad.WithNoOpenings())
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	for i, shell := range sealed.Shells() {
		fmt.Printf("shell %d: %d faces, void: %v\n", i, len(shell.Faces()), shell.IsVoid())
	}
	// Output:
	// shell 0: 6 faces, void: false
	// shell 1: 6 faces, void: true
}
