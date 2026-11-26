package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// RunSteps builds independent parts of one model at the same time. Each step
// gets its own context and must make every call on the document through it.
// A step sees the bodies that existed when RunSteps began and its own, never
// another step's, so the model comes out the same for every worker count. When
// RunSteps returns, the document lists the earlier bodies first and then each
// step's bodies in step order.
func Example_steps_run() {
	ctx := context.Background()
	doc := decad.New()

	// block extrudes a rectangle on the XY plane. A step calls it with its own ctx.
	block := func(ctx context.Context, x0, y0, x1, y1, height float64) (*decad.Body, error) {
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
		return decad.Extrude(doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along}).Do(ctx)
	}

	// A base plate built before the steps. A step may copy it, but not consume it.
	plate, err := block(ctx, 0, 0, 40, 40, 4)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}

	boss := func(x float64) func(ctx context.Context) error {
		return func(ctx context.Context) error {
			if _, err := block(ctx, x, 0, x+10, 10, 8); err != nil {
				return err
			}
			_, err := decad.Copy(plate).Do(ctx)
			return err
		}
	}
	err = decad.RunSteps(ctx, doc, []decad.Step{
		{Name: "boss at 100", Build: boss(100)},
		{Name: "boss at 200", Build: boss(200)},
	}, decad.WithStepWorkers(2))
	if err != nil {
		fmt.Printf("failed to run the steps: %s\n", err)
		return
	}

	for i, b := range doc.Bodies() {
		vol, err := b.Volume()
		if err != nil {
			fmt.Printf("failed to measure body %d: %s\n", i, err)
			return
		}
		fmt.Printf("body %d: %s\n", i, vol.Value)
	}
	// Output:
	// body 0: 6400 mm^3
	// body 1: 800 mm^3
	// body 2: 6400 mm^3
	// body 3: 800 mm^3
	// body 4: 6400 mm^3
}
