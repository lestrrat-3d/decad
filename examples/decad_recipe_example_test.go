package examples_test

import (
	"context"
	"encoding/json"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A Recipe is the model as a serializable record of intent — no pointer into
// the document, every step a value. encoding/json round-trips it through the
// sealed codecs, so it can be stored, diffed or translated into CAD code. A
// decoded recipe is for inspection and translation today: decad has no replay
// entry point, so a caller reads the steps back rather than handing the value
// to the package.
func Example_decad_recipe() {
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

	// Two steps: extrude the plate, then place a copy 200 mm along x.
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	motion, err := r3.Translation(r3.NewVec(200, 0, 0))
	if err != nil {
		fmt.Printf("failed to build motion: %s\n", err)
		return
	}
	if _, err := body.Placed(motion); err != nil {
		fmt.Printf("failed to place: %s\n", err)
		return
	}

	// Marshal the recipe and decode it back into a fresh decad.Recipe.
	data, err := json.Marshal(doc.Recipe())
	if err != nil {
		fmt.Printf("failed to marshal recipe: %s\n", err)
		return
	}
	var decoded decad.Recipe
	if err := json.Unmarshal(data, &decoded); err != nil {
		fmt.Printf("failed to unmarshal recipe: %s\n", err)
		return
	}

	// Inspect the decoded value: the ops, the extrude distance recovered from
	// its sealed extent, and the dependency the placement recorded.
	fmt.Printf("steps: %d\n", len(decoded.Steps))
	for i, step := range decoded.Steps {
		fmt.Printf("step %d: %s, depends on %v\n", i, step.Op, step.Inputs)
	}
	if dist, ok := decoded.Steps[0].Extent.(decad.Distance); ok {
		fmt.Printf("extrude distance: %s %s\n", dist.D, dist.Dir)
	}
	// Output:
	// steps: 2
	// step 0: extrude, depends on []
	// step 1: placed, depends on [0]
	// extrude distance: 10 mm Along
}
