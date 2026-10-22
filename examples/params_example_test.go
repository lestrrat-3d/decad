package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A document reads feature values from a parameter table, so a value can be
// derived from other parameters. Editing the table and running the builder
// again rebuilds the model; Bindings lists what each run read.
func Example_params() {
	build := func(width string) (*decad.Document, units.Value, error) {
		// Sharing the sketch World's table puts sketch dimensions and feature
		// values on the same definitions.
		w := sketch.NewWorld()
		doc := decad.New(decad.WithParams(w.Params()))
		t := doc.Params()
		for name, expr := range map[string]string{"width": width, "depth": "60"} {
			if err := t.SetExpr(name, expr, units.Millimeter); err != nil {
				return nil, units.Value{}, err
			}
		}
		if err := t.SetExpr("thickness", "width / 10", units.Millimeter); err != nil {
			return nil, units.Value{}, err
		}

		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, units.Value{}, err
		}
		wd, err := doc.Param("width")
		if err != nil {
			return nil, units.Value{}, err
		}
		dp, err := doc.Param("depth")
		if err != nil {
			return nil, units.Value{}, err
		}
		rect := s.CreateRectangle(0, 0, wd.Base(), dp.Base())
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, units.Value{}, err
		}
		th, err := doc.Param("thickness")
		if err != nil {
			return nil, units.Value{}, err
		}
		body, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: th, Dir: decad.Along})
		if err != nil {
			return nil, units.Value{}, err
		}
		volume, err := body.Volume()
		return doc, volume.Value, err
	}

	for _, width := range []string{"100", "200"} {
		doc, volume, err := build(width)
		if err != nil {
			fmt.Printf("failed to build: %s\n", err)
			return
		}
		fmt.Printf("width %s: volume %s\n", width, volume)
		for _, b := range doc.Bindings() {
			fmt.Printf("  %s = %s -> %s\n", b.Name, b.Expr, b.Value)
		}
	}
	// Output:
	// width 100: volume 60000 mm^3
	//   width = 100 -> 100 mm
	//   depth = 60 -> 60 mm
	//   thickness = width / 10 -> 10 mm
	// width 200: volume 240000 mm^3
	//   width = 200 -> 200 mm
	//   depth = 60 -> 60 mm
	//   thickness = width / 10 -> 20 mm
}
