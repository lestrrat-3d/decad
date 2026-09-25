package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/stepadapter"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
)

func Example_stepadapter_write() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 2, 3)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	header := step.Header{
		Description:   []string{"faceted solid"},
		Name:          "box.step",
		Timestamp:     time.Date(2026, 9, 25, 0, 0, 0, 0, time.UTC),
		Authors:       []string{"Example"},
		Organizations: []string{"Example"},
	}
	var out bytes.Buffer
	if err := stepadapter.Write(context.Background(), &out, body, header, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to write STEP: %s\n", err)
		return
	}
	fmt.Printf("AP214: %v, planar facets: %d\n",
		strings.Contains(out.String(), "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'))"),
		strings.Count(out.String(), "=ADVANCED_FACE("))
	// Output:
	// AP214: true, planar facets: 12
}
