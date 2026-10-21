package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_section() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	body, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(5), Dir: decad.Along,
	})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	offset, err := w.CreateOffsetPlane(w.XY(), 2.5)
	if err != nil {
		fmt.Printf("failed to create section plane: %s\n", err)
		return
	}
	frame, err := offset.Frame()
	if err != nil {
		fmt.Printf("failed to read section plane: %s\n", err)
		return
	}
	section, profiles, err := body.Section(context.Background(), frame)
	if err != nil {
		fmt.Printf("failed to section: %s\n", err)
		return
	}
	measured, err := decad.MeasureProfile(section, profiles[0])
	if err != nil {
		fmt.Printf("failed to measure section: %s\n", err)
		return
	}
	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	fmt.Printf("profiles: %d, area: %s\n", len(profiles), area.Value)
	// Output:
	// profiles: 1, area: 100 mm^2
}
