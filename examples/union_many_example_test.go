package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_unionMany() {
	ctx := context.Background()
	doc := decad.New()
	box := func(x0, y0, x1, y1 float64) (*decad.Body, error) {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		profiles := s.Profiles()
		if len(profiles) != 1 {
			return nil, fmt.Errorf("rectangle has %d profiles", len(profiles))
		}
		return decad.Extrude(ctx, doc, s, profiles[0],
			decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	}
	var bodies []*decad.Body
	for _, r := range [][4]float64{{0, 0, 4, 4}, {2, 2, 6, 6}, {5, -1, 9, 3}} {
		body, err := box(r[0], r[1], r[2], r[3])
		if err != nil {
			fmt.Printf("failed to build box: %s\n", err)
			return
		}
		bodies = append(bodies, body)
	}
	joined, err := decad.Union(ctx, bodies[0], bodies[1:]...)
	if err != nil {
		fmt.Printf("failed to join boxes: %s\n", err)
		return
	}
	volume, err := joined.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	cubicMM, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	fmt.Printf("bodies=%d volume=%.0f mm3\n", len(doc.Bodies()), cubicMM)
	// Output: bodies=1 volume=86 mm3
}
