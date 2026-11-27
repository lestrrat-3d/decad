package examples_test

import (
	"bytes"
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/bodywire"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_bodywire_roundTrip() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	if _, err = s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	body, err := decad.Extrude(decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along}).Do(ctx)
	if err != nil {
		fmt.Printf("failed to build body: %s\n", err)
		return
	}
	var frame bytes.Buffer
	if err = bodywire.Write(ctx, &frame, body, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to encode body: %s\n", err)
		return
	}
	received, err := bodywire.Read(ctx, &frame)
	if err != nil {
		fmt.Printf("failed to decode body: %s\n", err)
		return
	}
	mesh, err := received.Tessellate(ctx, units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to tessellate body: %s\n", err)
		return
	}
	fmt.Printf("received closed mesh: %t\n", mesh.Report().Closed)
	// Output: received closed mesh: true
}
