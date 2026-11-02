package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Refine tightens the bound a chain of mesh Unions over extruded cylinders
// publishes. Where two wire spans meet at a bend their walls touch nearly
// tangent, and the Union's held bound there is much coarser than its chords,
// so Tessellate refuses a fine tolerance. Refine proves how far the held mesh
// lies from the true wire and returns a body whose Tessellate accepts it.
func Example_refine() {
	ctx := context.Background()
	doc := decad.New()

	// span extrudes a 0.35 mm wire along +Z and turns it onto the axis axis,
	// starting at start.
	span := func(start, axis r3.Vec, length float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), 0.35), 0.35))
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		wire, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
		if err != nil {
			return nil, err
		}
		f, err := r3.NewFrame(start, r3.Vec{Z: 1}.Cross(axis), axis.Cross(r3.Vec{Z: 1}.Cross(axis)))
		if err != nil {
			return nil, err
		}
		place, err := r3.FromFrame(f)
		if err != nil {
			return nil, err
		}
		return decad.Transform(ctx, wire, place)
	}

	// Two spans meet end to end at a right-angle bend.
	first, err := span(r3.Vec{X: 3, Y: 1, Z: 2}, r3.Vec{X: 1}, 30)
	if err != nil {
		fmt.Printf("failed to build the first span: %s\n", err)
		return
	}
	second, err := span(r3.Vec{X: 33, Y: 1, Z: 2}, r3.Vec{Y: 1}, 20)
	if err != nil {
		fmt.Printf("failed to build the second span: %s\n", err)
		return
	}
	wire, err := decad.Union(ctx, first, second)
	if err != nil {
		fmt.Printf("failed to join the spans: %s\n", err)
		return
	}
	_, err = wire.Tessellate(ctx, units.Millimeters(0.02))
	fmt.Println("0.02 mm before Refine is refused:", errors.Is(err, decad.ErrUnsupported))

	refined, err := decad.Refine(ctx, wire, units.Millimeters(0.02))
	if err != nil {
		fmt.Printf("failed to refine: %s\n", err)
		return
	}
	mesh, err := refined.Tessellate(ctx, units.Millimeters(0.02))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	bound, err := mesh.Bound().In(units.Millimeter)
	if err != nil {
		fmt.Printf("failed to read the bound: %s\n", err)
		return
	}
	fmt.Println("refined bound within 0.02 mm:", bound <= 0.02)
	// Output:
	// 0.02 mm before Refine is refused: true
	// refined bound within 0.02 mm: true
}
