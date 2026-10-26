package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Boss raises a round post on a flat face, with a blind hole down its middle
// for a screw to bite into. Use it for the screw posts of a housing instead of
// extruding, joining and drilling by hand.
func Example_boss() {
	// Make a 40 x 40 x 5 mm plate: sketch a rectangle and extrude it (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// An 8 mm post, 8 mm tall, in the middle of the top face (the extrude's end
	// cap, at z = 5), with a 3 mm pilot hole 6 mm deep from the post's top.
	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 20, Z: 5},
		Diameter: units.Millimeters(8),
		Height:   units.Millimeters(8),
		Bore:     &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(6)},
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 8000 mm^3. The post adds 16π x 8 and the pilot removes
	// 2.25π x 6.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8360 mm^3
}

// A BossBore can carry a Head and a Thread, as a hole does. Here the post's
// pilot is a 3 mm tap drill with a counterbore at its mouth and an M3.5 x 0.5
// thread cut 4 mm into it below the recess, so a screw threads in under a collar.
func Example_boss_tapped() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 20, Z: 5},
		Diameter: units.Millimeters(8),
		Height:   units.Millimeters(8),
		Bore: &decad.BossBore{
			Diameter: units.Millimeters(3),
			Depth:    units.Millimeters(6),
			Head:     decad.Counterbore{Diameter: units.Millimeters(6), Depth: units.Millimeters(2)},
			Thread:   &decad.HoleThread{Pitch: units.Millimeters(0.5), Length: units.Millimeters(4)},
		},
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8311 mm^3
}

// Draft leans the boss's wall so a moulded or cast post releases from its tool.
// A positive angle makes the post narrower toward its top. Here a post 8 mm wide
// at the plate and 8 mm tall, drafted 3 degrees, is about 7.2 mm across at its top.
func Example_boss_draft() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 20, Z: 5},
		Diameter: units.Millimeters(8),
		Height:   units.Millimeters(8),
		Draft:    units.Degrees(3),
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 8000 mm^3, and the post is a frustum of radii 4 and 3.58 mm.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8361 mm^3
}

// RootFillet rounds the sharp corner where the post meets the plate, which
// spreads the load a post carries and eases a cast part's corner. Here a post 8
// mm wide and 8 mm tall gets a 1 mm fillet at its foot.
func Example_boss_root_fillet() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:       decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:         r3.Vec{X: 20, Y: 20, Z: 5},
		Diameter:   units.Millimeters(8),
		Height:     units.Millimeters(8),
		RootFillet: units.Millimeters(1),
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate and post hold 8402 mm^3, and the fillet adds about 7 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8408 mm^3
}

// Shape gives the boss a rectangular, slot or polygonal section instead of a
// round one, reusing the shapes a Hole takes. Here a hexagonal post 8 mm across
// flats and 6 mm tall stands on a plate, as the seat for a hex nut.
func Example_boss_shape() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:   decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:     r3.Vec{X: 20, Y: 20, Z: 5},
		Height: units.Millimeters(6),
		Shape:  decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(8), Angle: units.Degrees(0)},
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 8000 mm^3 and a hexagon 8 mm across flats, 55.4 mm^2, adds 332.6 mm^3 over 6 mm.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8333 mm^3
}
