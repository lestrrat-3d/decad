package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithTaper drafts an extrude: a positive angle leans every wall inward as it
// leaves the sketch plane, so a molded part releases along the sweep. The far
// cap is the profile offset by h·tan α, its corners mitered, and every wall
// stays an exact surface: a Plane leaning by the taper for each straight edge.
// The tangent of the angle is only ever enclosed, so the drafted body's
// measurements are approximate, each with the bound it carries.
func Example_decad_extrude_taper() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithTaper(units.Degrees(5)))
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The closed form h(a² − 2ad + 4d²/3) with d = h·tan 5°.
	d := 10 * math.Tan(5*math.Pi/180)
	want := 10 * (400 - 40*d + 4*d*d/3)
	fmt.Printf("faces: %d\n", len(body.Faces()))
	fmt.Printf("volume: %.3f mm^3 (%s)\n", vol.Value.Base(), vol.Exactness)
	fmt.Printf("closed form: %.3f mm^3\n", want)

	walls, err := decad.Faces(decad.Planar()).SelectFaces(body)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	for _, f := range walls {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		if err != nil {
			fmt.Printf("failed to read a normal: %s\n", err)
			return
		}
		// A cap faces straight along the sweep; a wall leans off it.
		if math.Abs(n.Value.Z) < 0.5 {
			fmt.Printf("wall leans %.2f degrees\n", math.Asin(n.Value.Z)*180/math.Pi)
		}
	}
	// Output:
	// faces: 6
	// volume: 3660.251 mm^3 (Approximate)
	// closed form: 3660.251 mm^3
	// wall leans 5.00 degrees
	// wall leans 5.00 degrees
	// wall leans 5.00 degrees
	// wall leans 5.00 degrees
}

// A tapered extrude can stop at a planar face of another live body. The
// selected face supplies the far level; the sketch remains the near level.
func Example_decad_extrude_taper_to_face() {
	w := sketch.NewWorld()
	stopFrame, err := r3.NewFrame(r3.NewVec(0, 0, 30), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		fmt.Printf("failed to make stop frame: %s\n", err)
		return
	}
	stopPlane, err := w.CreatePlaneFromFrame(stopFrame)
	if err != nil {
		fmt.Printf("failed to make stop plane: %s\n", err)
		return
	}
	stopSketch, err := w.CreateSketch(stopPlane)
	if err != nil {
		fmt.Printf("failed to make stop sketch: %s\n", err)
		return
	}
	stopRect := stopSketch.CreateRectangle(-30, -30, 30, 30)
	stopSketch.Fix(stopRect.A)
	if _, err := stopSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve stop sketch: %s\n", err)
		return
	}
	doc := decad.New()
	stop, err := doc.Extrude(stopSketch, stopSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to build stop: %s\n", err)
		return
	}
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to make part sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve part sketch: %s\n", err)
		return
	}
	end := decad.ToFace{Body: stop, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(stop)))}
	part, err := doc.Extrude(s, s.Profiles()[0], end, decad.WithTaper(units.Degrees(5)))
	if err != nil {
		fmt.Printf("failed to build tapered part: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\nvolume: %.0f mm^3\n", len(part.Faces()), volume.Value.Base())
	// Output:
	// faces: 6
	// volume: 9126 mm^3
}
