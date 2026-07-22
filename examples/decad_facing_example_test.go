package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Facing names the ONE cap a caller means. NormalTo(z) matches a slab's two
// parallel caps — the right answer when both are wanted, the wrong one for "the
// top". Facing(z) keeps only the face whose outward, material-leaving normal
// points along z. The typed CapStart / CapEnd helpers then name the same face
// through its provenance role without a "capStart" string literal a typo could
// break.
func Example_decad_facing() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 8 mm along +z: caps at z=0 and z=8.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	z := r3.NewVec(0, 0, 1)

	// NormalTo matches either sense: both caps.
	both, err := decad.Faces(decad.NormalTo(z)).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select caps: %s\n", err)
		return
	}
	fmt.Printf("NormalTo(z) caps: %d\n", len(both))

	// Facing keeps the single upward-facing cap.
	top, err := decad.Faces(decad.Planar(), decad.Facing(z)).Exactly(1).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select the top cap: %s\n", err)
		return
	}
	n, err := top[0].NormalAt(r3.NewVec(0, 0, 0))
	if err != nil {
		fmt.Printf("failed to read the cap normal: %s\n", err)
		return
	}
	fmt.Printf("Facing(z) caps: %d, outward normal (%g, %g, %g)\n", len(top), n.Value.X, n.Value.Y, n.Value.Z)

	// The typed cap role names the same face — no string literal.
	byRole, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))).Exactly(1).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select the end cap: %s\n", err)
		return
	}
	fmt.Printf("Facing(z) and CapEnd name the same face: %t\n",
		fmt.Sprint(top[0].Origins()) == fmt.Sprint(byRole[0].Origins()))
	// Output:
	// NormalTo(z) caps: 2
	// Facing(z) caps: 1, outward normal (0, 0, 1)
	// Facing(z) and CapEnd name the same face: true
}
