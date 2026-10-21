package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Selectors name faces and edges by what they are, never by index. To pick the
// top of a part, use Facing, which keeps only the face whose outward normal
// points the given way. NormalTo would match both the top and the bottom, since
// it ignores which way the face points. CapEnd names the same face by the
// feature that created it.
func Example_select_faces_by_direction() {
	// Make a plate, which has a top face and a bottom face to choose between.
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
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	up := r3.NewVec(0, 0, 1)

	// NormalTo matches the top and the bottom.
	both, err := decad.Faces(decad.NormalTo(up)).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("NormalTo: %d faces\n", len(both))

	// Facing matches only the top. Exactly(1) fails the call unless exactly
	// one face matches, which catches a query that is wrong.
	top, err := decad.Faces(decad.Planar(), decad.Facing(up)).Exactly(1).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("Facing: %d face\n", len(top))

	// The end cap of the extrude is that same top face.
	cap, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))).Exactly(1).SelectFaces(plate)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("same face as the end cap: %v\n", fmt.Sprint(top[0].Origins()) == fmt.Sprint(cap[0].Origins()))
	// Output:
	// NormalTo: 2 faces
	// Facing: 1 face
	// same face as the end cap: true
}
