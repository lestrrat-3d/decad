package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Body.Patch fills the two circular free rims of a surface-extruded tube.
// Its mesh shares the wall's chord stations with both new planar faces.
func Example_bodyPatchCircularMesh() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, 10)
	s.Fix(center)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	tube, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Println(err)
		return
	}
	filled, err := tube.Patch(context.Background(), decad.Edges(decad.Free()).Exactly(2))
	if err != nil {
		fmt.Println(err)
		return
	}
	mesh, err := filled.Tessellate(context.Background(), units.Millimeters(0.2),
		decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("faces: %d, vertices: %d, triangles: %d\n", len(filled.Faces()), len(mesh.Vertices()), len(mesh.Triangles()))
	fmt.Printf("boundary verified: %t, volume verified: %t\n", mesh.BoundaryVerified(), mesh.VolumeVerified())
	// Output:
	// faces: 3, vertices: 32, triangles: 60
	// boundary verified: true, volume verified: false
}
