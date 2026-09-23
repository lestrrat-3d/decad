package examples_test

import (
	"bytes"
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithSurfaceResult builds the feature's wall set and omits every face that
// exists only to close the solid, publishing a sheet body — Kind() ==
// BodySheet — instead of a solid (docs/surface-design.md §4). Volume and
// Centroid answer ErrNotSolid for a sheet, proven sound or not; Area and
// Bounds still answer, over the walls alone.
func Example_decad_surfaceExtrude() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, surface-extruded 10 mm along the sketch normal: the
	// two caps a solid extrude would carry are omitted, so every wall's own
	// two rim edges become free.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	if err != nil {
		fmt.Printf("failed to select free edges: %s\n", err)
		return
	}
	area, err := sheet.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	_, volErr := sheet.Volume()

	// A sheet's own boundary can be proven sound even though it encloses no
	// region: Verify's sheet validity audit (docs/surface-design.md §9.1)
	// decides this from the recorded topology, not from a triangulated
	// chord mesh.
	report, err2 := doc.Verify(context.Background())
	if err2 != nil {
		fmt.Printf("failed to verify: %s\n", err2)
		return
	}

	// A sheet tessellates its walls alone, at the manifold-with-boundary
	// audit docs/tessellation-design.md §1.2 runs in the closed-mesh audit's
	// place: 8 triangles against the 12 a solid extrude of the same plate
	// would carry, since the two caps are never chorded.
	mesh, err := sheet.Tessellate(context.Background(), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}

	// STL and OBJ still write a sheet's mesh — both formats are triangle
	// lists and neither requires closure — but Body.STL's own doc comment
	// says the file is NOT a solid despite carrying the format's
	// solid/endsolid keywords.
	var stl bytes.Buffer
	if err := sheet.STL(&stl); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}

	fmt.Printf("is sheet: %v\n", sheet.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(sheet.Faces()))
	fmt.Printf("free edges: %d\n", len(free))
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("volume error: %v\n", volErr)
	fmt.Printf("verify status: %s\n", report.Status)
	fmt.Printf("mesh triangles: %d\n", len(mesh.Triangles()))
	fmt.Printf("stl is solid keyword present: %v\n", bytes.Contains(stl.Bytes(), []byte("solid decad")))
	// Output:
	// is sheet: true
	// faces: 4
	// free edges: 8
	// area: 3200 mm^2
	// volume error: decad: body is not a solid
	// verify status: Sound
	// mesh triangles: 8
	// stl is solid keyword present: true
}
