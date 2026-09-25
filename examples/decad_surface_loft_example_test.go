package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithSurfaceResult on Loft builds the wall triangles alone and omits both
// section caps, publishing a sheet body — Kind() == BodySheet — instead of a
// solid (docs/surface-design.md §4). Volume and Centroid answer ErrNotSolid
// for a sheet, proven sound or not; Area and Bounds still answer, over the
// walls alone. Tessellate and OBJ export the wall triangles with both section
// rims free (docs/surface-design.md §10).
func Example_decad_surfaceLoft() {
	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create the bottom sketch: %s\n", err)
		return
	}
	bottom := s0.CreateRectangle(-20, -20, 20, 20)
	s0.Fix(bottom.A)
	if _, err := s0.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the bottom sketch: %s\n", err)
		return
	}

	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create the top plane: %s\n", err)
		return
	}
	s1, err := w.CreateSketch(top)
	if err != nil {
		fmt.Printf("failed to create the top sketch: %s\n", err)
		return
	}
	topRect := s1.CreateRectangle(-20, -20, 20, 20)
	s1.Fix(topRect.A)
	if _, err := s1.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the top sketch: %s\n", err)
		return
	}

	doc := decad.New()
	sheet, err := doc.Loft(context.Background(), s0, s0.Profiles()[0], s1, s1.Profiles()[0], decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
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
	// region: the crossing audit that built the body already ran over the
	// COMPLETE held triangle set — walls and both omitted caps together — so
	// non-self-intersection of the walls alone follows with no further proof
	// needed (docs/surface-design.md §9.1).
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	mesh, err := sheet.Tessellate(context.Background(), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	var obj bytes.Buffer
	if err := export.OBJ(context.Background(), &obj, sheet, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to export OBJ: %s\n", err)
		return
	}

	fmt.Printf("is sheet: %v\n", sheet.Kind() == decad.BodySheet)
	fmt.Printf("faces: %d\n", len(sheet.Faces()))
	fmt.Printf("free edges: %d\n", len(free))
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("volume error: %v\n", volErr)
	fmt.Printf("verify status: %s\n", report.Status)
	fmt.Printf("triangles: %d\n", len(mesh.Triangles()))
	fmt.Printf("OBJ faces: %d\n", strings.Count(obj.String(), "\nf "))
	// Output:
	// is sheet: true
	// faces: 8
	// free edges: 8
	// area: 1600 mm^2
	// volume error: decad: body is not a solid
	// verify status: Sound
	// triangles: 8
	// OBJ faces: 8
}
