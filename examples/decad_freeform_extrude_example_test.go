package examples_test

import (
	"bytes"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A Tier A free-form section — a spline, a closed spline, a fit spline, or a
// unit-weight NURBS curve — extrudes like any other: the side face it builds
// is opaque (KindNURBS), but its Volume is exact rational area times the sweep
// height and its rim edges publish a proven arc-length bracket, so the body is
// a fully measured solid.
//
// It also tessellates, and so exports to STL and OBJ. The free-form wall is
// chorded by exact bisection of its own Bézier chain, at a proven sagitta the
// requested tolerance caps, and the chording is shared with the caps that meet
// it, so the mesh is watertight by construction. Mesh.Bound reports the whole
// deviation the mesh took, chording included.
//
// A free-form curve must still meet its neighbours at shared endpoints, never
// by crossing (docs/spline-design.md §2.1) — a caller cannot guess that from
// the error alone, so join the endpoints in the sketch when a profile is
// rejected as ErrUnrecordableProfile.
func Example_decad_freeformExtrude() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A shallow fit-spline hump through three points, closed by a straight
	// chord back to its start.
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	if _, err := s.CreateFitSpline(start, mid, end); err != nil {
		fmt.Printf("failed to create fit spline: %s\n", err)
		return
	}
	s.CreateLine(end, start)

	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			prof = p
			break
		}
	}

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	fmt.Printf("faces: %d\n", len(body.Faces()))

	// Find the free-form wall among the built faces: the one whose surface
	// is a NURBSSurface. The straight closing chord's wall stays a Plane.
	var freeformFaces int
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindNURBS {
			freeformFaces++
		}
	}
	fmt.Printf("free-form (NURBS) walls: %d\n", freeformFaces)

	// Tessellate chords the free-form wall. The chord count follows from exact
	// rational bisection, so it is the same on every platform; the bound itself
	// is a float64 whose last bits are not, which is why the relation is
	// printed rather than the number.
	const tolerance = 0.1
	mesh, err := body.Tessellate(units.Millimeters(tolerance))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	fmt.Printf("mesh triangles: %d\n", len(mesh.Triangles()))
	fmt.Printf("chording within the requested tolerance: %t\n", mesh.Bound().Base() <= tolerance)

	// STL export writes that same mesh.
	var stl bytes.Buffer
	if err := body.STL(&stl); err != nil {
		fmt.Printf("failed to export STL: %s\n", err)
		return
	}
	fmt.Printf("STL export is non-empty: %t\n", stl.Len() > 0)
	// Output:
	// volume: 150 mm^3 (Approximate)
	// faces: 4
	// free-form (NURBS) walls: 1
	// mesh triangles: 24
	// chording within the requested tolerance: true
	// STL export is non-empty: true
}
