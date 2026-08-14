package examples_test

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A Tier A free-form section — a spline, a closed spline, a fit spline, or a
// unit-weight NURBS curve — now extrudes (docs/spline-design.md §10 P4b): the
// side face it builds is opaque (KindNURBS), but its Volume is exact rational
// area times the sweep height and its rim edges publish a proven arc-length
// bracket, so the body is a fully measured solid.
//
// A free-form curve must still meet its neighbours at shared endpoints, never
// by crossing (docs/spline-design.md §2.1) — a caller cannot guess that from
// the error alone, so join the endpoints in the sketch when a profile is
// rejected as ErrUnrecordableProfile.
//
// What P4b does NOT buy: Tessellate (and so STL/OBJ export) still refuses a
// free-form-walled body, staged for a later increment.
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

	// Tessellate — and so STL/OBJ export — still refuses a free-form-walled
	// body; that capability is staged for a later increment.
	if _, err := body.Tessellate(units.Millimeters(0.1)); err != nil {
		fmt.Printf("tessellate refuses: %s\n", err)
	}
	// Output:
	// volume: 150 mm^3 (Approximate)
	// faces: 4
	// free-form (NURBS) walls: 1
	// tessellate refuses: decad: not supported by the current evaluator: chording a boundary loop does not support a free-form boundary segment
}
