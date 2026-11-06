package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LoftFaces with WithConeTangentEnd lofts through the cap faces of a pointed
// solid that was cut into slabs, and closes the loft with one more section
// built tangent to a cone. The cone is a full Revolve of a straight meridian
// about world X; its apex and slope fix the end section exactly, so the loft
// ends on the cone rather than on a slab cap.
func Example_loft_faces_cone_tangent() {
	ctx := context.Background()
	doc := decad.New()
	w := sketch.NewWorld()

	// A thin rectangle in a plane through (8, 0, 0), lofted to the origin.
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0.6, -0.8, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		fmt.Printf("failed to create frame: %s\n", err)
		return
	}
	plane, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-3.025, -0.025, -2.975, 0.025)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	spike, err := decad.LoftFromPoint(ctx, doc, r3.NewVec(0, 0, 0), s, s.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}

	// A sheet standing on the line x = at, wide enough to cut the spike.
	sheet := func(at float64) (*decad.Body, error) {
		ss, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		a, b := ss.CreatePoint(at, -20), ss.CreatePoint(at, 20)
		ss.Fix(a)
		ss.Fix(b)
		ss.CreateLine(a, b)
		if _, err := ss.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, ss, ss.Chains()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(20)},
			Two: decad.DistanceSide{D: units.Millimeters(20)}})
	}
	cut1, err := sheet(5)
	if err != nil {
		fmt.Printf("failed to build the first sheet: %s\n", err)
		return
	}
	first, err := decad.Split(ctx, spike, cut1)
	if err != nil {
		fmt.Printf("failed to split: %s\n", err)
		return
	}
	cut2, err := sheet(4.5)
	if err != nil {
		fmt.Printf("failed to build the second sheet: %s\n", err)
		return
	}
	second, err := decad.Split(ctx, first[0], cut2)
	if err != nil {
		fmt.Printf("failed to split: %s\n", err)
		return
	}
	slab := second[1]

	// The slab's two caps are the loft's first sections.
	capOf := func(role decad.FeatureRef) (*decad.Face, error) {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(slab)
		if err != nil {
			return nil, err
		}
		if len(faces) != 1 {
			return nil, fmt.Errorf("want one cap face, got %d", len(faces))
		}
		return faces[0], nil
	}
	toe, err := capOf(decad.CapStart(slab))
	if err != nil {
		fmt.Printf("failed to pick the toe cap: %s\n", err)
		return
	}
	heel, err := capOf(decad.CapEnd(slab))
	if err != nil {
		fmt.Printf("failed to pick the heel cap: %s\n", err)
		return
	}

	// The cone: the triangle (-4,0), (-4,16), (8,0) revolved about X. Its apex
	// is at x = 8 and its slope is exactly 3/4.
	cs, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	pts := []*sketch.Point{cs.CreatePoint(-4, 0), cs.CreatePoint(-4, 16), cs.CreatePoint(8, 0)}
	for _, p := range pts {
		cs.Fix(p)
	}
	for i := range pts {
		cs.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := cs.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	cone, err := decad.Revolve(ctx, doc, cs, cs.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// Loft the two caps and end tangent to the cone along its +Y ruling.
	loft, err := decad.LoftFaces(ctx, []*decad.Face{toe, heel},
		decad.WithConeTangentEnd(cone, r3.NewVec(0, 1, 0)))
	if err != nil {
		fmt.Printf("failed to loft faces: %s\n", err)
		return
	}
	volume, err := loft.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("solid: %t\n", loft.IsSolid())
	// A pyramid on the base plane 6.4 mm from the apex holds 0.0025 x 6.4 / 3 = 0.0053 mm^3; the loft keeps the
	// part past x = 4.5, about 62 percent of it.
	fmt.Printf("volume: %.4f mm^3\n", volume.Value.Base())
	// Output:
	// solid: true
	// volume: 0.0033 mm^3
}
