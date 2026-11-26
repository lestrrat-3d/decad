package examples_test

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// RefineTo joins a straight-bevel sector join, a revolved blank and
// copies of one point-Loft tooth, directly at the bound the caller wants.
// Union builds such a join at 0.1 mm and Refine rebuilds it finer, so
// Union(...).RefineTo builds it once. A set that is not a sector join is refused
// before anything is built, and the caller falls back to Union then Refine.
func Example_unionRefined() {
	ctx := context.Background()
	doc := decad.New()

	// coneTool revolves a right triangle about the X axis into a cone with its
	// apex at (apex, 0, 0).
	coneTool := func(apex float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		points := []*sketch.Point{s.CreatePoint(-2, 0), s.CreatePoint(-2, apex+2), s.CreatePoint(apex, 0)}
		for _, p := range points {
			s.Fix(p)
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Revolve(doc, s, s.Profiles()[0],
			decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{}).Do(ctx)
	}

	// The blank: a ring section revolved about the X axis.
	blankWorld := sketch.NewWorld()
	section, err := blankWorld.CreateSketch(blankWorld.XY())
	if err != nil {
		fmt.Printf("failed to create the blank sketch: %s\n", err)
		return
	}
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: 9, V: 1.5}, {U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, p := range outline {
		points[i] = section.CreatePoint(p.U, p.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := section.Solve(ctx); err != nil {
		fmt.Printf("failed to solve the blank sketch: %s\n", err)
		return
	}
	blank, err := decad.Revolve(doc, section, section.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{}).Do(ctx)
	if err != nil {
		fmt.Printf("failed to revolve the blank: %s\n", err)
		return
	}

	// The tooth: a profile lofted to a point, trimmed by two cones.
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, -1))
	if err != nil {
		fmt.Printf("failed to build the tooth frame: %s\n", err)
		return
	}
	toothWorld := sketch.NewWorld()
	plane, err := toothWorld.CreatePlaneFromFrame(frame)
	if err != nil {
		fmt.Printf("failed to build the tooth plane: %s\n", err)
		return
	}
	tooth, err := toothWorld.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create the tooth sketch: %s\n", err)
		return
	}
	const half = 0.8
	rootLow, flankLow := tooth.CreatePoint(-4.3, -half), tooth.CreatePoint(-5.5, -half)
	tipLow, tipHigh := tooth.CreatePoint(-6, -half), tooth.CreatePoint(-6, half)
	flankHigh, rootHigh := tooth.CreatePoint(-5.5, half), tooth.CreatePoint(-4.3, half)
	center := tooth.CreatePoint(0, 0)
	for _, p := range []*sketch.Point{rootLow, flankLow, tipLow, tipHigh, flankHigh, rootHigh, center} {
		tooth.Fix(p)
	}
	tooth.CreateLine(rootLow, flankLow)
	tooth.CreateLine(flankLow, tipLow)
	tooth.CreateArc(center, tipHigh, tipLow)
	tooth.CreateLine(tipHigh, flankHigh)
	tooth.CreateLine(flankHigh, rootHigh)
	tooth.CreateArc(center, rootHigh, rootLow)
	if _, err := tooth.Solve(ctx); err != nil {
		fmt.Printf("failed to solve the tooth sketch: %s\n", err)
		return
	}
	lug, err := decad.LoftFromPoint(doc, r3.Vec{}, tooth, tooth.Profiles()[0]).Do(ctx)
	if err != nil {
		fmt.Printf("failed to loft the tooth: %s\n", err)
		return
	}
	toeCone, err := coneTool(7.5)
	if err != nil {
		fmt.Printf("failed to build the toe cone: %s\n", err)
		return
	}
	toeTrim, err := decad.Cut(lug, toeCone).Do(ctx)
	if err != nil {
		fmt.Printf("failed to trim the toe: %s\n", err)
		return
	}
	heelCone, err := coneTool(12)
	if err != nil {
		fmt.Printf("failed to build the heel cone: %s\n", err)
		return
	}
	trimmed, err := decad.Intersect(toeTrim, heelCone).Do(ctx)
	if err != nil {
		fmt.Printf("failed to trim the heel: %s\n", err)
		return
	}

	// Six teeth: the trimmed one and five copies turned about the axis.
	const count = 6
	bodies := []*decad.Body{blank, trimmed}
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/float64(count)))
		if err != nil {
			fmt.Printf("failed to build a rotation: %s\n", err)
			return
		}
		copied, err := decad.Copy(trimmed).Place(decad.CopyTransform{Transform: turn}).Do(ctx)
		if err != nil {
			fmt.Printf("failed to copy a tooth: %s\n", err)
			return
		}
		bodies = append(bodies, copied)
	}

	// A pair is no sector join: it is refused before anything is built, and
	// both bodies stay live.
	_, err = decad.Union(bodies[:2]...).RefineTo(units.Millimeters(0.03)).Do(ctx)
	fmt.Println("a pair is refused:", errors.Is(err, decad.ErrUnsupported), "bodies live:", len(doc.Bodies()) == len(bodies))

	// The join is built once at 0.03 mm.
	joined, err := decad.Union(bodies...).RefineTo(units.Millimeters(0.03)).Do(ctx)
	if err != nil {
		fmt.Printf("failed to join the teeth: %s\n", err)
		return
	}
	mesh, err := joined.Tessellate(ctx, units.Millimeters(0.03))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	bound, err := mesh.Bound().In(units.Millimeter)
	if err != nil {
		fmt.Printf("failed to read the bound: %s\n", err)
		return
	}
	fmt.Println("joined bound within 0.03 mm:", bound <= 0.03, "bodies live:", len(doc.Bodies()))
	// Output:
	// a pair is refused: true bodies live: true
	// joined bound within 0.03 mm: true bodies live: 1
}
