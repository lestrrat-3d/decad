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

// Pattern a feature about an axis and join the copies to a base. The feature is
// modeled once, as a loft of the same section between two planes; PatternCopies
// repeats it, and one Union joins the base, the feature and its copies. The
// result is a single extrusion of the joined section, so a fillet, a bore and a
// chamfer apply to it afterwards.
func Example_pattern_join() {
	const (
		radius = 3.0 // the base disc's radius, mm
		height = 2.0 // the common thickness, mm
		count  = 12  // lobes about the disc, the first included
	)
	ctx := context.Background()
	doc := decad.New()

	// The base: a disc on the XY plane.
	w := sketch.NewWorld()
	ds, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	ds.CreateCircle(ds.Origin(), radius)
	if _, err := ds.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	disc, err := decad.Extrude(ctx, doc, ds, ds.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude the disc: %s\n", err)
		return
	}

	// The feature: one lobe standing on the disc, with fit-spline flanks.
	top, err := w.CreateOffsetPlane(w.XY(), height)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	half := 0.4 * math.Pi / count
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile, error) {
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, nil, err
		}
		pt := func(rad, a float64) *sketch.Point {
			p := s.CreatePoint(rad*math.Cos(a), rad*math.Sin(a))
			s.Fix(p)
			return p
		}
		const fit = 5
		rr, rl := pt(radius, -half), pt(radius, half)
		right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
		for i := range fit {
			f := float64(i) / (fit - 1)
			rad, a := radius+0.35+0.65*f, half*(1-0.4*f)
			right[i], left[i] = pt(rad, -a), pt(rad, a)
		}
		s.CreateLine(rr, right[0])
		if _, err := s.CreateFitSpline(right...); err != nil {
			return nil, nil, err
		}
		s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
		if _, err := s.CreateFitSpline(left...); err != nil {
			return nil, nil, err
		}
		s.CreateLine(left[0], rl)
		s.CreateArc(s.Origin(), rr, rl)
		if _, err := s.Solve(ctx); err != nil {
			return nil, nil, err
		}
		return s, s.Profiles()[0], nil
	}
	s0, p0, err := section(w.XY())
	if err != nil {
		fmt.Printf("failed to draw the lower section: %s\n", err)
		return
	}
	s1, p1, err := section(top)
	if err != nil {
		fmt.Printf("failed to draw the upper section: %s\n", err)
		return
	}
	lobe, err := decad.Loft(ctx, doc, s0, p0, s1, p1)
	if err != nil {
		fmt.Printf("failed to loft the lobe: %s\n", err)
		return
	}

	// Repeat the lobe about the Z axis, then join the base and every lobe.
	copies, err := decad.PatternCopies(ctx, lobe,
		decad.CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: count})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	joined, err := decad.Union(ctx, disc, append([]*decad.Body{lobe}, copies...)...)
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	lv, err := lobe.Volume()
	if err != nil {
		fmt.Printf("failed to read the lobe volume: %s\n", err)
		return
	}
	jv, err := joined.Volume()
	if err != nil {
		fmt.Printf("failed to read the joined volume: %s\n", err)
		return
	}
	want := math.Pi*radius*radius*height + count*lv.Value.Base()
	fmt.Printf("copies: %d\n", len(copies))
	fmt.Printf("solid: %t\n", joined.IsSolid())
	fmt.Printf("volume matches disc plus lobes: %t\n", math.Abs(jv.Value.Base()-want) <= jv.Bound.Base()+lv.Bound.Base()*count)
	// Output:
	// copies: 11
	// solid: true
	// volume matches disc plus lobes: true
}
