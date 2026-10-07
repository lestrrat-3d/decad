package main

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// The linkage scenes' hardware colours: steel for the pins, posts and
// bearings, slate for the base plate. Neither reads as the hit colour
// (hitTinted).
var (
	steel = solidlens.RGB(0.5, 0.54, 0.6)
	slate = solidlens.RGB(0.24, 0.29, 0.37)
)

// linkBar is a flat link: a slot outline with semicircular ends of radius
// around the centres p and q in the XY plane, a bore of radius bore through
// each centre listed in bores, and z ∈ [z0, z0 + thickness].
type linkBar struct {
	p, q      [2]float64
	radius    float64
	bore      float64
	bores     []int // 0 for p, 1 for q
	z0        float64
	thickness float64
}

// body sketches the bar on the plane z = z0 with sketch's CreateSlot and one
// circle per bore sharing the slot's cap centre, and extrudes the region
// between them up by thickness.
func (b linkBar) body(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), b.z0)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	slot, err := s.CreateSlot(b.p[0], b.p[1], b.q[0], b.q[1], b.radius)
	if err != nil {
		return nil, err
	}
	for _, p := range []*sketch.Point{slot.C1, slot.C2, slot.A1.Start, slot.A1.End, slot.A2.Start, slot.A2.End} {
		s.Fix(p)
	}
	centres := []*sketch.Point{slot.C1, slot.C2}
	for _, i := range b.bores {
		s.CreateCircle(centres[i], b.bore)
	}
	return extrudeRegion(ctx, doc, s, len(b.bores), b.thickness)
}

// pinStep is one band of a stepped pin: a cylinder of radius over z up to
// top, or a tube when bore is positive.
type pinStep struct {
	radius, top float64
	bore        float64
}

// pinStack is a stepped pin about the vertical axis through (x, y), one
// extruded body per step: step i spans z from the previous step's top
// (bottom for the first) to its own top. Each step is an extruded cylinder:
// a revolved stepped pin made the folding arm's check about three times
// slower, most of it spent tessellating the revolve.
func pinStack(ctx context.Context, doc *decad.Document, x, y, bottom float64, steps []pinStep) ([]*decad.Body, error) {
	bodies := make([]*decad.Body, 0, len(steps))
	z := bottom
	for _, st := range steps {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		center := s.CreatePoint(x, y)
		s.Fix(center)
		s.CreateCircle(center, st.radius)
		holes := 0
		if st.bore > 0 {
			s.CreateCircle(center, st.bore)
			holes = 1
		}
		body, err := extrudeRegion(ctx, doc, s, holes, st.top-z)
		if err != nil {
			return nil, err
		}
		bodies = append(bodies, body)
		z = st.top
	}
	return bodies, nil
}

// extrudeRegion solves s and extrudes, up by height, its one valid region
// with the given number of holes.
func extrudeRegion(ctx context.Context, doc *decad.Document, s *sketch.Sketch, holes int, height float64) (*decad.Body, error) {
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes) == holes {
			if profile != nil {
				return nil, fmt.Errorf("sketch bounds two valid regions with %d holes", holes)
			}
			profile = candidate
		}
	}
	if profile == nil {
		return nil, fmt.Errorf("sketch bounds no valid region with %d holes", holes)
	}
	// Extrude has no context-aware variant; Solve above is the cancellable phase.
	return doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(height), Dir: decad.Along}) //nolint:contextcheck
}
