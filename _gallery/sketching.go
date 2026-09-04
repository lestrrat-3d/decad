package main

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/sketch"
)

// point is a plane-local sketch coordinate in millimetres.
type point struct {
	x, y float64
}

func rectangle(x0, y0, x1, y1 float64) []point {
	return []point{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
}

func circle(cx, cy, radius float64, segments int) []point {
	points := make([]point, segments)
	for i := range points {
		angle := 2 * math.Pi * float64(i) / float64(segments)
		points[i] = point{cx + radius*math.Cos(angle), cy + radius*math.Sin(angle)}
	}
	return points
}

// polylineProfile draws each loop as a closed chain of lines in s, solves the
// sketch, and returns the region those loops bound. The first loop is the
// outline and every later one is a hole, so the profile is picked by hole
// count rather than by position in sketch's own ordering.
func polylineProfile(ctx context.Context, s *sketch.Sketch, loops [][]point) (*sketch.Profile, error) {
	for loopIndex, loop := range loops {
		if len(loop) < 3 {
			return nil, fmt.Errorf("loop %d has fewer than three points", loopIndex)
		}
		points := make([]*sketch.Point, len(loop))
		for i, p := range loop {
			points[i] = s.CreatePoint(p.x, p.y)
		}
		if loopIndex == 0 {
			s.Fix(points[0])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes)+1 == len(loops) {
			return candidate, nil
		}
	}
	return nil, fmt.Errorf("sketch produced no profile for %d loops", len(loops))
}
