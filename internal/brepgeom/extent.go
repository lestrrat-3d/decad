package brepgeom

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ExtentReader reads one BRep face or band along an axis. The boolean is false
// for a band with no extra extent beyond its face.
type ExtentReader func(context.Context, int, r3.Vec, *freeform.FreeformWork) (float64, float64, float64, bool, error)

// Bounds unions the per-face and per-band extremes along each world axis and
// charges the largest extent error with section and level displacements.
func Bounds(ctx context.Context, count int, read ExtentReader, sectionDelta, axialDelta float64) (r3.Vec, r3.Vec, float64, error) {
	axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	minC := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
	maxC := [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	extremeBound := 0.0
	work := freeform.NewFreeformWork()
	for item := range count {
		for i, axis := range axes {
			if err := ctx.Err(); err != nil {
				return r3.Vec{}, r3.Vec{}, 0, err
			}
			lo, hi, bound, include, err := read(ctx, item, axis, work)
			if err != nil {
				return r3.Vec{}, r3.Vec{}, 0, err
			}
			if !include {
				continue
			}
			minC[i], maxC[i] = math.Min(minC[i], lo), math.Max(maxC[i], hi)
			extremeBound = math.Max(extremeBound, bound)
		}
	}
	terms := make([]float64, 0, 3)
	for _, term := range []float64{sectionDelta, extremeBound, axialDelta} {
		if term != 0 {
			terms = append(terms, term)
		}
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = proofbound.AbsSumUpper(terms...)
	}
	return r3.NewVec(minC[0], minC[1], minC[2]), r3.NewVec(maxC[0], maxC[1], maxC[2]), bound, nil
}

// ExtentAlong unions the same sources along one through-all stop direction.
func ExtentAlong(ctx context.Context, count int, axis r3.Vec, read ExtentReader) (float64, float64, float64, error) {
	lo, hi, bound := math.Inf(1), math.Inf(-1), 0.0
	work := freeform.NewFreeformWork()
	for item := range count {
		l, h, b, include, err := read(ctx, item, axis, work)
		if err != nil {
			return 0, 0, 0, err
		}
		if !include {
			continue
		}
		lo, hi, bound = math.Min(lo, l), math.Max(hi, h), math.Max(bound, b)
	}
	return lo, hi, bound, nil
}
