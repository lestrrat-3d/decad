package decad

import (
	"context"
	"math"
	"reflect"

	"github.com/lestrrat-3d/r3"
)

// analyticBodiesEqual is the exact set-identity fast path for evaluator
// payloads whose records are already in their normalized value form. Exact
// structural equality is deliberately only a sufficient certificate: a
// different record that might describe the same set stays undecided.
func analyticBodiesEqual(a, b *Body) bool {
	switch pa := a.payload.(type) {
	case prismPayload:
		pb, ok := b.payload.(prismPayload)
		return ok && reflect.DeepEqual(pa, pb)
	case cupPayload:
		pb, ok := b.payload.(cupPayload)
		return ok && reflect.DeepEqual(pa, pb)
	case revolvePayload:
		pb, ok := b.payload.(revolvePayload)
		return ok && reflect.DeepEqual(pa, pb)
	default:
		return false
	}
}

// measuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly;
// every other pair uses the read-only mesh intersection and its positive
// lower-volume gate.
func measuredInterference(ctx context.Context, a, b *Body, res pairResult) (Measurement, bool, error) {
	if res.contained != nil {
		return res.contained.volume, true, nil
	}
	if analyticBodiesEqual(a, b) {
		// Stable pair order chooses A when the represented sets are equal.
		return a.volume, true, nil
	}
	eval, err := evaluateBoolean(ctx, OpIntersect, a, b)
	if err != nil {
		if _, ok := asExpectedBoolean(err); ok {
			return Measurement{}, false, nil
		}
		return Measurement{}, false, err
	}
	value := math.Abs(eval.volume.Value.Base())
	if value-eval.volume.Bound.Base() <= 0 {
		return Measurement{}, false, nil
	}
	return eval.volume, true, nil
}

// interferencePairDiameter reads the greatest supported point distance from
// the pair. Analytic bodies contribute their exact support set from the
// clearance model; other shipped payloads contribute their held topology
// vertices. An incomplete set can only understate D and tighten the noise
// floor, never admit a coarse answer.
func interferencePairDiameter(ctx context.Context, a, b *Body) (float64, error) {
	var points []r3.Vec
	for _, body := range []*Body{a, b} {
		if geom, ok := newBodyGeom(body); ok {
			points = append(points, geom.supports...)
			continue
		}
		for _, vertex := range body.Vertices() {
			points = append(points, vertex.position)
		}
	}
	best := 0.0
	work := 0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return 0, err
				}
			}
			best = math.Max(best, points[i].Sub(points[j]).Len())
		}
	}
	return best, ctx.Err()
}

// interferenceWithinTolerance applies the pair-local volume gate. The
// overlap boundary lies on the operands' skins, so the noise quantum uses
// their summed surface areas rather than the transient intersection mesh.
func interferenceWithinTolerance(volume Measurement, a, b *Body, pairD, rel float64) bool {
	const eps = 1e-9
	area := math.Abs(a.area.Value.Base()) + math.Abs(b.area.Value.Base())
	quantum := eps * pairD * area
	return measurementWithinTolerance(volume, rel, func(value float64) (float64, bool) {
		ref := math.Max(value, quantum)
		return ref, usableMagnitude(ref)
	})
}
