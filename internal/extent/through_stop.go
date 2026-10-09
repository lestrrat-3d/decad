package extent

import (
	"cmp"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type throughStop[T any] struct {
	far, hi, delta float64
	ref            T
}

// ThroughStops resolves the far side of each occupied body against one sweep
// direction. Add must be called in live-body order so equal stops retain that
// order in the recorded dependencies.
type ThroughStops[T any] struct {
	origin, dir  r3.Vec
	travel, base float64
	stops        []throughStop[T]
}

// NewThroughStops starts a body-relative stop reading in the travel direction.
func NewThroughStops[T any](origin, dir r3.Vec, travel float64) *ThroughStops[T] {
	return &ThroughStops[T]{origin: origin, dir: dir, travel: travel, base: origin.Dot(dir)}
}

// Add examines a body's held far endpoint and both of its proven endpoint
// displacements. It refuses an interval that straddles the sketch plane.
func (s *ThroughStops[T]) Add(hi, bound, axial float64, ref T) error {
	// An all-analytic endpoint with two zero displacement terms stays exact.
	delta := bound
	if axial != 0 {
		delta = axial
		if bound != 0 {
			delta = proofbound.AbsSumUpper(bound, axial)
		}
	}
	far := hi - s.base
	tol := RelativeStopTolerance(math.Max(math.Abs(hi), math.Abs(s.base)))
	switch {
	case freeform.DownRound(far-delta) > tol:
		// Material lies beyond the plane outside its displacement.
	case proofbound.UpRound(far+delta) <= tol:
		return nil
	default:
		return fmt.Errorf(`%w: a through-all sweep cannot decide whether a body is in its path: its far side sits %v mm beyond the sketch plane, known only to a displacement of %v mm`, decaderr.ErrUnsupported, far, delta)
	}
	s.stops = append(s.stops, throughStop[T]{far: far, hi: hi, delta: delta, ref: ref})
	return nil
}

// Finish publishes the held farthest stop, a bound covering every competing
// far-end interval, and dependencies ordered nearest far side first.
func (s *ThroughStops[T]) Finish() (float64, float64, []T, error) {
	if len(s.stops) == 0 {
		return 0, 0, nil, fmt.Errorf(`%w: a through-all sweep found no live body in its path`, decaderr.ErrDegenerate)
	}
	slices.SortStableFunc(s.stops, func(a, b throughStop[T]) int { return cmp.Compare(a.far, b.far) })
	refs := make([]T, len(s.stops))
	for i, stop := range s.stops {
		refs[i] = stop.ref
	}
	last := s.stops[len(s.stops)-1]
	farDelta := last.delta
	for _, candidate := range s.stops {
		if proofbound.IsNonFinite(candidate.far) || proofbound.IsNonFinite(candidate.delta) {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all far-end uncertainty is not finite`, decaderr.ErrUnsupported)
		}
		upper := candidate.far
		if candidate.delta != 0 {
			upper = proofbound.UpRound(candidate.far + candidate.delta)
		}
		if proofbound.IsNonFinite(upper) {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all far-end uncertainty cannot be represented`, decaderr.ErrUnsupported)
		}
		if upper > last.far {
			farDelta = math.Max(farDelta, proofbound.UpRound(upper-last.far))
		}
	}
	stop := s.travel * last.far
	delta := proofbound.AbsSumUpper(ThroughStopRound(s.origin, s.dir, last.hi, s.travel, stop), farDelta)
	return stop, delta, refs, nil
}
