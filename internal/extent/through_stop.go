package extent

import (
	"cmp"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type throughStop[T any] struct {
	far, hi, delta float64
	denoted        *big.Rat
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
// denoted is the exact stop level the body's far side denotes, or nil.
func (s *ThroughStops[T]) Add(hi, bound, axial float64, ref T, denoted *big.Rat) error {
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
	s.stops = append(s.stops, throughStop[T]{far: far, hi: hi, delta: delta, denoted: denoted, ref: ref})
	return nil
}

// Finish publishes the held farthest stop, a bound covering every competing
// far-end interval, dependencies ordered nearest far side first, and the
// level the stop denotes. That level is the farthest body's denoted far side,
// and only when every other body's far-end interval ends at or below the
// farthest one's lower end, so no other body can be the true farthest. It is
// nil otherwise.
func (s *ThroughStops[T]) Finish() (float64, float64, []T, *big.Rat, error) {
	if len(s.stops) == 0 {
		return 0, 0, nil, nil, fmt.Errorf(`%w: a through-all sweep found no live body in its path`, decaderr.ErrDegenerate)
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
			return 0, 0, nil, nil, fmt.Errorf(`%w: a through-all far-end uncertainty is not finite`, decaderr.ErrUnsupported)
		}
		upper := candidate.far
		if candidate.delta != 0 {
			upper = proofbound.UpRound(candidate.far + candidate.delta)
		}
		if proofbound.IsNonFinite(upper) {
			return 0, 0, nil, nil, fmt.Errorf(`%w: a through-all far-end uncertainty cannot be represented`, decaderr.ErrUnsupported)
		}
		if upper > last.far {
			farDelta = math.Max(farDelta, proofbound.UpRound(upper-last.far))
		}
	}
	stop := s.travel * last.far
	delta := proofbound.AbsSumUpper(throughStopRound(s.origin, s.dir, last.hi, s.travel, stop), farDelta)
	return stop, delta, refs, s.uniqueDenoted(), nil
}

// uniqueDenoted is the farthest stop's denoted level when every other stop's
// far-end interval ends at or below the farthest one's lower end, else nil.
func (s *ThroughStops[T]) uniqueDenoted() *big.Rat {
	last := s.stops[len(s.stops)-1]
	far, delta := proofarith.FloatRat(last.far), proofarith.FloatRat(last.delta)
	if last.denoted == nil || far == nil || delta == nil {
		return nil
	}
	lower := new(big.Rat).Sub(far, delta)
	for _, other := range s.stops[:len(s.stops)-1] {
		of, od := proofarith.FloatRat(other.far), proofarith.FloatRat(other.delta)
		if of == nil || od == nil || new(big.Rat).Add(of, od).Cmp(lower) > 0 {
			return nil
		}
	}
	return new(big.Rat).Set(last.denoted)
}
