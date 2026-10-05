package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// This file is the scheduled step's reading of a band
// (docs/multibody-dynamics-design.md §10.3 and §10.4): a band track, whose
// depth grows while a pair rests or rolls at constant ω, and the ContactBand
// relation of a body whose held boundary carries a positive displacement.
// Both are touching relations whose penetration bound must lie within
// PenetrationResidual.

// bandEnd reads a SweepPersistentBand of a pair the contact set continues.
// A track that starts at the slice start, reaches its end and stays within
// PenetrationResidual throughout continues the pair with no event (full).
// Otherwise the slice ends at cut: the largest fraction of the sweep's own
// dyadic grid, no later than the track's end, through which the band stays
// within the residual (§10.3). ok is false when no positive fraction does.
func (w *World) bandEnd(sweep *decad.SweepReport) (*big.Rat, bool, bool) {
	track := sweep.ContactTrack
	if track == nil || track.Start().Fraction.Base() != 0 {
		return nil, false, false
	}
	end := exactBase(track.End().Fraction)
	if end == nil || end.Sign() <= 0 {
		return nil, false, false
	}
	within := func(f *big.Rat) bool {
		fraction, _ := f.Float64()
		band, err := track.BandAt(units.Scalar(fraction))
		return err == nil && band != nil && w.bandWithin(*band)
	}
	if within(end) {
		if end.Cmp(big.NewRat(1, 1)) == 0 && w.trackManifoldsWithin(track) {
			return nil, true, true
		}
		return end, false, true
	}
	duration := exactBase(pathDuration(sweep.PathA))
	resolution := exactBase(sweep.Request.TimeResolution)
	if duration == nil || resolution == nil || resolution.Sign() <= 0 {
		return nil, false, false
	}
	depth := uint(0)
	for depth < 52 && new(big.Rat).Mul(resolution, new(big.Rat).SetInt64(int64(1)<<depth)).Cmp(duration) < 0 {
		depth++
	}
	scale := new(big.Rat).SetInt64(int64(1) << depth)
	// The band grows with the fraction, so a binary search over the grid
	// below the track's end finds the last fraction within the residual.
	limit := new(big.Rat).Mul(end, scale)
	high := new(big.Int).Quo(limit.Num(), limit.Denom()).Int64()
	low := int64(0)
	for high-low > 1 {
		mid := low + (high-low)/2
		if within(big.NewRat(mid, int64(1)<<depth)) {
			low = mid
		} else {
			high = mid
		}
	}
	if low == 0 || !within(big.NewRat(low, int64(1)<<depth)) {
		return nil, false, false
	}
	return big.NewRat(low, int64(1)<<depth), false, true
}

// trackManifoldsWithin reads a track's manifold at both ends and the middle;
// each must stay within the contact request and PenetrationResidual.
func (w *World) trackManifoldsWithin(track *decad.SweepContactTrack) bool {
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(0.5), units.Scalar(1)} {
		manifold, err := track.ManifoldAt(fraction)
		if err != nil || !w.manifoldWithin(manifold) || !w.penetrationWithin(manifold) {
			return false
		}
	}
	return true
}

// bandWithin reports whether a band's upper end, Value plus Bound, lies
// within PenetrationResidual.
func (w *World) bandWithin(band decad.Measurement) bool {
	return finite(band.Value.Base(), band.Bound.Base()) && band.Value.Base() >= 0 && band.Bound.Base() >= 0 &&
		outwardSum(band.Value.Base(), band.Bound.Base()) <= w.step.PenetrationResidual.Base()
}

// contactBandWithin is §10.4's admission of a ContactBand relation: its gap
// band, Gap.Bound about a zero Gap.Value, lies within PenetrationResidual. A
// band that is not admitted leaves the pair undecided, never touching.
func (w *World) contactBandWithin(gap *decad.Measurement) bool {
	return gap != nil && gap.Value.Base() == 0 && w.bandWithin(decad.Measurement{
		Value: units.Millimeters(0), Bound: gap.Bound})
}

// continuesTrack reports whether a slice's final sweep keeps its pair in the
// contact set through the end of the step: a persistent touch track, or a
// band track that spans the slice within PenetrationResidual.
func (w *World) continuesTrack(sweep *decad.SweepReport) bool {
	switch sweep.Outcome {
	case decad.SweepPersistentTouch:
		return true
	case decad.SweepPersistentBand:
		_, full, ok := w.bandEnd(sweep)
		return ok && full
	default:
		return false
	}
}

// bandDepth is the upper end of a track's band through fraction f, the
// allowance §6.6 adds for a slice that ended on that band, or zero for an
// exact touch track.
func bandDepth(track *decad.SweepContactTrack, f float64) (float64, bool) {
	band, err := track.BandAt(units.Scalar(f))
	if err != nil {
		return 0, false
	}
	if band == nil {
		return 0, true
	}
	depth := outwardSum(band.Value.Base(), band.Bound.Base())
	return depth, finite(depth)
}

// fractionInstant labels a slice fraction for a sweep of the given duration:
// its fraction and the elapsed time, rounded once with that rounding as the
// bound.
func fractionInstant(f *big.Rat, duration units.Value) decad.SweepInstant {
	fraction, _ := f.Float64()
	elapsed := new(big.Rat).Mul(f, exactBase(duration))
	seconds, _ := elapsed.Float64()
	bound := new(big.Rat).Abs(new(big.Rat).Sub(elapsed, ratFloat(seconds)))
	slack, _ := bound.Float64()
	if ratFloat(slack).Cmp(bound) < 0 {
		slack = math.Nextafter(slack, math.Inf(1))
	}
	exactness := decad.Exact
	if slack != 0 {
		exactness = decad.Approximate
	}
	return decad.SweepInstant{Fraction: units.Scalar(fraction),
		Elapsed: decad.Measurement{Value: units.Seconds(seconds), Bound: units.Seconds(slack), Exactness: exactness}}
}
