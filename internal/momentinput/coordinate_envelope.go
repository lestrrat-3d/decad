package momentinput

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CoordinateUpper reads the largest analytic walk coordinate envelope in a
// profile, each walk read as CoordinateEnvelope reads it (WalkCoordinateUpper),
// so the two agree on every analytic profile. A non-nil set of pre-resolved walks must match this exact record,
// comparing the segments rather than just their count; a mismatch refuses
// instead of silently resolving again. Nil resolves each segment at the read site.
func CoordinateUpper(profile Profile, work *freeform.FreeformWork, walks *ProfileWalks) (float64, error) {
	if walks != nil && !walks.Matches(profile) {
		return 0, ErrResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := ResolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, err
			}
			if err := boundarywalk.RequireAnalyticWalk(w, "a placed cap frame"); err != nil {
				return 0, err
			}
			upper = math.Max(upper, WalkCoordinateUpper(w))
		}
	}
	return upper, nil
}

// CoordinateEnvelope includes free-form walks, whose control points bound
// their coordinate magnitude through boundarywalk.WalkOf. A boundary-extreme
// scan or centroid reach proof needs only this envelope; a placed cap frame
// still requires an analytic walk. A non-nil walk set must match this record.
//
// Like the walks' own CoordUpper it bounds the L1 magnitude |u| + |v| of every
// point of the section, the reading revolveaxis.Frame.RadialUpper takes as a
// radius bound, so a per-coordinate (L∞) reading such as
// capband.SegmentCoordinateUpper is not a substitute. A circular walk is read
// as WalkCoordinateUpper's tighter bound where that is smaller.
func CoordinateEnvelope(profile Profile, work *freeform.FreeformWork, walks *ProfileWalks) (float64, error) {
	if walks != nil && !walks.Matches(profile) {
		return 0, ErrResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := ResolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, err
			}
			upper = math.Max(upper, WalkCoordinateUpper(w))
		}
	}
	return upper, nil
}

// WalkCoordinateUpper is a walk's L1 coordinate bound, tightened for a circular
// walk. Every point of a circle about (cu, cv) of radius R has
// |u| + |v| ≤ |cu| + |cv| + R(|cos θ| + |sin θ|) ≤ |cu| + |cv| + √2·R, and a
// recorded arc or trimmed circle lies on its whole circle. R is read as the
// walk's radius plus its proven bound (RadiusBound, the bracket of the exact
// Start-to-Center distance for an ArcSeg), and math.Sqrt2 rounds above √2.
// The walk's own CoordUpper, |cu| + |cv| + 2·radiusUpper with an ArcSeg's
// radiusUpper read off the coordinates' L1 sizes, stands where it is smaller
// and for every other kind.
func WalkCoordinateUpper(w survey2d.SegmentWalk) float64 {
	if !w.IsCircular() || proofbound.IsNonFinite(w.RadiusBound) {
		return w.CoordUpper
	}
	radius := proofbound.AbsSumUpper(w.Radius, w.RadiusBound)
	return math.Min(w.CoordUpper, proofbound.AbsSumUpper(w.CU, w.CV, proofbound.ProductUpper(math.Sqrt2, radius)))
}

// ResolveOrRead returns one recorded segment's walk. A pre-resolved walk
// replays its measured per-read charges, so a cache read consumes the same
// work as a fresh resolution; without one, WalkOf resolves it. The caller
// checks that walks matches the profile before calling.
func ResolveOrRead(seg sectionrecord.CurveSegment, work *freeform.FreeformWork,
	walks *ProfileWalks, loopIndex, segIndex int) (survey2d.SegmentWalk, error) {
	if walks != nil {
		if walks.ReadCharges != nil {
			charge := walks.ReadCharges[loopIndex][segIndex]
			if err := work.Step(charge.Spent); err != nil {
				return survey2d.SegmentWalk{}, err
			}
			if err := work.ReconstructionStep(charge.ReconstructionSpent); err != nil {
				return survey2d.SegmentWalk{}, err
			}
		}
		return walks.At(loopIndex, segIndex), nil
	}
	return boundarywalk.WalkOf(seg, work)
}
