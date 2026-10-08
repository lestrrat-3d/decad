package momentinput

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CoordinateUpper reads the largest analytic walk coordinate envelope in a
// profile. A non-nil set of pre-resolved walks must match this exact record,
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
			upper = math.Max(upper, w.CoordUpper)
		}
	}
	return upper, nil
}

// CoordinateEnvelope includes free-form walks, whose control points bound
// their coordinate magnitude through boundarywalk.WalkOf. A boundary-extreme
// scan or centroid reach proof needs only this envelope; a placed cap frame
// still requires an analytic walk. A non-nil walk set must match this record.
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
			upper = math.Max(upper, w.CoordUpper)
		}
	}
	return upper, nil
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
