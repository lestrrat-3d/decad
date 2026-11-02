package momentinput

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// FirstMomentsReplay reads the producer's first-order scalar measurements only
// when both immutable cache keys match the recorded analytic profile. It does
// not copy the cache's rational pointers or evaluate a segment.
func (pw *ProfileWalks) FirstMomentsReplay(ctx context.Context, profile Profile,
	budget *proofbound.ReplayBudget) (Integrals, bool, error) {
	if pw == nil || budget == nil || !pw.Metered || pw.Spent != 0 ||
		pw.ReconstructionSpent != 0 || pw.integrals == nil ||
		pw.integralOrder < freeform.MomentFirstOrder ||
		len(pw.Holes) != len(profile.Holes) {
		return Integrals{}, false, nil
	}
	for _, key := range []Profile{pw.Profile, pw.integralProfile} {
		if len(key.Holes) != len(profile.Holes) || len(pw.Outer) != len(profile.Outer.Segments) {
			return Integrals{}, false, nil
		}
		for i := -1; i < len(profile.Holes); i++ {
			var a, b sectionrecord.LoopRecord
			var walks int
			if i < 0 {
				a, b, walks = key.Outer, profile.Outer, len(pw.Outer)
			} else {
				a, b, walks = key.Holes[i], profile.Holes[i], len(pw.Holes[i])
			}
			if len(a.Segments) != len(b.Segments) || walks != len(b.Segments) {
				return Integrals{}, false, nil
			}
			for j := range b.Segments {
				if err := budget.SpendWork(ctx, 64); err != nil {
					return Integrals{}, false, err
				}
				if !sectionrecord.IdenticalAnalyticSegment(a.Segments[j], b.Segments[j]) {
					return Integrals{}, false, nil
				}
			}
		}
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return Integrals{}, false, err
	}
	cached := pw.integrals
	if !cached.IsFinite(freeform.MomentFirstOrder) || cached.Area <= 0 ||
		proofbound.IsNonFinite(cached.CoordUpper) || cached.CoordUpper < 0 {
		return Integrals{}, false, nil
	}
	return Integrals{CoordUpper: cached.CoordUpper, Area: cached.Area, AreaBound: cached.AreaBound,
		Mu: cached.Mu, MuBound: cached.MuBound, Mv: cached.Mv,
		MvBound: cached.MvBound}, true, nil
}
