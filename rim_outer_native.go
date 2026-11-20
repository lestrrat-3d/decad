package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// When the joined nose ends on its cone heel, the adjacent arc is present
// only in the original cheek profile. Its two neighboring sides determine
// the sole centered arc. The native construction seal authenticates its face.
func (r rimFaceReader) bindAdjacentRound(ctx context.Context, a *rimextension.OuterAdmission) (bool, error) {
	native := a.Side.Body
	pp, yes := native.Payload.(prismrecord.Payload)
	if !yes || len(pp.Profile.Outer.Segments) > 64 {
		return false, nil
	}
	fs, err := r.faceList(ctx, native)
	if err != nil {
		return false, err
	}
	if err = r.meter.ReserveBundle(ctx, uint64(128+len(fs)*4), 8, 8192); err != nil {
		return false, err
	}
	idx := -1
	for i := range pp.Profile.Outer.Segments {
		if len(a.Side.Native.Origins) == 1 && a.Side.Native.Origins[0].Role == fmt.Sprintf("side(0,%d)", i) {
			idx = i
		}
	}
	if idx < 0 {
		return false, nil
	}
	count := len(pp.Profile.Outer.Segments)
	for _, i := range []int{(idx + 1) % count, (idx + count - 1) % count} {
		arc, yes := pp.Profile.Outer.Segments[i].(sectionrecord.ArcSeg)
		if !yes || arc.Center != (sectionrecord.Point2{}) {
			continue
		}
		for _, f := range fs {
			if len(f.Origins) == 1 && f.Origins[0].Role == fmt.Sprintf("side(0,%d)", i) {
				if a.Round.Native != nil {
					return false, nil
				}
				bound, e := native.ReplayPrismBuiltFace(ctx, f, r.meter)
				if !bound || e != nil {
					return false, e
				}
				a.Round = rimextension.BoundFace{Body: native, Native: f, Translation: a.Side.Translation}
			}
		}
	}
	if a.Round.Native == nil {
		return false, nil
	}
	return true, nil
}
