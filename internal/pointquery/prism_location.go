package pointquery

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LocatePrism classifies the world point p against the solid an undisplaced
// prism record denotes, in exact rational arithmetic over the held record
// (agent-docs/point-containment-design.md §7). decided is false where the
// record is not one this route reads exactly or the point meets a case it
// leaves open; the caller then classifies against the verified mesh (§2).
// p must be finite.
func LocatePrism(ctx context.Context, p r3.Vec, pp prismrecord.Payload) (Location, bool, error) {
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	if pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 || pp.SurfaceResult {
		return Undecided, false, nil
	}
	z0, z1 := proof.FloatRat(pp.Z0), proof.FloatRat(pp.Z1)
	if z0 == nil || z1 == nil || z0.Cmp(z1) >= 0 {
		return Undecided, false, nil
	}
	local, ok := placedLocal(p, pp.Frame, pp.Xform)
	if !ok {
		return Undecided, false, nil
	}
	lower, upper := local[2].Cmp(z0), local[2].Cmp(z1)
	if lower < 0 || upper > 0 {
		return Outside, true, nil
	}
	class, err := locateInProfile(proofbound.NewWorkBudget(ctx), pp.Profile, planePoint{u: local[0], v: local[1]})
	if err != nil {
		return Undecided, false, err
	}
	switch class {
	case regionOutside:
		return Outside, true, nil
	case regionBoundary:
		return OnBoundary, true, nil
	case regionInside:
		if lower == 0 || upper == 0 {
			return OnBoundary, true, nil
		}
		return Inside, true, nil
	default:
		return Undecided, false, nil
	}
}
