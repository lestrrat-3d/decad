package pointquery

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
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
	local, ok := prismLocal(p, pp)
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

// prismLocal solves the record's denoted affine map for the plane coordinates
// (u, v, z) of the world point p. The map is x ↦ T + B·O + L·x with
// L = B·[U V N] (massmoment.PrismRotation): the frame's held axes and origin
// and the placement's held basis B and translation T, read as exact
// rationals (agent-docs/evaluator-design.md §5.1). The held L is orthonormal
// only to rounding, so its transpose is not its inverse; Cramer's rule over
// exact rationals is. It reports false for a non-finite field or a singular L.
func prismLocal(p r3.Vec, pp prismrecord.Payload) (ratVec, bool) {
	l, err := massmoment.PrismRotation(pp.Frame, pp.Xform)
	if err != nil {
		return ratVec{}, false
	}
	b, err := massmoment.PlacementRotation(pp.Xform)
	if err != nil {
		return ratVec{}, false
	}
	origin, translation, point := exactVec(pp.Frame.Origin()), exactVec(pp.Xform.Translation()), exactVec(p)
	for i := range 3 {
		if origin[i] == nil || translation[i] == nil || point[i] == nil {
			return ratVec{}, false
		}
	}
	var offset ratVec
	for i := range 3 {
		w := new(big.Rat).Set(translation[i])
		for k := range 3 {
			w.Add(w, new(big.Rat).Mul(b[i][k], origin[k]))
		}
		offset[i] = new(big.Rat).Sub(point[i], w)
	}
	det := massmoment.Determinant(l)
	if det.Sign() == 0 {
		return ratVec{}, false
	}
	var local ratVec
	for k := range 3 {
		var replaced [3][3]*big.Rat
		for i := range 3 {
			for j := range 3 {
				replaced[i][j] = l[i][j]
			}
			replaced[i][k] = offset[i]
		}
		local[k] = new(big.Rat).Quo(massmoment.Determinant(replaced), det)
	}
	return local, true
}
