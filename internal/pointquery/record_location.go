package pointquery

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// LocateRecord classifies the world point p against the solid an analytic
// payload's own record denotes, in exact rational arithmetic: a straight
// prism (agent-docs/point-containment-design.md §7), a stacked prism or a
// stack BRep (§9) and a full solid Revolve (§10). Every other payload, and
// every record or point those routes leave open, reports decided false; the
// caller then classifies against the verified mesh (§2). p must be finite.
func LocateRecord(ctx context.Context, p r3.Vec, payload topo.Payload) (Location, bool, error) {
	switch pp := payload.(type) {
	case prismrecord.Payload:
		return LocatePrism(ctx, p, pp)
	case stackedrecord.Payload:
		return LocateStacked(ctx, p, pp)
	case breprecord.Payload:
		return LocateStackBrep(ctx, p, pp)
	case revolveplan.Payload:
		return LocateRevolve(ctx, p, pp)
	}
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	return Undecided, false, nil
}

// placedLocal solves a prism-family record's denoted affine map for the
// plane coordinates (u, v, z) of the world point p. The map is
// x ↦ T + B·O + L·x with L = B·[U V N] (massmoment.PrismRotation): the
// frame's held axes and origin and the placement's held basis B and
// translation T, read as exact rationals (agent-docs/evaluator-design.md
// §5.1). The held L is orthonormal only to rounding, so its transpose is not
// its inverse; Cramer's rule over exact rationals is. It reports false for a
// non-finite field or a singular L.
func placedLocal(p r3.Vec, frame r3.Frame, xform r3.Transform) (ratVec, bool) {
	l, err := massmoment.PrismRotation(frame, xform)
	if err != nil {
		return ratVec{}, false
	}
	b, err := massmoment.PlacementRotation(xform)
	if err != nil {
		return ratVec{}, false
	}
	origin, translation, point := exactVec(frame.Origin()), exactVec(xform.Translation()), exactVec(p)
	if !finiteRatVec(origin) || !finiteRatVec(translation) || !finiteRatVec(point) {
		return ratVec{}, false
	}
	var offset ratVec
	for i := range 3 {
		w := new(big.Rat).Set(translation[i])
		for k := range 3 {
			w.Add(w, new(big.Rat).Mul(b[i][k], origin[k]))
		}
		offset[i] = new(big.Rat).Sub(point[i], w)
	}
	return cramerSolve(l, offset)
}

// finiteRatVec reports that every component read as a rational.
func finiteRatVec(v ratVec) bool { return v[0] != nil && v[1] != nil && v[2] != nil }

// cramerSolve solves m·x = rhs over exact rationals. It reports false for a
// singular m.
func cramerSolve(m [3][3]*big.Rat, rhs ratVec) (ratVec, bool) {
	det := massmoment.Determinant(m)
	if det.Sign() == 0 {
		return ratVec{}, false
	}
	var out ratVec
	for k := range 3 {
		var replaced [3][3]*big.Rat
		for i := range 3 {
			for j := range 3 {
				replaced[i][j] = m[i][j]
			}
			replaced[i][k] = rhs[i]
		}
		out[k] = new(big.Rat).Quo(massmoment.Determinant(replaced), det)
	}
	return out, true
}

// regionLocation reads a plane class as a point class, for a point strictly
// between two levels.
func regionLocation(class regionClass) Location {
	switch class {
	case regionOutside:
		return Outside
	case regionInside:
		return Inside
	case regionBoundary:
		return OnBoundary
	}
	return Undecided
}
