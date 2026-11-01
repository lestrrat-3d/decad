package prismplacement

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CutTranslation resolves a same-frame, same-basis translation exactly into section and sweep coordinates.
func CutTranslation(a, b Operand) (Relative, *big.Rat, bool) {
	if a.Frame != b.Frame || a.Xform.Basis() != b.Xform.Basis() {
		return Relative{}, nil, false
	}
	basis := a.Xform.Basis()
	for _, v := range []r3.Vec{basis.EX, basis.EY, basis.EZ, a.Frame.U(), a.Frame.V(), a.Frame.N(),
		a.Frame.Origin(), a.Xform.Translation(), b.Xform.Translation()} {
		if !proofbound.FiniteVec(v) {
			return Relative{}, nil, false
		}
	}
	axis := func(v r3.Vec) proofarith.DyV3 {
		return proofarith.DvSub(proofarith.DvTransform(a.Xform, proofarith.DyVec(v)),
			proofarith.DyVec(a.Xform.Translation()))
	}
	u, v, n := axis(a.Frame.U()), axis(a.Frame.V()), axis(a.Frame.N())
	d := proofarith.DvSub(proofarith.DyVec(b.Xform.Translation()), proofarith.DyVec(a.Xform.Translation()))
	det := proofarith.DvDot(u, proofarith.DvCross(v, n)).Rat()
	if det.Sign() == 0 {
		return Relative{}, nil, false
	}
	ru := new(big.Rat).Quo(proofarith.DvDot(d, proofarith.DvCross(v, n)).Rat(), det)
	rv := new(big.Rat).Quo(proofarith.DvDot(u, proofarith.DvCross(d, n)).Rat(), det)
	rz := new(big.Rat).Quo(proofarith.DvDot(u, proofarith.DvCross(v, d)).Rat(), det)
	fu, _ := ru.Float64()
	fv, _ := rv.Float64()
	if math.IsInf(fu, 0) || math.IsInf(fv, 0) {
		return Relative{}, nil, false
	}
	m, err := r3.Translation(r3.NewVec(fu, fv, 0))
	if err != nil {
		return Relative{}, nil, false
	}
	charge := proofbound.AbsSumUpper(proofarith.RationalFloatError(ru, fu), proofarith.RationalFloatError(rv, fv))
	return Relative{Map: m, TransAbs: max(math.Abs(fu), math.Abs(fv)), TransError: charge}, rz, true
}
