package revolveproof

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// ChainArcSourcePoint encloses the recorded source endpoint about the exact
// path axis, including the axis resolver's own anchor and direction bounds.
func ChainArcSourcePoint(ax revolveaxis.Frame, u, v float64, uv proofbound.WalkEndBound,
) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	widen := func(value, bound float64) (proofbound.RatInterval, bool) {
		leaf, allowance := proofarith.FloatRat(value), proofarith.FloatRat(math.Abs(bound))
		if leaf == nil || allowance == nil {
			return proofbound.RatInterval{}, false
		}
		return proofbound.IntervalWiden(proofbound.PointInterval(leaf), allowance), true
	}
	values := [6]struct{ value, bound float64 }{
		{u, uv.U}, {v, uv.V},
		{ax.AU, ax.AUBound}, {ax.AV, ax.AVBound},
		{ax.DU, ax.DUBound}, {ax.DV, ax.DVBound},
	}
	var leaves [6]proofbound.RatInterval
	for i, entry := range values {
		var ok bool
		leaves[i], ok = widen(entry.value, entry.bound)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
	}
	du := proofbound.IntervalSub(leaves[0], leaves[2])
	dv := proofbound.IntervalSub(leaves[1], leaves[3])
	z := proofbound.IntervalAdd(proofbound.IntervalMul(du, leaves[4]), proofbound.IntervalMul(dv, leaves[5]))
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, leaves[4]), proofbound.IntervalMul(du, leaves[5]))
	return z, rho, true
}

// ChainArcAreaDensityAllow bounds the change from the held line's ideal
// revolve-area Jacobian to the source line and exact path axis. The wall's
// density is L*step*rho(t), and each of its three factors gets a separate
// absolute difference term, so opposite errors cannot cancel.
func ChainArcAreaDensityAllow(z0, rho0, z1, rho1 proofbound.RatInterval,
	heldZ0, heldRho0, heldZ1, heldRho1 float64,
	span, heldStep proofbound.RatInterval, n int) (float64, error) {
	refuse := func() (float64, error) {
		return 0, fmt.Errorf(`%w: the chain arc Sweep has no enclosed area density`, decaderr.ErrUnsupported)
	}
	if n <= 0 || heldStep.Lo.Sign() <= 0 {
		return refuse()
	}
	trueDZ := proofbound.IntervalSub(z1, z0)
	trueDR := proofbound.IntervalSub(rho1, rho0)
	trueLength, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(
		proofbound.IntervalSquare(trueDZ), proofbound.IntervalSquare(trueDR)))
	if !ok {
		return refuse()
	}
	heldDZ := proofarith.FloatRat(heldZ1 - heldZ0)
	heldDR := proofarith.FloatRat(heldRho1 - heldRho0)
	heldRho0Rat := proofarith.FloatRat(heldRho0)
	heldRho1Rat := proofarith.FloatRat(heldRho1)
	if heldDZ == nil || heldDR == nil || heldRho0Rat == nil || heldRho1Rat == nil {
		return refuse()
	}
	heldLength, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(
		proofbound.IntervalSquare(proofbound.PointInterval(heldDZ)),
		proofbound.IntervalSquare(proofbound.PointInterval(heldDR))))
	if !ok {
		return refuse()
	}
	trueStep := proofbound.IntervalScale(span, new(big.Rat).SetFrac64(1, int64(n)))
	deltaStep := proofbound.IntervalAbsUpper(proofbound.IntervalSub(trueStep, heldStep))
	deltaLength := proofbound.IntervalAbsUpper(proofbound.IntervalSub(trueLength, heldLength))
	deltaRho := proofbound.RatMax(
		proofbound.IntervalAbsUpper(proofbound.IntervalSub(rho0, proofbound.PointInterval(heldRho0Rat))),
		proofbound.IntervalAbsUpper(proofbound.IntervalSub(rho1, proofbound.PointInterval(heldRho1Rat))))
	rhoUpper := proofbound.RatMax(rho0.Hi, rho1.Hi)
	first := new(big.Rat).Mul(trueLength.Hi, deltaStep)
	first.Add(first, new(big.Rat).Mul(heldStep.Hi, deltaLength))
	first.Mul(first, rhoUpper)
	second := new(big.Rat).Mul(heldLength.Hi, heldStep.Hi)
	second.Mul(second, deltaRho)
	allow := proofbound.RatFloatUp(first.Add(first, second))
	if proofbound.IsNonFinite(allow) {
		return refuse()
	}
	return allow, nil
}
