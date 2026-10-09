package spherepath

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PairTransferStatus names the first bound that could not be proved.
type PairTransferStatus uint8

const (
	PairTransferOK PairTransferStatus = iota
	PairTransferPayloadUnsupported
	PairTransferNoNormalProof
	PairTransferPointTooCoarse
)

// PairTransferInput holds the exact path and the rounded contact readings.
// PointResolution is the exact base-unit value of the caller's request.
type PairTransferInput struct {
	Motion               PairMotion
	ObservedA, ObservedB proof.DyV3
	Fraction             *big.Rat
	IdealNormal          r3.Vec
	ObservedNormal       r3.Vec
	IdealNormalBound     float64
	ObservedNormalBound  float64
	ObservedNormalAngle  float64
	WitnessBounds        [2]float64
	SeparationBound      float64
	NormalResolution     float64
	PointResolution      *big.Rat
}

// PairTransferBounds are the outward readings for a float manifold on the
// exact affine path. NormalChanged leaves an unchanged exact normal untouched.
type PairTransferBounds struct {
	NormalBound, NormalAngle float64
	WitnessBounds            [2]float64
	SeparationBound          float64
	NormalChanged            bool
}

// TransferPairBounds charges rounded pose and normal differences to both
// witnesses and the separation reading before publishing a sphere-pair point.
func TransferPairBounds(in PairTransferInput) (PairTransferBounds, PairTransferStatus) {
	var out PairTransferBounds
	m := in.Motion
	deviation := new(big.Rat)
	for _, moving := range []struct {
		start, observed proof.DyV3
		delta           [3]proof.Dyadic
	}{{m.CenterA, in.ObservedA, m.DeltaA}, {m.CenterB, in.ObservedB, m.DeltaB}} {
		for axis := range 3 {
			center := new(big.Rat).Add(moving.start[axis].Rat(),
				new(big.Rat).Mul(moving.delta[axis].Rat(), in.Fraction))
			diff := new(big.Rat).Sub(moving.observed[axis].Rat(), center)
			deviation.Add(deviation, diff.Abs(diff))
		}
	}
	fraction, ok := proof.DyOfRat(in.Fraction)
	if !ok {
		return out, PairTransferPayloadUnsupported
	}
	idealA, idealB := m.CenterA, m.CenterB
	for axis := range 3 {
		idealA[axis] = proof.DyAdd(idealA[axis], proof.DyMul(m.DeltaA[axis], fraction))
		idealB[axis] = proof.DyAdd(idealB[axis], proof.DyMul(m.DeltaB[axis], fraction))
	}
	minimumDistance := math.Inf(1)
	for _, pair := range [][2]proof.DyV3{{idealA, idealB}, {in.ObservedA, in.ObservedB}} {
		squared := proof.DyZero()
		for axis := range 3 {
			delta := proof.DySubScalar(pair[1][axis], pair[0][axis])
			squared = proof.DyAdd(squared, proof.DyMul(delta, delta))
		}
		minimumDistance = math.Min(minimumDistance, proof.DySqrtDown(squared))
	}
	if minimumDistance <= 0 || !finitePairTransfer(minimumDistance) {
		return out, PairTransferNoNormalProof
	}
	// Unit-vector normalization changes by at most twice the center-line
	// displacement divided by the shorter center-line length.
	normalMotion := 0.0
	if deviation.Sign() > 0 {
		normalMotion = proofbound.ProvenUpRound(2 * proofbound.RatFloatUp(deviation) / minimumDistance)
	}
	if in.ObservedNormalBound == 0 && in.IdealNormalBound == 0 &&
		in.ObservedNormal == in.IdealNormal && CardinalNormal(in.ObservedNormal) {
		normalMotion = 0
	}
	normalDifference := math.Hypot(in.ObservedNormal.X-in.IdealNormal.X,
		math.Hypot(in.ObservedNormal.Y-in.IdealNormal.Y, in.ObservedNormal.Z-in.IdealNormal.Z))
	if !finitePairTransfer(normalMotion, normalDifference) ||
		normalDifference > proofbound.ProvenUpRound(in.ObservedNormalBound+in.IdealNormalBound+normalMotion) {
		return out, PairTransferNoNormalProof
	}
	out.NormalBound, out.NormalAngle = in.ObservedNormalBound, in.ObservedNormalAngle
	if normalMotion > 0 {
		out.NormalBound = proofbound.ProvenUpRound(out.NormalBound + normalMotion)
		out.NormalAngle = proofbound.ProvenUpRound(out.NormalAngle + 4*normalMotion)
		out.NormalChanged = true
	}
	if out.NormalBound > in.NormalResolution || out.NormalAngle > in.NormalResolution {
		return out, PairTransferNoNormalProof
	}
	for i, radius := range [2]proof.Dyadic{m.RadiusA, m.RadiusB} {
		bound := new(big.Rat).Add(proof.FloatRat(in.WitnessBounds[i]), deviation)
		if normalMotion > 0 {
			bound.Add(bound, proof.FloatRat(proofbound.ProvenUpRound(proofbound.RatFloatUp(radius.Rat())*normalMotion)))
		}
		if bound.Cmp(in.PointResolution) > 0 {
			return out, PairTransferPointTooCoarse
		}
		out.WitnessBounds[i] = proofbound.RatFloatUp(bound)
	}
	sepBound := new(big.Rat).Add(proof.FloatRat(in.SeparationBound), deviation)
	if normalMotion > 0 {
		sepBound.Add(sepBound, proof.FloatRat(proofbound.ProvenUpRound(
			proofbound.RatFloatUp(proof.DyAdd(m.RadiusA, m.RadiusB).Rat())*normalMotion)))
	}
	out.SeparationBound = proofbound.RatFloatUp(sepBound)
	return out, PairTransferOK
}

func finitePairTransfer(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
