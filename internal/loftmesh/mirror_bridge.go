package loftmesh

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// NearAxialPlacement reports whether motion is a pure rotation about Z whose
// Z axis image is the float just below one, with no translation.
func NearAxialPlacement(motion r3.Transform) bool {
	if !motion.IsValid() || motion.IsReflection() ||
		motion.Translation() != (r3.Vec{}) {
		return false
	}
	b := motion.Basis()
	return b.EZ == (r3.Vec{Z: math.Nextafter(1, 0)}) &&
		b.EX.Z == 0 && b.EY.Z == 0 && b.EX.X == b.EY.Y &&
		b.EX.Y == -b.EY.X
}

// MirrorUpperBridgeShift returns the constant Z translation that separates
// the reflected lower-half Join from the actual upper copy.
//
// For a lower source L reflected about z=middle, the actual upper copy is
// T(F(L)). The reflected lower-half Join represents F(T(L)). Their exact
// affine source maps differ only by the constant translation
// 2*middle*(1-T.EZ.Z) in Z. It reports false when motion is not a near-axial
// placement or the shift is not a positive finite number.
func MirrorUpperBridgeShift(motion r3.Transform, middle float64) (float64, bool) {
	if !NearAxialPlacement(motion) ||
		middle <= 0 || proofbound.IsNonFinite(middle) {
		return 0, false
	}
	gap := new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(motion.Basis().EZ.Z))
	shift := proofbound.RatFloatUp(new(big.Rat).Mul(
		new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(middle)), gap))
	if shift <= 0 || proofbound.IsNonFinite(shift) {
		return 0, false
	}
	return shift, true
}

// MirrorUpperBridgeSwept returns the volume swept by translating a source of
// the given certified area (value and bound) along shift. The pair's
// certified area bounds the upper source area along this translation, so the
// swept volume covers the symmetric difference. It reports false when the
// area or the swept volume is not finite.
func MirrorUpperBridgeSwept(shift, area, areaBound float64) (float64, bool) {
	areaUpper := proofbound.AbsSumUpper(area, areaBound)
	swept := proofbound.SweptVolumeAllow(shift, areaUpper)
	if proofbound.IsNonFinite(areaUpper) || proofbound.IsNonFinite(swept) {
		return 0, false
	}
	return swept, true
}
