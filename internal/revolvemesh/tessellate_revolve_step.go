//go:build race

package revolvemesh

import "github.com/lestrrat-3d/decad/internal/proof"

// RevDown is proof.NextDown(x), bit for bit, and RevUp is proof.NextUp(x).
// In the race build each steps an x of magnitude at least 2^-969 away from
// zero itself, small enough to inline: that covers almost every end the
// pre-test rounds. Every other x, NaN included, calls proof. The plain build's
// version is in tessellate_revolve_step_plain.go.
//
// One product and one sum give the neighbour with no tie to test. Write
// x = m·2^E with m in [1, 2), ulp = 2^(E−52) and c = (1 + 2^-52)·2^-53. The
// product x·c is normal because x ≥ 2^-969, and it rounds to r = q·ulp with q
// strictly between 1/2 and 3/2:
//
//   - at m = 1 the product (1 + 2^-52)·2^(E−53) is exact, so q = (1 + 2^-52)/2;
//   - at m ≥ 1 + 2^-52, q ≥ (1 + 2^-52)²(1 − 2^-53)/2, which exceeds 1/2;
//   - at m ≤ 2 − 2^-52, q ≤ (2 − 2^-52)(1 + 2^-52)(1 + 2^-53)/2, below 3/2.
//
// So x + r lies strictly between x + ulp/2 and x + 3ulp/2 and rounds to
// x + ulp, also at the top of a binade, where the spacing above x + ulp
// doubles. At x = MaxFloat64 the sum passes the MaxFloat64 + ulp/2 overflow
// threshold and rounds to +Inf, which is NextUp's answer there, and an
// infinite x returns itself, as NextUp's does. A negative x mirrors the
// argument with every sign flipped. The float64() conversion forbids fusing
// the product into the sum.
func RevDown(x float64) float64 {
	if x <= -0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextDown(x)
}

func RevUp(x float64) float64 {
	if x >= 0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextUp(x)
}
