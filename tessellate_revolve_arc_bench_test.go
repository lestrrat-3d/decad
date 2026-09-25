package decad

import (
	"math"
	"math/big"
	"testing"
)

// BenchmarkRevolveArcCellSlack measures the certified slack calculation for
// the inner-torus arc cell from TestRevolveArcCellSlackBoundsASignChangingJacobianGap.
func BenchmarkRevolveArcCellSlack(b *testing.B) {
	cell := arcCellFixture(10, 3, 3*math.Pi/2-0.4, 0.8)
	step := intervalScale(twoPiInterval(), big.NewRat(1, 16))
	// This representative twice-area lies between the inner-point and endpoint
	// model densities, so the Jacobian error changes sign within the cell.
	heldTwiceArea := pointInterval(big.NewRat(0, 1).SetFloat64(6.67))
	twoArea := [2]ratInterval{heldTwiceArea, heldTwiceArea}

	b.ReportAllocs()
	b.ResetTimer()
	var result float64
	for b.Loop() {
		var err error
		result, err = revolveArcCellSlack(cell, step, twoArea, 0)
		if err != nil {
			b.Fatal(err)
		}
		if result <= 0 || math.IsNaN(result) || math.IsInf(result, 0) {
			b.Fatalf("expected finite positive slack, got %g", result)
		}
	}
	b.ReportMetric(result, "slack")
}
