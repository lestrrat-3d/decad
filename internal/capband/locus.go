package capband

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// MiterLocusSubdivisions bounds the work and enclosure width for a mitered
// corner's offset path. Thirty-two ranges enclose a quarter-disk chamfer near
// its line-circle tangency without swamping its held chord.
const MiterLocusSubdivisions = 32

// MiterLocusUpper bounds the length of the conic corner-foot path from the
// original corner to the cap-level foot. Each range charges one work step.
// axialUpper encloses the stated side setback, not the rounded level gap.
// A false result means that no finite enclosure could be built.
func MiterLocusUpper(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk,
	apexU, apexV, axialUpper, dc, dcDelta float64) (float64, bool, error) {
	if dc <= 0 || !(axialUpper >= 0) || proofbound.IsNonFinite(axialUpper) {
		return 0, false, nil
	}
	// The offset span includes the setback's unit-conversion rounding.
	// Its lower endpoint gives a conservative axial rate.
	span, rate := dc, dc
	if dcDelta > 0 {
		span = proofbound.UpRound(dc + dcDelta)
		rate = freeform.DownRound(dc - dcDelta)
		if rate <= 0 {
			return 0, false, nil
		}
	}
	total := 0.0
	for _, r := range MiterLocusRanges(span) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		t0, t1 := r[0], r[1]
		speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, t0, t1, apexU, apexV)
		if !ok {
			return 0, false, nil
		}
		// Width encloses the sub-range's exact width. The axial rise is
		// rounded up at the conservative rate over the same sub-range.
		width := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(t1), proofarith.FloatRat(t0)))
		axial := proofbound.DivUpper(proofbound.ProductUpper(axialUpper, width), rate)
		total = proofbound.AbsSumUpper(total, proofbound.ChordLocusLengthAllow(speed, width, axial, 0))
	}
	return total, true, nil
}

// MiterLocusRanges tiles [0, span] using shared float endpoints. The last
// endpoint is span, so rounding cannot leave a gap between ranges.
func MiterLocusRanges(span float64) [MiterLocusSubdivisions][2]float64 {
	var ranges [MiterLocusSubdivisions][2]float64
	step := span / MiterLocusSubdivisions
	t0 := 0.0
	for k := range MiterLocusSubdivisions {
		t1 := float64(k+1) * step
		if k == MiterLocusSubdivisions-1 {
			t1 = span
		}
		ranges[k] = [2]float64{t0, t1}
		t0 = t1
	}
	return ranges
}
