package apitest_test

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins docs/clearance-design.md §2's revolve carrier displacement: a
// revolve's clearance carriers are built from the walk's float axis
// coordinates (z, ρ), a re-expression of the recorded plane point that rounds
// and snaps a near-axis radius onto the axis, so a published Clearance row
// must enclose the gap to the RECORDED geometry swept about the recorded
// axis, not only the gap between the carriers.

// clearanceRow returns the gap of the one clearance row doc's two bodies
// publish.
func clearanceRow(t *testing.T, doc *decad.Document) decad.Measurement {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Len(t, report.Clearances, 1, "status %v", report.Status)
	return report.Clearances[0].Gap
}

// requireGapEncloses requires |value − truth| ≤ bound over the rationals.
func requireGapEncloses(t *testing.T, gap decad.Measurement, truth *big.Rat) {
	t.Helper()
	diff := new(big.Rat).Sub(ratOf(gap.Value.Mag()), truth)
	diff.Abs(diff)
	off, _ := diff.Float64()
	require.LessOrEqualf(t, diff.Cmp(ratOf(gap.Bound.Mag())), 0,
		"the gap %.20g (%v) sits %g from the true gap, outside its bound %g", gap.Value.Mag(), gap.Exactness, off, gap.Bound.Mag())
}

// TestClearanceRevolveAxisReexpressionContainsTruth revolves the rectangle
// u ∈ [0.201, 0.601], v ∈ [0, 1] about the axis line u = 1048576.3, against
// a box whose face x = 0 faces the swept profile's φ = 0 edge. The recorded
// edge sits at x = 0.201 exactly, so the true gap is that float. The carriers
// read the radius ρ = fl(a − 0.201), which rounds by about half an ulp at 2²⁰
// (1.15e-10), and every carrier and vertex at φ = 0 sits that far off the
// recorded edge.
//
// Shown to fail: before the carriers' axis gap (RevolveCarrierResult's
// AxisGap) was charged into bodyGeom.delta, both sweeps published a gap
// 1.15e-10 off the truth with a bound of about 6e-11, which is only the
// angular term. A half or full turn's angular term (about 1.3e-10 and
// 2.6e-10 here) happened to cover the same rounding before the charge, so
// neither isolates it. Dropping the charge from addRevolveFaces turns both
// subtests red again.
func TestClearanceRevolveAxisReexpressionContainsTruth(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{30, 90} {
		t.Run(fmt.Sprintf("%v°", deg), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			const a = 1048576.3
			s := rectSketch(t, 0.201, 0, 0.601, 1)
			_, err := doc.Revolve(s, s.Profiles()[0],
				decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a, V: 1}},
				decad.AngleExtent{A: units.Degrees(deg), Dir: decad.Along})
			require.NoError(t, err)
			boxBodyAtZ(t, doc, -1, -1, 0, 2, -0.25, 0.5)
			requireGapEncloses(t, clearanceRow(t, doc), ratOf(0.201))
		})
	}
}

// TestClearanceRevolveAxisCornerContainsTruth is the same revolve against a
// box whose nearest feature is the swept profile's corner (0.201, 0) at φ = 0:
// the box's edge x = 0, y = −0.05 runs past it, so the true gap is
// √(0.201² + 0.05²) and the row's interval must contain it. The corner is
// where the junction vertex and the junction arc (a Circle3/Arc3 edge the
// kernel reads through newCEdge) sit, both built from the same re-expressed
// radius the carriers read, so the charge on the carriers is what covers them.
//
// Shown to fail: before the carriers' axis gap was charged, both sweeps
// published an interval whose upper end sat about 1e-10 below the truth.
func TestClearanceRevolveAxisCornerContainsTruth(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{30, 90} {
		t.Run(fmt.Sprintf("%v°", deg), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			const a = 1048576.3
			s := rectSketch(t, 0.201, 0, 0.601, 1)
			_, err := doc.Revolve(s, s.Profiles()[0],
				decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a, V: 1}},
				decad.AngleExtent{A: units.Degrees(deg), Dir: decad.Along})
			require.NoError(t, err)
			boxBodyAtZ(t, doc, -1, -2, 0, -0.05, -0.25, 0.5)
			gap := clearanceRow(t, doc)
			// Both ends of the interval are compared squared: the truth is
			// the square root of an exact rational.
			truth2 := new(big.Rat).Add(new(big.Rat).Mul(ratOf(0.201), ratOf(0.201)), new(big.Rat).Mul(ratOf(0.05), ratOf(0.05)))
			v, b := ratOf(gap.Value.Mag()), ratOf(gap.Bound.Mag())
			hi := new(big.Rat).Add(v, b)
			lo := new(big.Rat).Sub(v, b)
			require.GreaterOrEqual(t, new(big.Rat).Mul(hi, hi).Cmp(truth2), 0, "the interval's high end must not exclude the truth")
			if lo.Sign() > 0 {
				require.LessOrEqual(t, new(big.Rat).Mul(lo, lo).Cmp(truth2), 0, "the interval's low end must not exclude the truth")
			}
		})
	}
}

// TestClearanceRevolveAxisSnapContainsTruth revolves a half disc of radius r
// whose diameter sits 5e-10·r off the u axis — inside the axis snap tolerance
// 1e-9·r — a full turn about that axis. The build snaps both diameter ends onto
// the axis and reads the arc as a sphere centred on it, so its carrier is the
// ball of radius r; the recorded profile reaches r + 5e-10·r from the axis.
// A box r/10 above the ball's top therefore sits r/10 − 5e-10·r from the
// recorded geometry, and the row must enclose that.
//
// Shown to fail: before the carriers' axis gap was charged, scale 1 published
// 0.1 with a bound of 5e-16 against a 5e-10 miss, and scale 1000 published 100
// with a bound of 5e-13 against a 5e-7 miss.
func TestClearanceRevolveAxisSnapContainsTruth(t *testing.T) {
	t.Parallel()
	for _, r := range []float64{1, 1000} {
		t.Run(fmt.Sprintf("scale=%v", r), func(t *testing.T) {
			t.Parallel()
			near := 5e-10 * r
			doc := decad.New()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			start := s.CreatePoint(-r, near)
			end := s.CreatePoint(r, near)
			c := s.CreatePoint(0, near)
			s.Fix(start)
			s.Fix(end)
			s.Fix(c)
			s.CreateLine(start, end)
			s.CreateArc(c, end, start)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			g := r / 10
			boxBodyAtZ(t, doc, -r/4, r+g, r/4, 2*r, -r/4, r/2)
			truth := new(big.Rat).Sub(ratOf(r+g), new(big.Rat).Add(ratOf(r), ratOf(near)))
			gap := clearanceRow(t, doc)
			requireGapEncloses(t, gap, truth)
			require.Equal(t, decad.Approximate, gap.Exactness)
		})
	}
}
