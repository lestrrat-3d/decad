package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// cornerAngleLower is a proven LOWER bound on the exact angle about (cU, cV)
// between the directions to side and capEnd, read from the low end of the
// Atan2Interval enclosure of their cross and dot products. It is zero where the
// enclosure reaches zero.
func cornerAngleLower(cU, cV float64, side, capEnd Point2) float64 {
	diff := func(a, b float64) *big.Rat { return new(big.Rat).Sub(proofarith.FloatRat(a), proofarith.FloatRat(b)) }
	au, av := diff(side.U, cU), diff(side.V, cV)
	bu, bv := diff(capEnd.U, cU), diff(capEnd.V, cV)
	cross := new(big.Rat).Sub(new(big.Rat).Mul(au, bv), new(big.Rat).Mul(av, bu))
	dot := new(big.Rat).Add(new(big.Rat).Mul(au, bu), new(big.Rat).Mul(av, bv))
	if cross.Sign() == 0 {
		return 0
	}
	angle := proofbound.Atan2Interval(cross, dot, false)
	if angle.Lo.Sign() != angle.Hi.Sign() {
		return 0
	}
	lo, hi := new(big.Rat).Abs(angle.Lo), new(big.Rat).Abs(angle.Hi)
	if hi.Cmp(lo) < 0 {
		lo = hi
	}
	return proofbound.RatFloatDown(lo)
}

// TestCapPatchWindowSkewCoversTheExactCornerAngle checks a mitered circular
// patch's published window skew, which the mesh's skewGap and the band
// volume's chord-versus-locus term both read, is at least the exact angle
// between each corner's side directrix end and cap directrix end. The rows
// are sectors drawn away from the sketch origin where the larger of the two
// held window differences, each a difference of two float Atan2 readings,
// falls below the larger exact corner angle (120 of 568 patches in a
// 600-sector sweep did, by up to 9.1e-15 of the angle).
//
// Shown to fail: with capPatchWindowSkew reading the two held windows'
// difference again, every row fails.
func TestCapPatchWindowSkewCoversTheExactCornerAngle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		cx, cy, r, phi, d float64
	}{
		{7407.402, -1975.3, 7, 5.8743333333333334, 0.67307692307692302},
		{7407.402, -2962.95, 4, 5.7390000000000008, 0.29230769230769232},
		{4938.268, -3950.6, 8, 5.9903333333333331, 0.58461538461538465},
	} {
		name := fmt.Sprintf(`sector r=%g phi=%g at (%g, %g)`, tc.r, tc.phi, tc.cx, tc.cy)
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			_, cbp := chamferedSectionBody(t, sectorSection(tc.cx, tc.cy, tc.r, tc.phi), tc.d)
			budget := proofbound.NewWorkBudget(t.Context())
			cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], freeform.NewFreeformWork())
			require.NoError(t, err)
			joins, err := capOffsetJoins(budget, cl, cbp.loopOffset(0))
			require.NoError(t, err)

			var skew float64
			found := 0
			for _, p := range cbp.patches {
				if p.geom.Circular && p.geom.SideRadius != 0 {
					skew = capPatchWindowSkew(p.geom)
					found++
				}
			}
			require.Equal(t, 1, found, `a sector has one circular wall`)

			heldMax, lowerMax := 0.0, 0.0
			n := len(cl.walks)
			for i, w := range cl.walks {
				if !w.IsCircular() {
					continue
				}
				start, end := capWallFoot(joins, i, n)
				capTh0, capTh1, _ := capWallSweep(w.CU, w.CV, start, end, w.Th1-w.Th0)
				c0, c1 := capband.WindowOnBranch(capTh0, capTh1, w.Th0)
				for _, corner := range []struct {
					held      float64
					side, cap Point2
				}{
					{math.Abs(c0 - w.Th0), Point2{U: w.StartU, V: w.StartV}, start},
					{math.Abs(w.Th1 - c1), Point2{U: w.EndU, V: w.EndV}, end},
				} {
					lower := cornerAngleLower(w.CU, w.CV, corner.side, corner.cap)
					require.Positive(t, lower, `the corner is a genuine miter`)
					heldMax, lowerMax = math.Max(heldMax, corner.held), math.Max(lowerMax, lower)
					require.GreaterOrEqual(t, skew, lower,
						`the published skew %v covers the exact corner angle, at least %v`, skew, lower)
				}
			}
			require.Less(t, heldMax, lowerMax, `the premise: the larger held window difference is below the larger exact corner angle`)
		})
	}
}

// TestCapWallArcBoundChargesTheRoundedOffsetRadius checks a circular wall's
// cap-level arc charges its sweep times the gap between the held offset radius
// and the exact R − dc, for a setback stated in millimetres. The quarter disk
// of radius 10 chamfered 0.1 mm holds R − dc rounded, so the gap is positive.
// The turn term moves the arc's ends along the held circle and does not cover
// a radius that is off along the whole sweep.
//
// Shown to fail: with buildCapBand passing dcDelta (zero here) as the radial
// shift again, the published bound equals the bound with no radial term.
func TestCapWallArcBoundChargesTheRoundedOffsetRadius(t *testing.T) {
	t.Parallel()
	const dc = 0.1
	chamfered, cbp := chamferedSectionBody(t, quarterDiskSection(10), dc)
	setback := cbp.loopSetback(0)
	require.Zero(t, setback.dcDelta, `a setback stated in millimetres converts exactly`)
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, dc)
	require.NoError(t, err)
	delta, err := capContourDelta(cl.walks, joins, dc, 0)
	require.NoError(t, err)

	n := len(cl.walks)
	checked := 0
	for i, w := range cl.walks {
		if !w.IsCircular() {
			continue
		}
		start, end := capWallFoot(joins, i, n)
		capRadius, err := capBandRadius(w, dc)
		require.NoError(t, err)
		radius, ok := capOffsetRadiusSpan(w, dc, 0)
		require.True(t, ok)
		gap := proofbound.IntervalFloatError(radius, capRadius)
		require.Positive(t, gap, `the premise: R − dc is not a float64`)
		capTh0, capTh1, wraps := capWallSweep(w.CU, w.CV, start, end, w.Th1-w.Th0)
		sweep := capTh1 - capTh0
		withoutRadial := capcontour.CapWallArcBound(w.CU, w.CV, start, end, capRadius, capRadius*sweep, wraps, delta, 0)

		var arc *Edge
		for _, e := range chamfered.Edges() {
			if c, ok := e.curve.(Arc3); ok && c.Radius.Mag() == capRadius {
				arc = e
			}
		}
		require.NotNil(t, arc, `the band's cap-level arc is an edge of the body`)
		require.Equal(t, math.Abs(capRadius*sweep), arc.length)
		require.GreaterOrEqual(t, arc.lengthBound, withoutRadial+math.Abs(sweep)*gap*(1-1e-9),
			`the arc's bound %v carries its sweep %v times the radius gap %v on top of %v`,
			arc.lengthBound, sweep, gap, withoutRadial)
		checked++
	}
	require.Equal(t, 1, checked)
}

// TestCapMiterLocusRangesTileTheOffsetSpan checks the miter locus's
// sub-ranges cover [0, span] end to end for setbacks where two separately
// rounded ends, k·step + step and (k+1)·step, differ. Those setbacks are where
// a range built from each end separately leaves a one-ulp offset range that no
// speed enclosure covers.
//
// Shown to fail: building range k as [k·step, k·step + step] again leaves a
// gap between two ranges on every row.
func TestCapMiterLocusRangesTileTheOffsetSpan(t *testing.T) {
	t.Parallel()
	for _, span := range []float64{8.705582497752681, 0.1, 2.981098884280702, 4.928686} {
		step := span / capMiterLocusSubdivisions
		separate := false
		for k := range capMiterLocusSubdivisions - 1 {
			if float64(k)*step+step != float64(k+1)*step {
				separate = true
			}
		}
		require.True(t, separate, `the premise: span %v rounds the two ends apart somewhere`, span)

		ranges := capMiterLocusRanges(span)
		require.Zero(t, ranges[0][0])
		require.Equal(t, span, ranges[capMiterLocusSubdivisions-1][1])
		for k, r := range ranges {
			require.Less(t, r[0], r[1], `span %v: sub-range %d runs forward`, span, k)
			if k > 0 {
				require.Equal(t, ranges[k-1][1], r[0], `span %v: sub-range %d starts where %d ends`, span, k, k-1)
			}
		}
	}
}

// TestCapMiterLocusUpperReadsTheStatedAxialRise checks the miter locus bound
// at a quarter disk's line-circle corner is never below the straight distance
// between the locus's two ends — the original corner and the cap-level foot,
// axialUpper apart along the sweep — over axial rises from far below to far
// above the in-plane setback. A curve is never shorter than its chord.
func TestCapMiterLocusUpperReadsTheStatedAxialRise(t *testing.T) {
	t.Parallel()
	const dc = 2
	_, cbp := chamferedSectionBody(t, quarterDiskSection(10), dc)
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := oneLoopCornerLoop(budget, cbp.loops()[0], freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := capOffsetJoins(budget, cl, dc)
	require.NoError(t, err)
	n := len(cl.walks)
	checked := 0
	for i, j := range joins {
		prev, cur := cl.walks[(i+n-1)%n], cl.walks[i]
		if j.arc || j.g1 || (!prev.IsCircular() && !cur.IsCircular()) {
			continue
		}
		for _, axial := range []float64{1e-3, 0.3, 2, 50, 1e4} {
			total, ok, err := capMiterLocusUpper(budget, prev, cur, j.vU, j.vV, axial, dc, 0)
			require.NoError(t, err)
			require.True(t, ok)
			chordSq := proofarith.RatSquaredDistance3(j.m.U, j.m.V, axial, j.vU, j.vV, 0)
			rt := proofarith.FloatRat(total)
			require.GreaterOrEqual(t, new(big.Rat).Mul(rt, rt).Cmp(chordSq), 0,
				`corner %d, axial rise %v: the locus bound %v is below its own chord`, i, axial, total)
		}
		checked++
	}
	require.Equal(t, 2, checked, `a quarter disk has two line-circle miters`)
}
