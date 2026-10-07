package prismcells

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func ratPoint(u, v float64) [2]*big.Rat {
	return [2]*big.Rat{new(big.Rat).SetFloat64(u), new(big.Rat).SetFloat64(v)}
}

// TestSinLower pins sinLower against crossings whose sine is a rational
// closed form, and its refusal of a ball wider than half a circle's radius.
// Each bound must sit at or below the true sine over the ball it reads.
// Shown to fail with the line/circle rho term deleted ("line and circle over
// a ball" read 0.6, above the 0.5 the ball reaches) and with the
// half-radius check deleted ("a ball wider than half the radius" read a
// positive bound).
func TestSinLower(t *testing.T) {
	t.Parallel()
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
	}
	circle := func(cu, cv, r float64) CurveSegment {
		return CircleSeg{Center: sectionrecord.Point2{U: cu, V: cv}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	}
	for _, tc := range []struct {
		name       string
		a, b       CurveSegment
		pt         [2]*big.Rat
		rho        float64
		want, slop float64
	}{
		// Directions (1, 0) and (3, 4): sine 4/5.
		{"two lines", line(0, 0, 10, 0), line(0, 0, 3, 4), ratPoint(0, 0), 0, 0.8, 1e-15},
		// The circle's radius at (3, 4) is (3, 4), its tangent (−4, 3): against
		// (1, 0) the sine is 3/5.
		{"line and circle", line(-10, 4, 10, 4), circle(0, 0, 5), ratPoint(3, 4), 0, 0.6, 1e-15},
		// Over a ball of radius 0.5 the bound is (3 − 0.5)/5.
		{"line and circle over a ball", line(-10, 4, 10, 4), circle(0, 0, 5), ratPoint(3, 4), 0.5, 0.5, 1e-15},
		// Radii (3, 4) and (−3, 4): |cross| = 24, sine 24/25.
		{"two circles", circle(0, 0, 5), circle(6, 0, 5), ratPoint(3, 4), 0, 0.96, 1e-15},
		{"an arc and a line", ArcSeg{
			Center: sectionrecord.Point2{U: 0, V: 0}, Start: sectionrecord.Point2{U: 5, V: 0},
			End: sectionrecord.Point2{U: 0, V: 5}, TStart: 0, TEnd: 1,
		}, line(-10, 4, 10, 4), ratPoint(3, 4), 0, 0.6, 1e-15},
		{"a ball wider than half the radius", line(-10, 4, 10, 4), circle(0, 0, 5), ratPoint(3, 4), 2.6, 0, 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := sinLower(tc.a, tc.b, tc.pt, tc.rho)
			require.NoError(t, err)
			require.LessOrEqual(t, got, tc.want, "a lower bound never exceeds the true sine")
			require.GreaterOrEqual(t, got, tc.want-tc.slop)
		})
	}
}

// crossingScene arranges a 10×10 square A and a band B whose lower edge
// runs from (−5, 10 − 10·slope) to (15, 10 + 10·slope), so it crosses A's
// top edge at (5, 10) with the sine slope/√(1 + slope²), and A's left wall
// nearly square.
func crossingScene(t *testing.T, slope float64) ([]*sketch.Profile, map[sketch.Entity]Origin) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	tags := map[sketch.Entity]Origin{}
	poly := func(isB bool, pts ...[2]float64) {
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(p[0], p[1])
			s.Fix(ps[i])
		}
		for i := range ps {
			tags[s.CreateLine(ps[i], ps[(i+1)%len(ps)])] = Origin{IsB: isB, Hole: -1}
		}
	}
	poly(false, [2]float64{0, 0}, [2]float64{10, 0}, [2]float64{10, 10}, [2]float64{0, 10})
	poly(true, [2]float64{-5, 10 - 10*slope}, [2]float64{15, 10 + 10*slope}, [2]float64{15, 12 + 10*slope}, [2]float64{-5, 12 - 10*slope})
	_, err = s.Solve(context.Background())
	require.NoError(t, err)
	return s.Profiles(), tags
}

// TestCrossingChargeSumsBothDisplacements pins the charge at a crossing of
// sine about 0.0995 when both operands bring the same displacement δ: the
// crossing can move by (δ + δ)/sin θ, and the charge must cover it. Shown to
// fail with the charge taking max(δ1, δ2) in place of δ1 + δ2 (it read about
// 11δ, under the 20δ the crossing can move).
func TestCrossingChargeSumsBothDisplacements(t *testing.T) {
	t.Parallel()
	const slope, delta = 0.1, 1e-12
	profiles, tags := crossingScene(t, slope)
	charge, ok, err := CrossingCharge(proofbound.NewWorkBudget(t.Context()), tags, profiles, delta, delta)
	require.NoError(t, err)
	require.True(t, ok)
	sinUpper := slope / math.Sqrt(1+slope*slope) * (1 + 1e-12)
	require.GreaterOrEqual(t, charge, 2*delta/sinUpper)
	require.LessOrEqual(t, charge, 2*delta/(sinUpper*(1-1e-9))+1.01*delta)

	one, ok, err := CrossingCharge(proofbound.NewWorkBudget(t.Context()), tags, profiles, delta, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, one, delta/sinUpper)
	require.Less(t, one, charge)
}

// TestCrossingChargeDeclinesNearTangent pins A6's noise floor: a proven
// sine at the floor ε has no charge and one above it does. sketch already
// declines to certify a line/line cut whose sine is below about 1e-8
// (TExact = false, which RecordEdge refuses and CrossingCharge declines), so
// the floor is reached through a tangent junction, whose sine bound is zero;
// the shallowest certified crossing, a sine of about 1e-6, is charged its
// amplified displacement instead. Shown to fail with the floor in
// aboveNoiseFloor deleted.
func TestCrossingChargeDeclinesNearTangent(t *testing.T) {
	t.Parallel()
	require.False(t, aboveNoiseFloor(sectionaudit.ContactEps))
	require.False(t, aboveNoiseFloor(0))
	require.True(t, aboveNoiseFloor(2*sectionaudit.ContactEps))

	const slope, delta = 1e-6, 1e-12
	profiles, tags := crossingScene(t, slope)
	charge, ok, err := CrossingCharge(proofbound.NewWorkBudget(t.Context()), tags, profiles, 0, delta)
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, charge, delta/(slope*(1+1e-9)))

	none, ok, err := CrossingCharge(proofbound.NewWorkBudget(t.Context()), tags, profiles, 0, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Zero(t, none, "no displacement, no charge")

	uncertified, tagsU := crossingScene(t, 1e-10)
	_, ok, err = CrossingCharge(proofbound.NewWorkBudget(t.Context()), tagsU, uncertified, 0, delta)
	require.NoError(t, err)
	require.False(t, ok, "an uncertified cut has no charge, and the caller falls back")
}

// TestCrossingChargeOutsideTouchIsNoCut pins the premise of crossing.go's
// outside-touch argument: a wedge B whose apex touches A's top wall at
// (5, 10) from outside, and a triangle whose edge passes exactly through A's
// corner from outside, are each arranged by sketch as two separate cells
// with no cut and no shared vertex, so there is no crossing to charge, and
// the charge is zero. A displaced apex can still dip into A, but every point
// it moves lies within δ_B of A's recorded wall, inside the section
// displacement's own tube. If sketch ever cuts at an outside touch, the
// split assertion below fails, and CrossingCharge's vertex reading then
// charges the pair like any other cut.
func TestCrossingChargeOutsideTouchIsNoCut(t *testing.T) {
	t.Parallel()
	for _, wedge := range [][][2]float64{
		{{5, 10}, {15, 11}, {15, 12}},
		{{0, 11}, {20, 9}, {20, 15}},
	} {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		tags := map[sketch.Entity]Origin{}
		poly := func(isB bool, pts ...[2]float64) {
			ps := make([]*sketch.Point, len(pts))
			for i, p := range pts {
				ps[i] = s.CreatePoint(p[0], p[1])
				s.Fix(ps[i])
			}
			for i := range ps {
				tags[s.CreateLine(ps[i], ps[(i+1)%len(ps)])] = Origin{IsB: isB, Hole: -1}
			}
		}
		poly(false, [2]float64{0, 0}, [2]float64{10, 0}, [2]float64{10, 10}, [2]float64{0, 10})
		poly(true, wedge...)
		_, err = s.Solve(context.Background())
		require.NoError(t, err)
		profiles := s.Profiles()

		split, err := HasSplitBoundary(proofbound.NewWorkBudget(t.Context()), profiles)
		require.NoError(t, err)
		require.False(t, split, "an outside touch must not be arranged as a cut")
		require.Len(t, profiles, 2)
		for _, p := range profiles {
			first := tags[p.Outer[0].Entity].IsB
			for _, e := range p.Outer {
				require.Equal(t, first, tags[e.Entity].IsB, "each cell is one operand's own outline")
			}
		}
		charge, ok, err := CrossingCharge(proofbound.NewWorkBudget(t.Context()), tags, profiles, 1e-12, 1e-12)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, charge)
	}
}
