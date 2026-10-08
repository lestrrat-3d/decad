package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// patternPrism extrudes the rectangle [u0, u1]×[v0, v1] by 5, or a circle of
// radius r about (u0, v0) when r > 0.
func patternPrism(t *testing.T, u0, v0, u1, v1, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	if r > 0 {
		c := s.CreatePoint(u0, v0)
		s.Fix(c)
		s.CreateCircle(c, r)
	} else {
		rect := s.CreateRectangle(u0, v0, u1, v1)
		s.Fix(rect.A)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return body
}

func patternInstance(t *testing.T, b *Body) prismPayload {
	t.Helper()
	pp, ok := b.payload.(prismPayload)
	require.True(t, ok)
	return pp
}

// TestPatternCopiesChargesTheMotion is §8's δ_pattern: an integer-millimetre
// step along +x and the quarter turns of Count = 4 on integer coordinates
// publish exactly zero and keep the receiver's own placement; a 0.3 in step
// publishes at least the conversion's own rounding; the six-pin circle
// publishes a displacement that covers each recorded centre's distance from
// (20 cos 60i°, 20 sin 60i°), with √3 taken to 512 bits, and zero on its
// half turn.
//
// Legs shown to fail (each broken in pattern.go, the fixture watched go red,
// then restored): the point charge (every instance then publishes zero, and
// the inch and pin rows miss), and the exact quarter turns (Count = 4 then
// reads certified trig and publishes a nonzero charge).
func TestPatternCopiesChargesTheMotion(t *testing.T) {
	t.Parallel()
	t.Run("integer step", func(t *testing.T) {
		t.Parallel()
		peg := patternPrism(t, 0, 0, 5, 5, 0)
		copies, err := peg.PatternCopies(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 4})
		require.NoError(t, err)
		for _, c := range copies {
			pp := patternInstance(t, c)
			require.Zero(t, pp.sectionDelta)
			require.Equal(t, r3.Identity(), pp.xform, "a frame-keeping instance keeps the receiver's placement")
		}
	})
	t.Run("along the sweep", func(t *testing.T) {
		t.Parallel()
		peg := patternPrism(t, 0, 0, 5, 5, 0)
		copies, err := peg.PatternCopies(t.Context(), LinearPattern{Dir: r3.NewVec(0, 0, 1), Step: units.Millimeters(10), Count: 2})
		require.NoError(t, err)
		require.NotEqual(t, r3.Identity(), patternInstance(t, copies[0]).xform, "a PlacedCopy instance carries the motion in its placement")
	})
	t.Run("quarter turns", func(t *testing.T) {
		t.Parallel()
		block := patternPrism(t, 18, -1, 22, 1, 0)
		copies, err := block.PatternCopies(t.Context(), CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 4})
		require.NoError(t, err)
		for _, c := range copies {
			require.Zero(t, patternInstance(t, c).sectionDelta)
		}
	})
	t.Run("inch step", func(t *testing.T) {
		t.Parallel()
		peg := patternPrism(t, 0, 0, 5, 5, 0)
		step := units.Inches(0.3)
		copies, err := peg.PatternCopies(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: step, Count: 2})
		require.NoError(t, err)
		held, err := step.In(units.Millimeter)
		require.NoError(t, err)
		conversion := extent.ConversionRound(step, units.Millimeter, held)
		require.Positive(t, conversion, "the premise: 0.3 in does not convert to a float")
		require.GreaterOrEqual(t, patternInstance(t, copies[0]).sectionDelta, conversion)
	})
	t.Run("six pins", func(t *testing.T) {
		t.Parallel()
		pin := patternPrism(t, 20, 0, 0, 0, 2)
		copies, err := pin.PatternCopies(t.Context(), CircularPattern{Axis: r3.NewVec(0, 0, 1), Count: 6})
		require.NoError(t, err)
		root3 := new(big.Float).SetPrec(512).Sqrt(new(big.Float).SetPrec(512).SetInt64(3))
		ten3 := new(big.Float).SetPrec(512).Mul(root3, new(big.Float).SetPrec(512).SetInt64(10))
		f := func(v float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(v) }
		neg := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(512).Neg(x) }
		wants := [][2]*big.Float{{f(10), ten3}, {f(-10), ten3}, {f(-20), f(0)}, {f(-10), neg(ten3)}, {f(10), neg(ten3)}}
		for i, c := range copies {
			pp := patternInstance(t, c)
			if i+1 == 3 {
				require.Zero(t, pp.sectionDelta, "the half turn is exact")
			} else {
				require.Positive(t, pp.sectionDelta, "60° turns read certified trig")
			}
			circle, ok := pp.profile.Outer.Segments[0].(CircleSeg)
			require.True(t, ok)
			du := new(big.Float).SetPrec(512).Sub(f(circle.Center.U), wants[i][0])
			dv := new(big.Float).SetPrec(512).Sub(f(circle.Center.V), wants[i][1])
			sq := new(big.Float).SetPrec(512).Add(new(big.Float).Mul(du, du), new(big.Float).Mul(dv, dv))
			allow := f(pp.sectionDelta)
			require.LessOrEqual(t, sq.Cmp(allow.Mul(allow, allow)), 0, "instance %d centre %v", i+1, circle.Center)
		}
	})
}

// TestPatternLinearMotionUsesTheDenotedStep pins the linear offset's own
// arithmetic: along a 3-4-5 direction the certified 1/|Dir| is exact, and a
// 10 mm step moves a point by exactly (6, 8).
func TestPatternLinearMotionUsesTheDenotedStep(t *testing.T) {
	t.Parallel()
	rp, err := resolvePattern(LinearPattern{Dir: r3.NewVec(3, 4, 0), Step: units.Millimeters(10), Count: 2})
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	mv, err := rp.linearMotion(frame, r3.Identity(), 1)
	require.NoError(t, err)
	p, charge, err := mv(Point2{U: 1, V: 2})
	require.NoError(t, err)
	require.Equal(t, Point2{U: 7, V: 10}, p)
	require.Zero(t, charge)
	require.Zero(t, proofarith.RationalFloatError(big.NewRat(7, 1), p.U))
}

// TestPatternCopiesUnionStackArm pins which arm a union-built stack takes:
// an exact integer step keeps the frame and the receiver's placement, and a
// rounding 0.3 in step copies through PlacedCopy, whose placement carries the
// motion.
func TestPatternCopiesUnionStackArm(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		Name string
		Step units.Value
		Keep bool
	}{
		{Name: "integer step", Step: units.Millimeters(20), Keep: true},
		{Name: "inch step", Step: units.Inches(0.3)},
	} {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			doc := New()
			box := func(plane *sketch.Plane, x0, y0, x1, y1 float64) *Body {
				s, err := w.CreateSketch(plane)
				require.NoError(t, err)
				rect := s.CreateRectangle(x0, y0, x1, y1)
				s.Fix(rect.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				b, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
				require.NoError(t, err)
				return b
			}
			top, err := w.CreateOffsetPlane(w.XY(), 5)
			require.NoError(t, err)
			part, err := Union(t.Context(), box(w.XY(), 0, 0, 10, 10), box(top, 3, 3, 7, 7))
			require.NoError(t, err)
			sp, ok := part.payload.(stackedPrismPayload)
			require.True(t, ok)
			runs, err := sp.outerRuns()
			require.NoError(t, err)
			require.Len(t, runs, 2, "the premise: the outer loop changes between slabs")
			copies, err := part.PatternCopies(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: tc.Step, Count: 2})
			require.NoError(t, err)
			inst, ok := copies[0].payload.(stackedPrismPayload)
			require.True(t, ok)
			if tc.Keep {
				require.Equal(t, r3.Identity(), inst.xform)
				require.Zero(t, inst.sectionDelta)
				return
			}
			require.NotEqual(t, r3.Identity(), inst.xform)
		})
	}
}
