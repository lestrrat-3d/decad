package verifyrun

import (
	"math/big"
	"testing"

	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func testBox(lo, hi r3.Vec, bound float64) measurement.Box {
	return measurement.Box{Min: lo, Max: hi, Bound: units.Millimeters(bound)}
}

func testShift(x, y, z float64) shift {
	return shift{v: &[3]*big.Rat{ratOf(x), ratOf(y), ratOf(z)}}
}

// TestBoxesDisjointShifted holds the shifted box test to the unshifted one: it
// agrees with measurement.BoxesDisjoint at zero shifts, boxes whose inflated
// ends touch are disjoint, and boxes that overlap by one float step are not,
// with and without shifts.
func TestBoxesDisjointShifted(t *testing.T) {
	t.Parallel()
	unit := testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0)
	for _, tc := range []struct {
		name   string
		b      measurement.Box
		sa, sb shift
		apart  bool
	}{
		{"zero shifts, apart", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, shift{}, true},
		{"zero shifts, touching", testBox(r3.Vec{X: 1}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, true},
		{"zero shifts, one step of overlap", testBox(r3.Vec{X: 1 - 0x1p-53}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, false},
		{"zero shifts, overlapping", testBox(r3.Vec{X: 0.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, false},
		{"a bound reaches into the other box", testBox(r3.Vec{X: 1.25}, r3.Vec{X: 2, Y: 1, Z: 1}, 0.5), shift{}, shift{}, false},
		{"bounds just touch", testBox(r3.Vec{X: 1.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0.5), shift{}, shift{}, true},
		{"shift moves b clear", testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0), shift{}, testShift(1, 0, 0), true},
		{"shift moves b into a", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1.5, 0, 0), false},
		{"shift moves a clear, other axis", testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0), testShift(0, 0, -1), shift{}, true},
		{"shifts equal, boxes overlapping", testBox(r3.Vec{X: 0.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), testShift(5, 5, 5), testShift(5, 5, 5), false},
		{"shift leaves one step of overlap", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1-0x1p-52, 0, 0), false},
		{"shift leaves exact touching", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1, 0, 0), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.apart, boxesDisjointShifted(unit, tc.sa, tc.b, tc.sb))
			// Disjointness is symmetric.
			require.Equal(t, tc.apart, boxesDisjointShifted(tc.b, tc.sb, unit, tc.sa))
			if !tc.sa.equal(tc.sb) {
				return
			}
			// Equal shifts move both boxes alike, so they agree with the raw test.
			require.Equal(t, measurement.BoxesDisjoint(unit, tc.b), boxesDisjointShifted(unit, tc.sa, tc.b, tc.sb))
		})
	}
}

// rectPrism is a prism of the rectangle [0, w]×[0, h] on frame f, levels 0
// to depth, placed by xf.
func rectPrism(t *testing.T, f r3.Frame, xf r3.Transform, w, h, depth float64) *topo.Body {
	t.Helper()
	pts := []sectionrecord.Point2{{U: 0, V: 0}, {U: w, V: 0}, {U: w, V: h}, {U: 0, V: h}}
	var segs []sectionrecord.CurveSegment
	for i := range pts {
		segs = append(segs, sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1})
	}
	return &topo.Body{Payload: prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Frame:   f, Z0: 0, Z1: depth, Xform: xf,
	}}
}

// dyadic rounds x to a multiple of 1/1024, so sums of such values stay exact.
func dyadic(x float64) float64 { return math.Round(x*1024) / 1024 }

// TestPlanesApartShiftMatchesMovedBody holds the shifted end-cap and slab
// separators to the unshifted ones on a body really moved: a block on an
// inexact lift and a small box whose frame origin moves by d, every sum a
// multiple of 1/1024 and so exact. Reading the box under shift d must give the
// same answer as moving it by d, for each rule, both argument orders, across a
// sweep of d that crosses the block's top plane.
//
// Leg shown to fail: adding the shift with the wrong sign, or reading the
// first body as the moved one, disagrees with the moved box in the sweep.
func TestPlanesApartShiftMatchesMovedBody(t *testing.T) {
	t.Parallel()
	lift, err := r3.RotationAround(r3.Vec{X: 3, Y: -2, Z: 1}, r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.4))
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	block := rectPrism(t, xy, lift, 4, 2, 1)
	box := func(o r3.Vec) *topo.Body {
		f, err := r3.NewFrame(o, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		return rectPrism(t, f, r3.Identity(), 0.25, 0.25, 0.25)
	}
	rules := []struct {
		name    string
		shifted func(a, b *topo.Body, d [3]*big.Rat) bool
		moved   func(a, b *topo.Body) bool
	}{
		{"slab", sideApartBy, sideApart},
		{"end cap", capApartBy, capApart},
		{"both", planesApart, func(a, b *topo.Body) bool { return planesApart(a, b, zeroVec()) }},
	}
	// The box sits 0.1 above the block's top and 0.1 below its bottom, read
	// along the lift's local z, so both planes of each pair decide somewhere.
	for _, local := range []r3.Vec{{X: 2, Y: 1, Z: 1.1}, {X: 2, Y: 1, Z: -0.35}} {
		for _, dir := range []r3.Vec{{Z: 1}, {X: 1, Y: -1, Z: 1}} {
			at := lift.Apply(local)
			origin := r3.Vec{X: dyadic(at.X), Y: dyadic(at.Y), Z: dyadic(at.Z)}
			seen := map[bool]struct{}{}
			for k := -32; k <= 32; k++ {
				step := float64(k) / 128
				d := r3.Vec{X: dir.X * step, Y: dir.Y * step, Z: dir.Z * step}
				dv := ratVec(d)
				back := scaleVec(dv, big.NewRat(-1, 1))
				still, moved := box(origin), box(origin.Add(d))
				for _, rule := range rules {
					want := rule.moved(block, moved)
					require.Equal(t, want, rule.shifted(block, still, dv), "%s, block first, d = %v", rule.name, d)
					require.Equal(t, rule.moved(moved, block), rule.shifted(still, block, back), "%s, box first, d = %v", rule.name, d)
					if rule.name == "both" {
						seen[want] = struct{}{}
					}
				}
			}
			require.Len(t, seen, 2, "the sweep from %v along %v crosses a plane of the block", local, dir)
		}
	}
}
