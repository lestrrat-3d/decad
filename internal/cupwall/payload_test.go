package cupwall_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestViewDenotedLevels assembles a cup over a receiver on [-0.2 in, 20]
// shelled 0.3 in thick, opened at either end in either sense, and reads the
// three levels' denotations from ViewFor and again from the stored record's
// View. With r = d(0.2 in) and T = d(0.3 in), the open end and the kept end
// keep the receiver's -r or 20, and the derived floor moves the kept end by T
// into the part inward or out of it outward.
func TestViewDenotedLevels(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	mm := func(v units.Value) (float64, float64, *big.Rat) {
		held, delta, err := extent.MagnitudeInBounded(v, units.Length, units.Millimeter, "the level")
		require.NoError(t, err)
		require.Positive(t, delta, "%v converts with a rounding", v)
		return held, delta, extent.ExactConversion(v, units.Millimeter)
	}
	bottom, bottomDelta, r := mm(units.Inches(0.2))
	tmm, tDelta, thickness := mm(units.Inches(0.3))
	minusR := new(big.Rat).Neg(r)
	twenty := big.NewRat(20, 1)
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: -bottom, Z1: 20, Z0Delta: bottomDelta,
		Z0Denoted: prismplacement.DenoteLevel(minusR, -bottom, bottomDelta),
		Profile:   momentinput.Profile{Outer: capShellLoop(0, 0, 100, 60)}}
	inner := momentinput.Profile{Outer: capShellLoop(5, 5, 95, 55)}
	outer := momentinput.Profile{Outer: capShellLoop(-5, -5, 105, 65)}
	plus := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	minus := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	for _, tc := range []struct {
		name               string
		s                  float64
		removedEnd         bool
		open, outerZ, cavZ *big.Rat
	}{
		{"inward open at the top", 1, true, twenty, minusR, plus(minusR, thickness)},
		{"outward open at the top", -1, true, twenty, minus(minusR, thickness), minusR},
		{"inward open at the bottom", 1, false, minusR, twenty, minus(twenty, thickness)},
		{"outward open at the bottom", -1, false, minusR, plus(twenty, thickness), twenty},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			offset := inner
			if tc.s < 0 {
				offset = outer
			}
			in := prismshell.CapShellInput{Prism: pp, KeptCaps: 1, Sense: tc.s, Thickness: units.Inches(0.3),
				HeldThickness: tmm, ThicknessDelta: tDelta}
			view := cupwall.ViewFor(in, offset, 0, tc.removedEnd)
			cup, err := view.Payload(t.Context())
			require.NoError(t, err)
			for _, v := range []cupwall.View{view, cup.View()} {
				requireDenotes(t, tc.open, prismplacement.DenotedLevel(v.ZOpen, v.ZOpenDelta, v.ZOpenDenoted), "open end")
				requireDenotes(t, tc.outerZ, prismplacement.DenotedLevel(v.ZOuter, v.ZOuterDelta, v.ZOuterDenoted), "outer floor")
				requireDenotes(t, tc.cavZ, prismplacement.DenotedLevel(v.ZCav, v.ZCavDelta, v.ZCavDenoted), "cavity floor")
			}
			// The interface's two sides record the one cavity floor level.
			floor, wall := cup.Stack.Slabs[0], cup.Stack.Slabs[1]
			if !tc.removedEnd {
				wall, floor = floor, wall
			}
			lower, upper := floor.DenotedZ1(), wall.DenotedZ0()
			if !tc.removedEnd {
				lower, upper = wall.DenotedZ1(), floor.DenotedZ0()
			}
			requireDenotes(t, tc.cavZ, lower, "the interface's lower side")
			requireDenotes(t, tc.cavZ, upper, "the interface's upper side")
		})
	}
}

// requireDenotes requires got to be the rational want.
func requireDenotes(t *testing.T, want, got *big.Rat, what string) {
	t.Helper()
	require.NotNil(t, got, "%s records a denotation", what)
	require.Zero(t, want.Cmp(got), "%s denotes %s, not %s", what, want.RatString(), got.RatString())
}
