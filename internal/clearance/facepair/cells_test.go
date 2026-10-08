package facepair_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/facepair"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// These tests pin docs/clearance-design.md §4's windowed nested cell on
// face pairs built by hand. Legs seen red, each by breaking what it guards:
// deleting the cell leaves every nested pair of the band test undecided;
// reading d_sup at the window's midpoint instead of its ends puts the tilt
// test's lower bound above the true minimum; and admitting a lower bound at
// or below tol answers the filled bore.

// cylinderFace is a full cylinder face of radius r about the line through
// anchor along the unit axis, trimmed to the axial window [z0, z1].
func cylinderFace(anchor, axis r3.Vec, r, z0, z1 float64) *clearance.CFace {
	u := clearance.PerpTo(axis)
	f := &clearance.CFace{
		Kind:   clearance.CkCylinder,
		Anchor: anchor,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Sweep:  clearance.AngWindow{Full: true},
		ZWin:   clearance.NewLinWindow(z0, z1),
	}
	lo, hi := anchor.Add(axis.Scale(z0)), anchor.Add(axis.Scale(z1))
	f.Box = clearance.BoxUnion(clearance.CircleBox(lo, axis, r), clearance.CircleBox(hi, axis, r))
	f.Wit = []r3.Vec{lo.Add(u.Scale(r))}
	return f
}

// sphereFace is a whole sphere of radius r about centre.
func sphereFace(centre r3.Vec, r float64) *clearance.CFace {
	axis := r3.NewVec(0, 0, 1)
	u := clearance.PerpTo(axis)
	return &clearance.CFace{
		Kind:   clearance.CkSphere,
		Anchor: centre,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Sweep:  clearance.AngWindow{Full: true},
		Merid:  clearance.AngWindow{Full: true},
		Box:    [2]r3.Vec{centre.Sub(r3.NewVec(r, r, r)), centre.Add(r3.NewVec(r, r, r))},
		Wit:    []r3.Vec{centre.Add(u.Scale(r))},
	}
}

// faceCell runs one face pair through the §4 table at the tolerance of a pair
// whose coordinates reach scale.
func faceCell(t *testing.T, f, g *clearance.CFace, scale float64) (*clearance.CellSink, float64) {
	t.Helper()
	tol := 1e-9 * scale
	k := facepair.New(t.Context(), tol, tol)
	sink := &clearance.CellSink{}
	k.FaceCell(f, g, sink)
	require.NoError(t, k.Err())
	return sink, tol
}

// TestWindowedNestedBand is the band test: a nested pair whose spines sit a
// rounding error apart, where the oracle can prove neither a ring family nor
// a crossing. A pin of radius 5 over z ∈ [−1, 9] stands in a bore of radius
// 5.5 over z ∈ [0, 8], its axis 1e-14 off the bore's; the same offset nests a
// ball in a bore, a short pin in a ball and a ball in a ball, and a pin
// tilted 1e-12 rad, inside the parallel oracle's undecided band, takes the
// cell from the line pair that has no critical. Each reads its lower bound
// r_g − r_f − d_sup within 2·tol, at or below the true minimum, and an upper
// bound within 2·tol of that minimum, never Exact, in both orders: the
// annular 0.5 for every pair but the pin in the ball, whose window ends sit
// 1 mm from the ball's centre, so d_sup = 1 and the bound reads 2.5 against a
// true 5.5 − √5. A pin that fills its bore reads undecided.
func TestWindowedNestedBand(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	bore := cylinderFace(r3.NewVec(48, 0, 0), z, 5.5, 0, 8)
	off := r3.NewVec(48, 1e-14, -1)
	tilted, ok := r3.NewVec(0, -1e-12, 1).Normalize()
	require.True(t, ok)
	for _, c := range []struct {
		name    string
		inner   *clearance.CFace
		outer   *clearance.CFace
		lo, gap float64
	}{
		{"pin in bore", cylinderFace(off, z, 5, 0, 10), bore, 0.5, 0.5},
		{"tilted pin in bore", cylinderFace(r3.NewVec(48, 0, 4), tilted, 5, -5, 5), bore, 0.5, 0.5},
		{"ball in bore", sphereFace(r3.NewVec(48, 1e-14, 4), 5), bore, 0.5, 0.5},
		{"pin in ball", cylinderFace(r3.NewVec(48, 1e-14, 0), z, 2, -1, 1), sphereFace(r3.NewVec(48, 0, 0), 5.5),
			2.5, 5.5 - math.Sqrt(5)},
		{"ball in ball", sphereFace(r3.NewVec(48, 1e-14, 4), 5), sphereFace(r3.NewVec(48, 0, 4), 5.5), 0.5, 0.5},
	} {
		for _, swap := range []bool{false, true} {
			f, g := c.inner, c.outer
			if swap {
				f, g = g, f
			}
			name := c.name
			if swap {
				name += " swapped"
			}
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				sink, tol := faceCell(t, f, g, 62)
				require.False(t, sink.Unsure, `the nested pair is decided`)
				require.False(t, sink.Overlap)
				lo, hi, exact, ok := sink.Interval()
				require.True(t, ok)
				require.False(t, exact, `a windowed reading is never Exact`)
				require.InDelta(t, c.lo, lo, 2*tol)
				require.InDelta(t, c.gap, hi, 2*tol)
				require.LessOrEqual(t, lo, c.gap)
				require.GreaterOrEqual(t, hi, c.gap)
			})
		}
	}
	t.Run("filled bore", func(t *testing.T) {
		t.Parallel()
		sink, _ := faceCell(t, cylinderFace(off, z, 5.5, 0, 10), bore, 62)
		require.True(t, sink.Unsure, `a pin that fills its bore is a contact §6 does not certify`)
	})
}

// TestWindowedNestedTilt is the tilt test: a pin of radius 5 and half-height
// 5, turned α = 1e-6 rad about X through its centre at the origin, in a bore
// of radius 5.5 about Z. The pin's window end sits 5·sin α off the bore's
// axis, so its farthest point reaches 5·(sin α + cos α) from it and the true
// minimum is 5.5 − 5·(sin α + cos α) ≈ 0.5 − 5e-6. The lower bound reads d_sup
// off the window's ends, 5·sin α, and sits at or below that minimum; the row
// reads within 1e-7 of it.
func TestWindowedNestedTilt(t *testing.T) {
	t.Parallel()
	const alpha = 1e-6
	axis := r3.NewVec(0, -math.Sin(alpha), math.Cos(alpha))
	pin := cylinderFace(r3.Vec{}, axis, 5, -5, 5)
	bore := cylinderFace(r3.NewVec(0, 0, -8), r3.NewVec(0, 0, 1), 5.5, 0, 16)
	trueMin := 5.5 - 5*(math.Sin(alpha)+math.Cos(alpha))
	sink, _ := faceCell(t, pin, bore, 16)
	require.False(t, sink.Unsure)
	require.False(t, sink.Overlap)
	lo, hi, exact, ok := sink.Interval()
	require.True(t, ok)
	require.False(t, exact)
	require.LessOrEqual(t, lo, trueMin, `the lower bound never overstates the minimum`)
	require.GreaterOrEqual(t, hi, trueMin)
	require.InDelta(t, 0.5-5e-6, (lo+hi)/2, 1e-7)
}

// TestWindowedNestedKeepsTheCoarseEnclosure pins that the windowed cell
// tightens the coarse enclosure of a line pair with no critical rather than
// replacing it. A pin of radius 1 over z ∈ [20, 22], its axis 1e-12 rad off
// the bore's, stands above a bore of radius 5.5 over z ∈ [0, 8]: the windowed
// cell proves only 5.5 − 1 and admits no witness, while the faces' boxes lie
// 12 apart and the coarse witnesses bound the pair above. Seen red: the
// windowed bound alone reads [4.5, +Inf).
func TestWindowedNestedKeepsTheCoarseEnclosure(t *testing.T) {
	t.Parallel()
	tilted, ok := r3.NewVec(0, -1e-12, 1).Normalize()
	require.True(t, ok)
	pin := cylinderFace(r3.Vec{}, tilted, 1, 20, 22)
	bore := cylinderFace(r3.Vec{}, r3.NewVec(0, 0, 1), 5.5, 0, 8)
	sink, _ := faceCell(t, pin, bore, 22)
	require.False(t, sink.Unsure)
	lo, hi, _, ok := sink.Interval()
	require.True(t, ok, `the coarse witnesses bound the pair above`)
	// The boxes lie 12 apart; the coarse lower bound reads that less the
	// charge for the boxes' own float corners (clearance.CellSink.Coarse).
	require.LessOrEqual(t, lo, 12.0)
	require.InDelta(t, 12.0, lo, 1e-9)
	require.False(t, math.IsInf(hi, 1))
	require.GreaterOrEqual(t, hi, lo)
}
