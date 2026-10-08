package curvecells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvecells"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// windowedCylinder is a full cylinder face of radius r about the line through
// anchor along the unit axis, trimmed to the axial window [z0, z1].
func windowedCylinder(anchor, axis r3.Vec, r, z0, z1 float64) *clearance.CFace {
	u := clearance.PerpTo(axis)
	lo, hi := anchor.Add(axis.Scale(z0)), anchor.Add(axis.Scale(z1))
	return &clearance.CFace{
		Kind:   clearance.CkCylinder,
		Anchor: anchor,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Sweep:  clearance.AngWindow{Full: true},
		ZWin:   clearance.NewLinWindow(z0, z1),
		Box:    clearance.BoxUnion(clearance.CircleBox(lo, axis, r), clearance.CircleBox(hi, axis, r)),
		Wit:    []r3.Vec{lo.Add(u.Scale(r))},
	}
}

// windowedRim is a complete circle edge of radius r about centre in the plane
// normal to Z.
func windowedRim(centre r3.Vec, r float64) *clearance.CEdge {
	axis := r3.NewVec(0, 0, 1)
	u := clearance.PerpTo(axis)
	return &clearance.CEdge{
		Center: centre,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: r,
		Ang:    clearance.AngWindow{Full: true},
		Box:    clearance.CircleBox(centre, axis, r),
	}
}

// TestWindowedCircleEdge pins the edge tier's twin of docs/clearance-design.md
// §4's windowed nested cell: a circular edge against a sphere or cylinder face
// whose coaxiality the oracle cannot decide. A bore's rim of radius 5.5 sits
// 1e-14 off a pin's axis outside a pin of radius 5; a pin's rim of radius 5
// sits as far off inside a bore of radius 5.5; the bore's rim rings a ball of
// radius 5 whose centre sits 1e-14 off the rim's axis; and a pin tilted
// 1e-12 rad, inside the parallel oracle's undecided band, stands inside the
// bore's rim. Each reads its 0.5 mm gap within 2·tol, never Exact, and
// decided. A rim that matches the pin's radius stays at the coarse
// enclosure's zero lower bound. Seen red: deleting the reading leaves every
// decided case at that zero.
func TestWindowedCircleEdge(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	tilted, ok := r3.NewVec(0, -1e-12, 1).Normalize()
	require.True(t, ok)
	ball := &clearance.CFace{
		Kind:   clearance.CkSphere,
		Anchor: r3.NewVec(48, 1e-14, 0),
		Axis:   z,
		RefU:   clearance.PerpTo(z),
		RefV:   z.Cross(clearance.PerpTo(z)),
		Radius: 5,
		Sweep:  clearance.AngWindow{Full: true},
		Merid:  clearance.AngWindow{Full: true},
		Box:    [2]r3.Vec{r3.NewVec(43, -5, -5), r3.NewVec(53, 5, 5)},
		Wit:    []r3.Vec{r3.NewVec(53, 1e-14, 0)},
	}
	for _, c := range []struct {
		name string
		face *clearance.CFace
		edge *clearance.CEdge
		want float64
	}{
		{"bore rim outside a pin", windowedCylinder(r3.NewVec(48, 0, -1), z, 5, 0, 10), windowedRim(r3.NewVec(48, 1e-14, 0), 5.5), 0.5},
		{"pin rim inside a bore", windowedCylinder(r3.NewVec(48, 0, 0), z, 5.5, 0, 8), windowedRim(r3.NewVec(48, 1e-14, 4), 5), 0.5},
		{"bore rim around a ball", ball, windowedRim(r3.NewVec(48, 0, 0), 5.5), 0.5},
		{"tilted pin inside a bore rim", windowedCylinder(r3.NewVec(48, 0, 0), tilted, 5, -4, 4), windowedRim(r3.NewVec(48, 0, 0), 5.5), 0.5},
		{"rim on the pin", windowedCylinder(r3.NewVec(48, 0, -1), z, 5, 0, 10), windowedRim(r3.NewVec(48, 1e-14, 0), 5), 0},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			tol := 1e-9 * 62
			k := curvecells.New(t.Context(), tol, tol)
			sink := &clearance.CellSink{}
			k.FaceEdge(c.face, c.edge, sink)
			require.NoError(t, k.Err())
			require.False(t, sink.Unsure)
			lo, hi, exact, ok := sink.Interval()
			require.True(t, ok)
			require.False(t, exact)
			require.InDelta(t, c.want, lo, 2*tol)
			require.LessOrEqual(t, lo, c.want)
			if c.want > 0 {
				require.InDelta(t, c.want, hi, 2*tol)
				require.GreaterOrEqual(t, hi, c.want)
			}
		})
	}
}

// TestWindowedCircleKeepsTheCoarseEnclosure pins that the windowed reading
// tightens the coarse enclosure rather than replacing it. A rim of radius 30
// about (20, 0, 20), its axis 1e-12 rad off Z, rings a pin of radius 5 over
// z ∈ [0, 8] about Z: the band proves only 30 − 20 − 5 and the pin's trim
// admits no witness at the rim's height, while the two boxes lie 12 apart and
// the coarse witnesses bound the pair above. Seen red: the windowed bound
// alone reads [5, +Inf).
func TestWindowedCircleKeepsTheCoarseEnclosure(t *testing.T) {
	t.Parallel()
	axis, ok := r3.NewVec(0, -1e-12, 1).Normalize()
	require.True(t, ok)
	u := clearance.PerpTo(axis)
	centre := r3.NewVec(20, 0, 20)
	rim := &clearance.CEdge{
		Center: centre,
		Axis:   axis,
		RefU:   u,
		RefV:   axis.Cross(u),
		Radius: 30,
		Ang:    clearance.AngWindow{Full: true},
		Box:    clearance.CircleBox(centre, axis, 30),
	}
	pin := windowedCylinder(r3.Vec{}, r3.NewVec(0, 0, 1), 5, 0, 8)
	tol := 1e-9 * 50
	k := curvecells.New(t.Context(), tol, tol)
	sink := &clearance.CellSink{}
	k.FaceEdge(pin, rim, sink)
	require.NoError(t, k.Err())
	require.False(t, sink.Unsure)
	lo, hi, _, ok := sink.Interval()
	require.True(t, ok, `the coarse witnesses bound the pair above`)
	// The boxes lie 12 apart; the coarse lower bound reads that less the
	// charge for the boxes' own float corners (clearance.CellSink.Coarse).
	require.LessOrEqual(t, lo, 12.0)
	require.InDelta(t, 12.0, lo, tol)
	require.False(t, math.IsInf(hi, 1))
	require.GreaterOrEqual(t, hi, lo)
}
