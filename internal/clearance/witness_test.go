package clearance_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ratSq is x² over the rationals.
func ratSq(x float64) *big.Rat {
	r := new(big.Rat).SetFloat64(x)
	return r.Mul(r, r)
}

// TestWitnessGap pins the gap a face's witness carries into the coarse
// enclosure (clearance.Witness): the exact distance from the float point to
// its carrier when the trim admits the point's foot with margin, and no gap
// at all on the trim's own boundary. Each offset witness sits a known dyadic
// distance d off its carrier, so its gap must be at least d (compared by
// squares) and within an ulp-sized slack of it.
//
// Legs seen red: returning a zero gap fails every offset case, and accepting
// a foot the trim classifies as ambiguous gives every boundary witness a
// finite gap.
func TestWitnessGap(t *testing.T) {
	t.Parallel()
	const margin = 1e-9
	d := math.Ldexp(1, -30)
	z := r3.NewVec(0, 0, 1)
	cylinder := &clearance.CFace{
		Kind:   clearance.CkCylinder,
		Axis:   z,
		RefU:   r3.NewVec(1, 0, 0),
		RefV:   r3.NewVec(0, 1, 0),
		Radius: 1,
		Sweep:  clearance.NewAngWindow(-math.Pi/2, math.Pi/2),
		ZWin:   clearance.NewLinWindow(0, 2),
	}
	// The cone ρ = z about the z axis from the origin.
	cone := &clearance.CFace{
		Kind:  clearance.CkCone,
		Axis:  z,
		RefU:  r3.NewVec(1, 0, 0),
		RefV:  r3.NewVec(0, 1, 0),
		Rise:  1,
		Run:   1,
		Sweep: clearance.AngWindow{Full: true},
		ZWin:  clearance.NewLinWindow(0, 2),
	}
	for _, c := range []struct {
		name string
		face *clearance.CFace
		w    r3.Vec
		// wantSq is the square of the witness's exact distance from its
		// carrier; nil names a witness on the trim's boundary, with no gap.
		wantSq *big.Rat
	}{
		{"cylinder on its wall", cylinder, r3.NewVec(1, 0, 1), new(big.Rat)},
		{"cylinder off its wall", cylinder, r3.NewVec(1+d, 0, 1), ratSq(d)},
		{"cylinder at its window end", cylinder, r3.NewVec(1, 0, 0), nil},
		{"cylinder at its sweep end", cylinder, r3.NewVec(0, 1, 1), nil},
		{"sphere off its surface", sphereFace(r3.NewVec(3, 0, 0), 2), r3.NewVec(5+d, 0, 0), ratSq(d)},
		// (1 + d, 0, 1 − d) is |ρ − z|/√2 = √2·d off the cone.
		{"cone off its wall", cone, r3.NewVec(1+d, 0, 1-d), new(big.Rat).Mul(big.NewRat(2, 1), ratSq(d))},
		{"cone at its window end", cone, r3.NewVec(2, 0, 2), nil},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			gap := c.face.WitnessGap(c.w, margin)
			if c.wantSq == nil {
				require.True(t, math.IsInf(gap, 1), "a witness on the trim's boundary carries no gap, read %g", gap)
				return
			}
			require.GreaterOrEqual(t, ratSq(gap).Cmp(c.wantSq), 0, "the gap %g holds the witness's distance", gap)
			want, _ := c.wantSq.Float64()
			require.InDelta(t, math.Sqrt(want), gap, 1e-15)
		})
	}
}

// TestEdgeWitnesses pins an edge's witnesses: a segment's ends with no gap
// and its midpoint with its exact distance from the segment's line, and an
// arc's mid-angle point with its exact distance from the circle, while the
// arc's start, on its own window's end, is never read. Seen red: accepting an
// ambiguous angle keeps the arc's start.
func TestEdgeWitnesses(t *testing.T) {
	t.Parallel()
	const margin = 1e-9
	seg := &clearance.CEdge{Line: true, A: r3.NewVec(0, 0, 0), B: r3.NewVec(1, 3, 0)}
	ws := seg.Witnesses(margin)
	require.Len(t, ws, 3)
	require.Equal(t, []clearance.Witness{
		{P: seg.A},
		{P: seg.B},
		// The midpoint is exact, so it lies on the segment.
		{P: r3.NewVec(0.5, 1.5, 0)},
	}, ws)

	arc := &clearance.CEdge{
		Center: r3.NewVec(2, 0, 0),
		Axis:   r3.NewVec(0, 0, 1),
		RefU:   r3.NewVec(1, 0, 0),
		RefV:   r3.NewVec(0, 1, 0),
		Radius: 3,
		Ang:    clearance.NewAngWindow(0, 1),
	}
	ws = arc.Witnesses(margin)
	require.Len(t, ws, 1, "the arc's start sits on its window's end")
	// The mid-angle point is a float sample in the arc's plane, |ρ − 3| off
	// the circle: the gap must hold (3 − gap)² ≤ ρ² ≤ (3 + gap)².
	p, gap := ws[0].P, ws[0].Gap
	require.Zero(t, p.Z)
	dx := new(big.Rat).Sub(new(big.Rat).SetFloat64(p.X), big.NewRat(2, 1))
	rho2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), ratSq(p.Y))
	lo := new(big.Rat).Sub(big.NewRat(3, 1), new(big.Rat).SetFloat64(gap))
	hi := new(big.Rat).Add(big.NewRat(3, 1), new(big.Rat).SetFloat64(gap))
	require.LessOrEqual(t, new(big.Rat).Mul(lo, lo).Cmp(rho2), 0)
	require.GreaterOrEqual(t, new(big.Rat).Mul(hi, hi).Cmp(rho2), 0)
	require.Less(t, gap, 1e-14)
}
