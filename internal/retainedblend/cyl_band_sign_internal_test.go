package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The band of flatBand lies in the plane z = 0: 0 ≤ x ≤ 1 and 0 ≤ y (the end
// plane), every held vertex bounded by 0.001 and the rounding 0.001, so a point
// within 0.002 of a plane is undecided on it. A point within that of an end
// plane that no other plane drops is read by the sign of its exact offset, and
// the offset is charged into its bound (retained-edge-blend §4.3).

// charged is bound grown by the offset d, as signRead charges it with no slack.
func charged(bound, d float64) float64 { return proofbound.AbsSumUpper(bound, d) }

// TestSignReadChargesHeldVertex reads a held vertex within its bound of the end
// plane by its sign and raises its bound by the offset. Omit the charge and the
// bound assertions fail.
func TestSignReadChargesHeldVertex(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name string
		off  float64
	}{
		{"inside by 0.0015", 0.0015},
		{"outside by 0.0015", -0.0015},
		{"inside by a float step's worth", 3e-16},
	} {
		off := c.off
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			// Vertex 0 lies off the end plane y = 0 by off and inside x ∈ [0, 1]: no
			// other plane drops it, so no order of the planes decides it.
			w, b, f := flatBand(r3.NewVec(0.5, off, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
			cuts, err := w.cutBand(f, b)
			require.NoError(t, err)

			want := charged(0.001, math.Abs(off))
			require.Equal(t, want, w.beta[0], "the offset is charged into the held bound")
			require.Greater(t, w.beta[0], 0.001)
			require.Equal(t, want, w.raised, "a raised held bound is reported")
			require.Equal(t, 0.001, w.beta[1], "a vertex that no reading undecided keeps its bound")
			require.Equal(t, 0.001, w.beta[2])

			if off > 0 {
				// Inside the band on the side of its offset: the whole facet is removed.
				require.Equal(t, map[int]bool{0: true, 1: true, 2: true}, cuts.removed)
				require.Empty(t, w.out)
				return
			}
			// Outside it, on the other side: the vertex stays, and so does the sliver
			// of the facet outside y ≥ 0 that it cuts off.
			require.Equal(t, map[int]bool{1: true, 2: true}, cuts.removed)
			require.NotEmpty(t, w.out)
		})
	}
}

// TestSignReadChargesCutPoint reads a cut point within its bound of the end plane
// by its sign and charges the offset into the point's own bound: the cuts of x = 0
// and x = 1 on a facet edge 0.0015 off the end plane.
func TestSignReadChargesCutPoint(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(-0.5, 0.0015, 0), r3.NewVec(1.5, 0.0015, 0), r3.NewVec(0.5, 3, 0))
	cuts, err := w.cutBand(f, b)
	require.NoError(t, err)
	require.Equal(t, map[int]bool{2: true}, cuts.removed)

	// The cuts the plane made: facet bound 0.001 plus the rounding 0 plus the
	// grid's 0.001, then the offset.
	made := 0
	for id := 3; id < len(w.verts); id++ {
		p := w.verts[id]
		if math.Abs(p.Y-0.0015) > 1e-12 {
			continue
		}
		made++
		require.InDelta(t, 0.0035, w.beta[id], 1e-12, "the cut at x = %v", p.X)
	}
	require.Equal(t, 2, made)
	require.Equal(t, 0.001, w.beta[0], "the corners outside the band keep their bound")
	require.Equal(t, 0.001, w.beta[1])
}

// TestSignReadOnlyWhereReadingRefuses reads nothing by sign that another plane or
// the plain reading decides: a corner another plane drops, a corner clear of the
// plane and a corner on it by provenance keep their bounds, and so do the cuts
// that need no sign.
func TestSignReadOnlyWhereReadingRefuses(t *testing.T) {
	t.Parallel()

	t.Run("a corner another plane drops is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(-0.5, 0, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.6, 4, 0))
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		for id, beta := range w.beta {
			require.Equal(t, 0.001, math.Min(beta, 0.001), "vertex %d", id)
			if id < 3 {
				require.Equal(t, 0.001, beta, "held vertex %d", id)
			}
		}
		require.Zero(t, w.raised)
	})
	t.Run("a corner on the plane by provenance is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.5, 0, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		b.ends[0].on = map[int]bool{0: true}
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, []float64{0.001, 0.001, 0.001}, w.beta)
		require.Zero(t, w.raised)
	})
	t.Run("a corner clear of the plane is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.5, 0.01, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, []float64{0.001, 0.001, 0.001}, w.beta)
		require.Zero(t, w.raised)
	})
}

// TestSignReadRefusals keeps every refusal that is not an end plane's: a point
// within its bound of the main plane or of a further plane is undecided still,
// and so is any point of an angle-main band, which has no exact main plane.
func TestSignReadRefusals(t *testing.T) {
	t.Parallel()

	t.Run("a corner two planes leave undecided", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.0005, 0.0005, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a vertex of the wall lies on an end face's plane")
	})
	t.Run("a corner within its bound of the main plane", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(1.0015, 1, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Equal(t, 0.001, w.beta[0], "nothing is charged for a plane that is not an end plane's")
	})
	t.Run("a band without an exact main plane reads no sign", func(t *testing.T) {
		t.Parallel()
		w, b, _ := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		b.main = nil
		b.w = w
		b.side = make([]map[int]int, bandOther+len(b.others))
		for q := range b.side {
			b.side[q] = map[int]int{}
		}
		poly := []bandPt{w.heldPt(0), w.heldPt(1), w.heldPt(2)}
		b.sign = true
		_, err := b.clip(poly, bandEnd0, 0.001)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Equal(t, 0.001, w.beta[0])
		b.active = []int{bandEnd0}
		_, ok := b.clipBySign(poly, b.active, 0.001)
		require.Equal(t, -1, ok)
	})
}

// TestChargeShared makes a vertex that an earlier cut of the same held edge made
// carry the larger bound of the two copies of the point.
func TestChargeShared(t *testing.T) {
	t.Parallel()

	w := &work{verts: []r3.Vec{{}}, beta: []float64{0.002}}
	w.chargeShared(0, &bandPt{bound: 0.0035})
	require.Equal(t, 0.002, w.beta[0], "a point no sign read charged changes nothing")
	w.chargeShared(0, &bandPt{bound: 0.0035, signed: [2]int8{3, 0}})
	require.Equal(t, 0.0035, w.beta[0])
	w.chargeShared(0, &bandPt{bound: 0.003, signed: [2]int8{0, 1}})
	require.Equal(t, 0.0035, w.beta[0], "the bound never shrinks")
}

// TestSignReadChargesSlack charges the route's arithmetic slack with the offset,
// which covers the rounding of the offset itself.
func TestSignReadChargesSlack(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
	b.slack = 0.0005
	_, err := w.cutBand(f, b)
	require.NoError(t, err)
	require.InDelta(t, 0.001+0.0015+0.0005, w.beta[0], 1e-12)
}

// TestSignReadChargeReachesCrossing makes the cut of an edge by the end plane take
// the ends' bounds as a sign reading left them. The edge's two corners lie 0.0015
// either side of the plane and are first read by it in the crossing, after the cuts
// of x = 0 and x = 1 made points on the edge that are read first.
func TestSignReadChargeReachesCrossing(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(-0.5, -0.0015, 0), r3.NewVec(1.5, 0.0015, 0), r3.NewVec(0.5, 3, 0))
	_, err := w.cutBand(f, b)
	require.NoError(t, err)

	// The corners' bounds are raised by the offset, and the cut on y = 0 takes the
	// raised bound with the grid's 0.001.
	require.InDelta(t, 0.0025, w.beta[0], 1e-12)
	require.InDelta(t, 0.0025, w.beta[1], 1e-12)
	found := false
	for id := 3; id < len(w.verts); id++ {
		if p := w.verts[id]; math.Abs(p.Y) > 1e-12 || math.Abs(p.X-0.5) > 1e-12 {
			continue
		}
		found = true
		require.InDelta(t, 0.0035, w.beta[id], 1e-12, "the cut of the edge by the end plane")
	}
	require.True(t, found)
}
