package prismcells

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func rgLoop(pts ...sectionrecord.Point2) sectionrecord.LoopRecord { return rgPoly(pts...) }

// CertifiedContact (§5.3, §7.6): the certified gap test of a section's own
// segments, loops against loops included.
func TestCertifiedContact(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	square := rgLoop(pt(0, 0), pt(10, 0), pt(10, 10), pt(0, 10))
	// a clockwise triangular hole whose tip stands g from the square's right side
	web := func(g float64) []sectionrecord.LoopRecord {
		return []sectionrecord.LoopRecord{square, rgLoop(pt(10-g, 5), pt(5, 3), pt(5, 7))}
	}
	entries, err := sectionaudit.EntriesOf(budget(), web(1))
	require.NoError(t, err)
	floor, err := sectionaudit.ContactFloor(budget(), entries)
	require.NoError(t, err)
	require.InDelta(t, 1.4e-8, floor, 0.5e-8, "about 1.4e-8 for a 10 × 10 section")

	t.Run("a web thinner than the floor between two loops refuses", func(t *testing.T) {
		ok, why, err := CertifiedContact(budget(), web(floor/2), 0)
		require.NoError(t, err)
		require.False(t, ok)
		require.Contains(t, why, "of loop 0 and segment")
		require.Contains(t, why, "of loop 1")
	})
	t.Run("a web above the floor passes", func(t *testing.T) {
		ok, why, err := CertifiedContact(budget(), web(4*floor), 0)
		require.NoError(t, err)
		require.True(t, ok, why)
	})
	t.Run("a displacement above a quarter of the floor refuses", func(t *testing.T) {
		ok, _, err := CertifiedContact(budget(), web(1e-3), floor/3)
		require.NoError(t, err)
		require.False(t, ok)
		ok, _, err = CertifiedContact(budget(), web(1e-3), floor/5)
		require.NoError(t, err)
		require.True(t, ok)
	})
	t.Run("neighbours that share a vertex are not paired", func(t *testing.T) {
		ok, why, err := CertifiedContact(budget(), []sectionrecord.LoopRecord{square}, 0)
		require.NoError(t, err)
		require.True(t, ok, why)
	})
	t.Run("a crossing near a segment's end is no gap", func(t *testing.T) {
		// Sides 0 and 3 cross at x = 1e-7, 1e-8 of side 0's length from its
		// start: segMinDist reads a positive gap there, the certified bound none.
		crossed := rgLoop(pt(0, 0), pt(10, 0), pt(10, 10), pt(1e-7, 1), pt(1e-7, -1))
		w0, err := boundarywalk.WalkOf(crossed.Segments[0], nil)
		require.NoError(t, err)
		w3, err := boundarywalk.WalkOf(crossed.Segments[3], nil)
		require.NoError(t, err)
		require.Positive(t, sectionaudit.SegMinDist(w0, w3), "the float closed form is blind to it")
		ok, _, err := CertifiedContact(budget(), []sectionrecord.LoopRecord{crossed}, 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
}

// An edge's box is the one SegGapLower reads (§6.2): it holds the points at the
// radius on the float ends' bearings, widened by c.
func TestWidenedBoxIsBuiltFromTheGapReading(t *testing.T) {
	t.Parallel()
	const r, c = 5.0, 1e-3
	// An arc from bearing 0.2 to 1.0 with both float ends 0.01 inside the
	// circle, so the points SegGapLower reads lie outside the float ends' box.
	w := survey2d.SegmentWalk{Kind: survey2d.WalkCircular, Radius: r, Th0: 0.2, Th1: 1.0,
		StartU: (r - 0.01) * math.Cos(0.2), StartV: (r - 0.01) * math.Sin(0.2),
		EndU: (r - 0.01) * math.Cos(1.0), EndV: (r - 0.01) * math.Sin(1.0)}
	box, ok := widenedBox(arrangedEdge{walk: w, c: c})
	require.True(t, ok)
	require.InDelta(t, r*math.Cos(1.0)-c, box[0], 1e-9, "min u: the end at bearing 1.0 on the circle, less c")
	require.InDelta(t, r*math.Sin(0.2)-c, box[1], 1e-9)
	require.InDelta(t, r*math.Cos(0.2)+c, box[2], 1e-9, "max u: the end at bearing 0.2 on the circle, plus c")
	require.InDelta(t, r*math.Sin(1.0)+c, box[3], 1e-9)
	require.Less(t, w.StartU, r*math.Cos(0.2), "the float end alone would give a smaller box")
}

// CertifiedContactLoops (carrier-record-design §8.3, T1 and T2): a pair of two
// zero-displacement loops is skipped, a displaced loop is tested against every
// loop, and a per-loop extra joins every segment's allowance.
func TestCertifiedContactLoops(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	square := rgLoop(pt(0, 0), pt(10, 0), pt(10, 10), pt(0, 10))
	web := func(g float64) []sectionrecord.LoopRecord {
		return []sectionrecord.LoopRecord{square, rgLoop(pt(10-g, 5), pt(5, 3), pt(5, 7))}
	}
	entries, err := sectionaudit.EntriesOf(budget(), web(1))
	require.NoError(t, err)
	floor, err := sectionaudit.ContactFloor(budget(), entries)
	require.NoError(t, err)
	thin := web(floor / 2)

	t.Run("two zero-displacement loops closer than the floor pass", func(t *testing.T) {
		ok, why, err := CertifiedContactLoops(budget(), thin, []float64{0, 0}, nil)
		require.NoError(t, err)
		require.True(t, ok, why)
		ok, _, err = CertifiedContact(budget(), thin, 0)
		require.NoError(t, err)
		require.False(t, ok, "CertifiedContact tests every pair")
	})
	t.Run("the same pair refuses once one loop is displaced", func(t *testing.T) {
		ok, why, err := CertifiedContactLoops(budget(), thin, []float64{floor / 20, 0}, nil)
		require.NoError(t, err)
		require.False(t, ok)
		require.Contains(t, why, "contact floor")
	})
	t.Run("a segment allowance of a quarter of the floor refuses", func(t *testing.T) {
		wide := web(1e-3)
		ok, _, err := CertifiedContactLoops(budget(), wide, []float64{0, 0}, []float64{0, floor / 2})
		require.NoError(t, err)
		require.False(t, ok)
		ok, _, err = CertifiedContactLoops(budget(), wide, []float64{0, 0}, []float64{0, floor / 20})
		require.NoError(t, err)
		require.True(t, ok)
	})
	t.Run("the extra of a zero-displacement loop widens a tested pair", func(t *testing.T) {
		wide := web(1e-3)
		ok, _, err := CertifiedContactLoops(budget(), wide, []float64{floor / 20, 0}, []float64{0, 0})
		require.NoError(t, err)
		require.True(t, ok)
		gap := 1e-3
		ok, _, err = CertifiedContactLoops(budget(), wide, []float64{floor / 20, 0}, []float64{0, gap})
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("a length mismatch refuses", func(t *testing.T) {
		ok, _, err := CertifiedContactLoops(budget(), thin, []float64{0}, nil)
		require.NoError(t, err)
		require.False(t, ok)
	})
}
