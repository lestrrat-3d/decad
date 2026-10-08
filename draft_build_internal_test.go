package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// draftSlot extrudes docs/draft-design.md F3's slot: lines 30 mm long joined
// by semicircles of radius 5 at G1 joins, swept 8 mm at a 3 degree taper.
func draftSlot(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(-15, 0, 15, 0, 5)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along}, WithTaper(units.Degrees(3)))
	require.NoError(t, err)
	return b
}

// TestDraftOffsetEnclosesTheExactProduct pins §8.1's span: every corner of the
// exact product of the height span and the tangent enclosure lies within
// dDelta of the held d, and dDelta is positive even for a millimetre sweep and
// a degree angle, since the tangent enclosure has width. A taper whose cosine
// enclosure reaches zero is SD2.
func TestDraftOffsetEnclosesTheExactProduct(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct{ h, hDelta, deg float64 }{
		{10, 0, 5}, {8, 0, -3}, {25.4, 3.6e-15, 7.5}, {1, 0, 89},
	} {
		alpha := tc.deg * units.Degree.Factor()
		d, dDelta, err := draftOffset(tc.h, tc.hDelta, alpha, 0)
		require.NoError(t, err)
		require.Greater(t, dDelta, 0.0)
		require.Equal(t, math.Signbit(d), tc.deg < 0)
		tan, ok := proofbound.RadTanSpan(proofbound.PointInterval(proofarith.FloatRat(alpha)))
		require.True(t, ok)
		for _, h := range []float64{tc.h - tc.hDelta, tc.h + tc.hDelta} {
			for _, tv := range []*big.Rat{tan.Lo, tan.Hi} {
				exact := new(big.Rat).Mul(proofarith.FloatRat(h), tv)
				gap := new(big.Rat).Abs(new(big.Rat).Sub(exact, proofarith.FloatRat(d)))
				require.LessOrEqual(t, gap.Cmp(proofarith.FloatRat(dDelta)), 0)
			}
		}
	}
	_, _, err := draftOffset(10, 0, math.Pi/2, 1e-3)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestDraftPayloadRecordsTheFarSection pins Table BD row BD1: the body's
// payload carries the far section the build made, one loop per near loop, and
// the positive displacement its built contour carries.
func TestDraftPayloadRecordsTheFarSection(t *testing.T) {
	t.Parallel()
	b := draftSlot(t)
	dp, ok := b.payload.(draftPayload)
	require.True(t, ok)
	require.True(t, dp.nearStart)
	require.Greater(t, dp.d, 0.0)
	require.Greater(t, dp.farDelta, 0.0)
	require.Len(t, dp.far.Outer.Segments, 4)
	require.Empty(t, dp.far.Holes)
	for _, seg := range dp.far.Outer.Segments {
		if arc, ok := seg.(ArcSeg); ok {
			require.InDelta(t, 5-dp.d, math.Hypot(arc.Start.U-arc.Center.U, arc.Start.V-arc.Center.V), 1e-12)
		}
	}
}

// TestDraftConeWallsReadNoChordLocusTerm pins §8's chord-versus-locus term at
// zero: both corners of each of the slot's Cone walls are G1 feet on the
// wall's own radials, so the band's proven corner skews are zero, and with
// them capband's chord-versus-locus allowance.
func TestDraftConeWallsReadNoChordLocusTerm(t *testing.T) {
	t.Parallel()
	b := draftSlot(t)
	dp := b.payload.(draftPayload)
	cbp := dp.band()
	nearZ, nearDelta, farZ, matSign := dp.levels()
	work := freeform.NewFreeformWork()
	loop := cbp.loops()[0]
	near, err := draftNearRim(t.Context(), cbp.prismLike(0, 0), 0, loop, nearZ, nearDelta, work)
	require.NoError(t, err)
	band, err := buildCapBand(t.Context(), &Body{doc: b.doc}, b.origin.producer, cbp, 0, loop, farZ, matSign, near, work)
	require.NoError(t, err)
	cones := 0
	for _, g := range band.geom {
		if !g.Circular {
			continue
		}
		cones++
		require.Zero(t, capPatchWindowSkew(g))
	}
	require.Equal(t, 2, cones)
}

// TestDraftDenotedNormalAllow pins the wall normal's denoted-surface term: it
// grows with the far contour's displacement, and a height its own span cannot
// keep positive reads +Inf rather than a number that understates it.
func TestDraftDenotedNormalAllow(t *testing.T) {
	t.Parallel()
	small := draftDenotedNormalAllow(1e-15, 0, 0, 10, 0)
	large := draftDenotedNormalAllow(1e-12, 0, 0, 10, 0)
	require.Greater(t, small, 0.0)
	require.Greater(t, large, small)
	require.GreaterOrEqual(t, small, 2e-16)
	require.True(t, math.IsInf(draftDenotedNormalAllow(1e-15, 6, 6, 10, 0), 1))
}
