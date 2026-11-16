package prismrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pt(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }

func arcLoop(start, end sectionrecord.Point2) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: pt(0, 0), Start: start, End: end, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: end, End: start, TStart: 0, TEnd: 1},
	}}}
}

func TestLemmaH(t *testing.T) {
	t.Run("line takes its larger junction entry", func(t *testing.T) {
		profile := carrierSquare()
		cr := prismrecord.ZeroCarrierRecord(profile)
		cr.Junction[0][0], cr.Junction[0][1] = 3e-13, 5e-13
		// Segment 1 sits between junction 0 (3e-13) and junction 1 (5e-13).
		require.Equal(t, 5e-13, cr.SegmentH(0, 1))
		require.Equal(t, 3e-13, cr.SegmentH(0, 0), "segment 0 sits between junction 3 (0) and junction 0 (3e-13)")
		require.Equal(t, 5e-13, cr.LoopH(0))
	})

	t.Run("whole circle takes its carrier entry", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(carrierSquare())
		cr.Segment[1][0] = 2e-13
		require.Equal(t, 2e-13, cr.SegmentH(1, 0))
	})

	t.Run("arc takes eps plus twice eps plus delta", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(arcLoop(pt(10, 0), pt(0, 10)))
		cr.Segment[0][0] = 1e-13
		cr.Junction[0][0], cr.Junction[0][1] = 2e-13, 3e-13
		h := cr.SegmentH(0, 0)
		want := 1e-13 + 2*(1e-13+3e-13)
		require.GreaterOrEqual(t, h, want)
		require.InDelta(t, want, h, 1e-26)
	})

	t.Run("arc whose end could slide past its sweep is infinite", func(t *testing.T) {
		// A near-full-turn arc: its short-way slide cannot be bounded.
		cr := prismrecord.ZeroCarrierRecord(arcLoop(pt(10, 0), pt(10, -1e-9)))
		cr.Junction[0][0], cr.Junction[0][1] = 1e-6, 1e-6
		require.True(t, math.IsInf(cr.SegmentH(0, 0), 1))
		// A tiny sweep cannot absorb a slide either.
		cr = prismrecord.ZeroCarrierRecord(arcLoop(pt(10, 0), pt(10, 1e-9)))
		cr.Junction[0][0], cr.Junction[0][1] = 1e-6, 1e-6
		require.True(t, math.IsInf(cr.SegmentH(0, 0), 1))
	})

	t.Run("arc smaller than its slide is infinite", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(arcLoop(pt(1, 0), pt(0, 1)))
		cr.Junction[0][0], cr.Junction[0][1] = 3, 3
		require.True(t, math.IsInf(cr.SegmentH(0, 0), 1))
	})

	t.Run("narrowed circle reads its range as the sweep", func(t *testing.T) {
		circle := sectionrecord.CircleSeg{Center: pt(0, 0), Radius: units.Millimeters(10), CCW: true, TStart: 0.1, TEnd: 0.6}
		profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}
		cr := prismrecord.ZeroCarrierRecord(profile)
		cr.Segment[0][0], cr.Junction[0][0] = 1e-13, 1e-13
		require.InDelta(t, 1e-13+2*(2e-13), cr.SegmentH(0, 0), 1e-26)
		circle.TEnd = 0.1 + 1e-14 // a tiny sweep cannot absorb the slide
		profile.Outer.Segments[0] = circle
		cr = prismrecord.ZeroCarrierRecord(profile)
		cr.Segment[0][0], cr.Junction[0][0] = 1e-13, 1e-13
		require.True(t, math.IsInf(cr.SegmentH(0, 0), 1))
	})

	t.Run("zero entries read zero whatever the arc", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(arcLoop(pt(10, 0), pt(10, 1e-9)))
		require.Zero(t, cr.SegmentH(0, 0))
	})
}

func TestCarrierRecordCR4AndOwnership(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	profile := arcLoop(pt(10, 0), pt(0, 10))

	t.Run("record shares no slice with the payload", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(profile)
		profile.Outer.Segments[1] = sectionrecord.LineSeg{Start: pt(9, 9), End: pt(8, 8), TStart: 0, TEnd: 1}
		require.NotEqual(t, profile.Outer.Segments[1], cr.Profile.Outer.Segments[1])
		profile = arcLoop(pt(10, 0), pt(0, 10))
	})

	t.Run("an entry above h_loop at the displacement is valid, an infinite h is not", func(t *testing.T) {
		cr := prismrecord.ZeroCarrierRecord(profile)
		cr.Segment[0][0] = 1e-13
		cr.Junction[0][0], cr.Junction[0][1] = 1e-13, 1e-13
		h := cr.HMax()
		require.Greater(t, h, 1e-13)

		pp := prismrecord.Payload{Profile: profile, SectionDelta: h}
		cr.SectionDelta = h
		pp.Carriers = &cr
		_, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.True(t, ok)

		// CR4: a displacement below an arc's h_loop is not a valid record.
		pp.SectionDelta = 1e-13
		cr2 := cr
		cr2.SectionDelta = 1e-13
		pp.Carriers = &cr2
		_, ok, err = pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.False(t, ok, "h_loop above SectionDelta violates CR4")
	})

	t.Run("a windowed zero record stays valid with a radial residual", func(t *testing.T) {
		// A sketch-fillet arc whose End sits a few ulps off Start's radius.
		end := pt(0, math.Nextafter(10, 11))
		windowed := arcLoop(pt(10, 0), end)
		pp := prismrecord.Payload{Profile: windowed}
		cr, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, cr.HMax())
	})
}
