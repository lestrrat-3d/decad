package prismrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func carrierSquare() momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	hole := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: p(5, 5), Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
	}}
	return momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			line(p(0, 0), p(10, 0)), line(p(10, 0), p(10, 10)), line(p(10, 10), p(0, 10)), line(p(0, 10), p(0, 0)),
		}},
		Holes: []sectionrecord.LoopRecord{hole},
	}
}

func TestCarrierRecord(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	profile := carrierSquare()

	t.Run("undisplaced payload answers zeros", func(t *testing.T) {
		cr, ok, err := prismrecord.Payload{Profile: profile}.CarrierRecord(budget)
		require.NoError(t, err)
		require.True(t, ok)
		require.Zero(t, cr.Max())
		require.True(t, cr.ExactLoop(0))
		require.True(t, cr.ExactLoop(1))
		require.False(t, cr.ExactLoop(2))
	})

	valid := func() prismrecord.Payload {
		cr := prismrecord.ZeroCarrierRecord(profile)
		cr.SectionDelta = 1e-13
		cr.Junction[0][1] = 1e-13
		return prismrecord.Payload{Profile: profile, SectionDelta: 1e-13, Carriers: &cr}
	}

	t.Run("valid record", func(t *testing.T) {
		pp := valid()
		cr, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, 1e-13, cr.Max())
		require.False(t, cr.ExactLoop(0))
		require.True(t, cr.ExactLoop(1))
		// CarrierExact reads only ExactCarriers; an exact loop is read through the record.
		exact, err := pp.CarrierExact(budget, 1)
		require.NoError(t, err)
		require.False(t, exact)
	})

	t.Run("no record", func(t *testing.T) {
		pp := valid()
		pp.Carriers = nil
		_, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("changed displacement is stale", func(t *testing.T) {
		pp := valid()
		pp.SectionDelta = math.Nextafter(pp.SectionDelta, 1)
		_, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("changed segment is stale", func(t *testing.T) {
		pp := valid()
		moved := carrierSquare()
		seg := moved.Outer.Segments[0].(sectionrecord.LineSeg)
		seg.End.U = math.Nextafter(seg.End.U, 11)
		moved.Outer.Segments[0] = seg
		pp.Profile = moved
		_, ok, err := pp.CarrierRecord(budget)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("malformed entries are stale", func(t *testing.T) {
		for name, mutate := range map[string]func(*prismrecord.CarrierRecord){
			"above delta": func(cr *prismrecord.CarrierRecord) { cr.Segment[0][0] = 1 },
			"negative":    func(cr *prismrecord.CarrierRecord) { cr.Junction[1][0] = -1e-20 },
			"short":       func(cr *prismrecord.CarrierRecord) { cr.Segment[0] = cr.Segment[0][:3] },
			"not finite":  func(cr *prismrecord.CarrierRecord) { cr.Junction[0][2] = math.NaN() },
		} {
			t.Run(name, func(t *testing.T) {
				pp := valid()
				cr := *pp.Carriers
				cr.Segment = [][]float64{append([]float64(nil), cr.Segment[0]...), append([]float64(nil), cr.Segment[1]...)}
				cr.Junction = [][]float64{append([]float64(nil), cr.Junction[0]...), append([]float64(nil), cr.Junction[1]...)}
				mutate(&cr)
				pp.Carriers = &cr
				_, ok, err := pp.CarrierRecord(budget)
				require.NoError(t, err)
				require.False(t, ok)
			})
		}
	})

	t.Run("uniform", func(t *testing.T) {
		cr := prismrecord.UniformCarrierRecord(profile, 2e-14)
		require.Equal(t, 2e-14, cr.Max())
		require.Equal(t, 2e-14, cr.LoopMax(1))
		require.False(t, cr.ExactLoop(1))
	})
}

// TestCarrierRecordFailsClosed pins §3.4's ownership: Build clears a record,
// BuildWithRecord binds one to the body it mints, and a payload copied into
// another body or placed again claims nothing.
func TestCarrierRecordFailsClosed(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	sq := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 4}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 4}, End: sectionrecord.Point2{U: 4, V: 4}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 4, V: 4}, End: sectionrecord.Point2{V: 4}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{V: 4}, End: sectionrecord.Point2{}, TStart: 0, TEnd: 1},
	}}}
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	pp := prismrecord.Payload{Profile: sq, Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 1, SectionDelta: 1e-13}
	cr := prismrecord.UniformCarrierRecord(sq, 1e-13)
	pp.Carriers = &cr

	built, err := prismrecord.Build(t.Context(), env, pp, freeform.NewFreeformWork())
	require.NoError(t, err)
	_, ok, err := prismrecord.CarrierRecordOf(budget, built)
	require.NoError(t, err)
	require.False(t, ok, "Build clears the record")

	owned, err := prismrecord.BuildWithRecord(t.Context(), env, pp, cr)
	require.NoError(t, err)
	_, ok, err = prismrecord.CarrierRecordOf(budget, owned)
	require.NoError(t, err)
	require.True(t, ok)

	copied := &topo.Body{Payload: owned.Payload}
	_, ok, err = prismrecord.CarrierRecordOf(budget, copied)
	require.NoError(t, err)
	require.False(t, ok, "a payload copied into another body claims nothing")

	placed, err := owned.Payload.(prismrecord.Payload).Placed(t.Context(), env, r3.Identity())
	require.NoError(t, err)
	_, ok, err = prismrecord.CarrierRecordOf(budget, placed)
	require.NoError(t, err)
	require.False(t, ok, "Placed re-evaluates through Build and drops the record")
}
