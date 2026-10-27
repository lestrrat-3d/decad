package prismshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capShell is a shell of the prism over section × [0, h] at the thickness,
// inward when s is +1 and outward when s is -1, keeping kept caps.
func capShell(t *testing.T, section momentinput.Profile, h, s, thickness float64, kept int) prismshell.CapShellInput {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return prismshell.CapShellInput{
		Prism:         prismrecord.Payload{Profile: section, Frame: frame, Z0: 0, Z1: h, Xform: r3.Identity()},
		KeptCaps:      kept,
		Sense:         s,
		Thickness:     units.Millimeters(thickness),
		HeldThickness: thickness,
	}
}

// square10 is the section [0, 10]².
func square10() momentinput.Profile {
	return momentinput.Profile{Outer: squareAt(0, 0, 10)}
}

func capShellEnv() topo.Env {
	return topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
}

// requireVolume checks body's volume against want within its own bound.
func requireVolume(t *testing.T, want float64, body *topo.Body) {
	t.Helper()
	require.InDelta(t, want, body.Volume.Value.Base(), body.Volume.Bound.Base()+1e-9)
}

// TestCapShellOffsetSectionGates walks the construction gates in order: the
// section limit, then the floor limit a kept cap adds inward, each with its
// own refusal, and none of them outward.
func TestCapShellOffsetSectionGates(t *testing.T) {
	t.Parallel()
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	_, err := capShell(t, square10(), 4, 1, 5, 0).OffsetSection(budget())
	require.ErrorIs(t, err, decaderr.ErrDegenerate, "t reaches the section's inradius")

	_, err = capShell(t, square10(), 4, 1, 4, 1).OffsetSection(budget())
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "meets or exceeds the accepted maximum", "a cup's floor eats the sweep")

	_, err = capShell(t, square10(), 1e-12, 1, 1, 1).OffsetSection(budget())
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "at or below the evaluator's rounding tolerance")

	_, err = capShell(t, square10(), 4, 1, 2, 2).OffsetSection(budget())
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "leaves no cavity between the two kept caps", "two floors eat the sweep")

	// With both caps removed only the section limit applies.
	offset, err := capShell(t, square10(), 4, 1, 4, 0).OffsetSection(budget())
	require.NoError(t, err)
	area, err := momentinput.LoopEnclosedArea(t.Context(), offset.Outer)
	require.NoError(t, err)
	require.InDelta(t, 4.0, area.Value, area.Bound+1e-9)

	// Outward no limit applies: the dilation grows around the section.
	_, err = capShell(t, square10(), 4, -1, 50, 2).OffsetSection(budget())
	require.NoError(t, err)
}

// TestCapShellBuilds checks each record's volume: a tube is the annulus over
// the sweep, holed bands are one lump per band, and a closed shell is the
// receiver less its cavity inward or the dilation less the receiver outward.
// A dilation rounds each convex corner with a quarter circle of radius t.
func TestCapShellBuilds(t *testing.T) {
	t.Parallel()

	t.Run("tube", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			s    float64
			want float64
		}{{s: 1, want: (100 - 64) * 10}, {s: -1, want: (40 + math.Pi) * 10}} {
			in := capShell(t, square10(), 10, tc.s, 1, 0)
			offset, err := in.OffsetSection(proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			body, err := prismshell.BuildTube(t.Context(), capShellEnv(), in, offset)
			require.NoError(t, err)
			requireVolume(t, tc.want, body)
			require.Len(t, body.Lumps, 1)
		}
	})
	t.Run("bands", func(t *testing.T) {
		t.Parallel()
		holed := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{reversed(squareAt(4, 4, 2))}}
		in := capShell(t, holed, 10, 1, 1, 0)
		budget := proofbound.NewWorkBudget(t.Context())
		offset, err := in.OffsetSection(budget)
		require.NoError(t, err)
		delta, err := in.SectionDelta(budget)
		require.NoError(t, err)
		body, err := prismshell.BuildBands(t.Context(), capShellEnv(), in, offset, delta)
		require.NoError(t, err)
		// The outer band is 10² − 8²; the hole's lining is its dilation, the
		// four sides pushed out by 1 and the corners rounded, less the hole.
		requireVolume(t, (36+8+math.Pi)*10, body)
		require.Len(t, body.Lumps, 2)
	})
	t.Run("closed", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			s    float64
			want float64
		}{{s: 1, want: 1000 - 512}, {s: -1, want: (140+math.Pi)*12 - 1000}} {
			body, err := prismshell.BuildClosed(t.Context(), capShellEnv(), capShell(t, square10(), 10, tc.s, 1, 2))
			require.NoError(t, err)
			requireVolume(t, tc.want, body)
			require.Len(t, body.Lumps, 1)
			shells := body.Lumps[0].Shells
			require.Len(t, shells, 2)
			require.False(t, shells[0].Void)
			require.True(t, shells[1].Void, "the cavity is a void shell")
		}
	})
	t.Run("closed holed", func(t *testing.T) {
		t.Parallel()
		// The 10² section less a 2² hole, 10 tall, hollowed inward by 1: the
		// cavity is the 8² inset less the hole's dilation (4² with its four
		// corners rounded, 12 + π) over the 8 between the kept caps, a ring
		// around the hole's lining.
		holed := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{reversed(squareAt(4, 4, 2))}}
		body, err := prismshell.BuildClosed(t.Context(), capShellEnv(), capShell(t, holed, 10, 1, 1, 2))
		require.NoError(t, err)
		requireVolume(t, 96*10-(64-12-math.Pi)*8, body)
		require.Len(t, body.Lumps, 1)
		shells := body.Lumps[0].Shells
		require.Len(t, shells, 2)
		require.False(t, shells[0].Void)
		require.True(t, shells[1].Void, "the ring around the hole's lining is the void shell")
		sp, ok := body.Payload.(stackedrecord.Payload)
		require.True(t, ok)
		require.True(t, sp.ClosedShell)
		mesh, err := stackedrecord.Tessellate(t.Context(), body, sp, 0.05, true)
		require.NoError(t, err)
		require.NoError(t, topobuild.RequireShellWinding(t.Context(), mesh, body.Lumps))

		// The same record without the mark reads the lining's outer wall as
		// exterior, so the cavity becomes a second, inside-out lump. The
		// per-shell winding audit refuses that mesh.
		sp.ClosedShell = false
		unmarked, err := stackedrecord.Build(t.Context(), capShellEnv(), sp, stackedrecord.OwnPlan{})
		require.NoError(t, err)
		require.Len(t, unmarked.Lumps, 2)
		require.False(t, unmarked.Lumps[1].Shells[0].Void)
		mesh, err = stackedrecord.Tessellate(t.Context(), unmarked, sp, 0.05, true)
		require.NoError(t, err)
		err = topobuild.RequireShellWinding(t.Context(), mesh, unmarked.Lumps)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "do not wind outward")
	})
	t.Run("closed holed outward", func(t *testing.T) {
		t.Parallel()
		// The same section wrapped outward by 0.5: the outer wall is the 11²
		// square with its four corners rounded (120 + π/4), the hole erodes to
		// the 1² square at (4.5, 4.5) with mitered corners, and the cavity is
		// the section itself over [0, 10], a ring around the hole's lining.
		holed := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{reversed(squareAt(4, 4, 2))}}
		body, err := prismshell.BuildClosed(t.Context(), capShellEnv(), capShell(t, holed, 10, -1, 0.5, 2))
		require.NoError(t, err)
		requireVolume(t, (120+math.Pi/4-1)*11-96*10, body)
		require.Len(t, body.Lumps, 1)
		shells := body.Lumps[0].Shells
		require.Len(t, shells, 2)
		require.False(t, shells[0].Void)
		require.True(t, shells[1].Void, "the ring around the hole's lining is the void shell")
		sp, ok := body.Payload.(stackedrecord.Payload)
		require.True(t, ok)
		require.True(t, sp.ClosedShell)
		require.Len(t, sp.Slabs, 3)
		require.Equal(t, []float64{-0.5, 0, 10}, []float64{sp.Slabs[0].Z0, sp.Slabs[1].Z0, sp.Slabs[2].Z0})
		require.Equal(t, 10.5, sp.Slabs[2].Z1)
		require.Len(t, sp.Slabs[1].Regions, 2, "the outer band and the hole's lining")
		// The cavity's floor and ceiling expose the receiver's own section.
		require.Equal(t, []momentinput.Profile{holed}, sp.Interfaces[0].LowerExposed)
		require.Equal(t, []momentinput.Profile{holed}, sp.Interfaces[1].UpperExposed)
		mesh, err := stackedrecord.Tessellate(t.Context(), body, sp, 0.05, true)
		require.NoError(t, err)
		require.NoError(t, topobuild.RequireShellWinding(t.Context(), mesh, body.Lumps))
	})
	t.Run("closed holed outward fills a hole", func(t *testing.T) {
		t.Parallel()
		// Outward the hole erodes by t: the 2² hole empties at t = 1 and past
		// it, and a radius-1 round hole at t = 1. Each drops a feature (S11a).
		square := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{reversed(squareAt(4, 4, 2))}}
		round := momentinput.Profile{Outer: squareAt(0, 0, 10), Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 5, V: 5}, Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
		}}}}
		for _, tc := range []struct {
			name    string
			section momentinput.Profile
			t       float64
		}{{"square at its half-width", square, 1}, {"square past it", square, 1.5}, {"circle at its radius", round, 1}} {
			_, err := prismshell.BuildClosed(t.Context(), capShellEnv(), capShell(t, tc.section, 10, -1, tc.t, 2))
			require.ErrorIs(t, err, offset2d.ErrDrop, tc.name)
			require.ErrorIs(t, err, decaderr.ErrUnsupported, tc.name)
		}
	})
	t.Run("closed refusals", func(t *testing.T) {
		t.Parallel()
		_, err := prismshell.BuildClosed(t.Context(), capShellEnv(), capShell(t, square10(), 10, 1, 5, 2))
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		in := capShell(t, square10(), 10, 1, 1, 2)
		in.Prism.SectionDelta = 1e-6
		_, err = prismshell.BuildClosed(t.Context(), capShellEnv(), in)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, "an inexact section is not shelled")
	})
}

// reversed walks a square loop clockwise, the sense a hole is recorded in.
func reversed(l sectionrecord.LoopRecord) sectionrecord.LoopRecord {
	out := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(l.Segments))}
	for i, seg := range l.Segments {
		ls, ok := seg.(sectionrecord.LineSeg)
		if !ok {
			panic("squareAt builds lines only")
		}
		out.Segments[len(l.Segments)-1-i] = sectionrecord.LineSeg{Start: ls.End, End: ls.Start, TStart: 0, TEnd: 1}
	}
	return out
}
