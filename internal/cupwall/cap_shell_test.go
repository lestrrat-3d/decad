package cupwall_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func capShellLoop(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	line := func(a, b, c, d float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: sectionrecord.Point2{U: a, V: b}, End: sectionrecord.Point2{U: c, V: d}, TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(u0, v0, u1, v0), line(u1, v0, u1, v1), line(u1, v1, u0, v1), line(u0, v1, u0, v0)}}
}

// capShellInput is a 100 x 60 box 20 tall, shelled 5 thick with a thickness
// whose own displacement is 1e-3, so the offset section carries a nonzero
// displacement the build must record.
func capShellInput(t *testing.T, keptCaps int, hole bool) prismshell.CapShellInput {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: capShellLoop(0, 0, 100, 60)}
	if hole {
		inner := capShellLoop(40, 20, 60, 40)
		// A hole is walked against the outer loop.
		for i, j := 0, len(inner.Segments)-1; i < j; i, j = i+1, j-1 {
			inner.Segments[i], inner.Segments[j] = inner.Segments[j], inner.Segments[i]
		}
		for i, s := range inner.Segments {
			l := s.(sectionrecord.LineSeg)
			l.Start, l.End = l.End, l.Start
			inner.Segments[i] = l
		}
		profile.Holes = []sectionrecord.LoopRecord{inner}
	}
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 20, Profile: profile}
	return prismshell.CapShellInput{Prism: pp, KeptCaps: keptCaps, Sense: 1, Thickness: units.Millimeters(5),
		HeldThickness: 5, ThicknessDelta: 1e-3}
}

func capShellEnv() topo.Env { return topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}} }

func faceWithRole(t *testing.T, body *topo.Body, role string) *topo.Face {
	t.Helper()
	for _, f := range topobuild.Faces(body) {
		for _, o := range f.Origins {
			if o.Role == role {
				return f
			}
		}
	}
	return nil
}

// TestCapShell builds each shell class from a real prism payload through the
// offset stage and checks what the result records. Two guards have no input of
// their own: the first context poll in PlanCapShell is repeated by the offset
// construction, and SectionDelta fails only on cancellation, which the build
// that follows reports as well.
func TestCapShell(t *testing.T) {
	t.Parallel()

	wantDelta := func(t *testing.T, in prismshell.CapShellInput) float64 {
		t.Helper()
		d, err := in.SectionDelta(proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, err)
		require.Positive(t, d, "the fixture's thickness displacement reaches the offset section")
		return d
	}

	for _, tc := range []struct {
		name       string
		removedEnd bool
		floorZ     float64
	}{
		{"cup open at the end cap", true, 5},
		{"cup open at the start cap", false, 15},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			in := capShellInput(t, 1, false)
			plan, err := cupwall.PlanCapShell(t.Context(), in, tc.removedEnd)
			require.NoError(t, err)
			body, err := plan.Build(t.Context(), capShellEnv())
			require.NoError(t, err)
			cup, ok := body.Payload.(cupwall.Payload)
			require.True(t, ok)
			require.Equal(t, wantDelta(t, in), cup.OffsetDelta)
			require.Equal(t, in.HeldThickness, cup.Thickness)
			require.Equal(t, in.ThicknessDelta, cup.ThicknessDelta)
			require.Equal(t, cupwall.Inward, cup.Sense)
			floor := faceWithRole(t, body, cupwall.RoleShellCap)
			require.NotNil(t, floor, "a cup has a pocket floor")
			require.InDelta(t, tc.floorZ, floor.Surface.(surfacegeom.Plane).Frame.Origin().Z, 1e-9)
		})
	}

	t.Run("outward cup opens the same way", func(t *testing.T) {
		t.Parallel()
		in := capShellInput(t, 1, false)
		in.Sense = -1
		plan, err := cupwall.PlanCapShell(t.Context(), in, true)
		require.NoError(t, err)
		body, err := plan.Build(t.Context(), capShellEnv())
		require.NoError(t, err)
		cup := body.Payload.(cupwall.Payload)
		require.Equal(t, cupwall.Outward, cup.Sense)
		require.Equal(t, wantDelta(t, in), cup.OffsetDelta)
	})

	t.Run("both caps removed over a plain section is a tube", func(t *testing.T) {
		t.Parallel()
		in := capShellInput(t, 0, false)
		plan, err := cupwall.PlanCapShell(t.Context(), in, false)
		require.NoError(t, err)
		body, err := plan.Build(t.Context(), capShellEnv())
		require.NoError(t, err)
		tube, ok := body.Payload.(prismrecord.Payload)
		require.True(t, ok, "a tube is a plain prism over the annular section")
		require.Len(t, tube.Profile.Holes, 1)
		require.Nil(t, faceWithRole(t, body, cupwall.RoleShellCap))
	})

	t.Run("both caps removed over a holed section is bands", func(t *testing.T) {
		t.Parallel()
		in := capShellInput(t, 0, true)
		plan, err := cupwall.PlanCapShell(t.Context(), in, false)
		require.NoError(t, err)
		body, err := plan.Build(t.Context(), capShellEnv())
		require.NoError(t, err)
		require.Nil(t, faceWithRole(t, body, cupwall.RoleShellCap))
		require.Greater(t, len(topobuild.Faces(body)), 8, "one band around the outer loop plus one lining the hole")
		bands, ok := body.Payload.(stackedrecord.Payload)
		require.True(t, ok, "bands are one stacked slab")
		require.Equal(t, wantDelta(t, in), bands.SectionDelta)
	})

	t.Run("a canceled context stops the offset stage", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := cupwall.PlanCapShell(ctx, capShellInput(t, 1, false), true)
		require.ErrorIs(t, err, context.Canceled)
	})

	t.Run("a wall thicker than the section refuses", func(t *testing.T) {
		t.Parallel()
		in := capShellInput(t, 1, false)
		in.Thickness, in.HeldThickness = units.Millimeters(40), 40
		plan, err := cupwall.PlanCapShell(t.Context(), in, true)
		require.Error(t, err)
		require.Nil(t, plan)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
	})

	t.Run("cancellation after the offset stage stops the build", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		plan, err := cupwall.PlanCapShell(ctx, capShellInput(t, 0, true), false)
		require.NoError(t, err)
		cancel()
		_, err = plan.Build(ctx, capShellEnv())
		require.ErrorIs(t, err, context.Canceled)
	})
}
