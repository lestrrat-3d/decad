package prismrecord_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func captureBuildEnv() topo.Env {
	return topo.Env{Doc: &topo.Document{}, Ref: 7, Alloc: topotest.Alloc{}}
}

func requireCaptureBody(t *testing.T, a, b *topo.Body) {
	t.Helper()
	require.Equal(t, a.Kind, b.Kind)
	require.Equal(t, a.Solid, b.Solid)
	for i, pair := range [][2]float64{{a.Volume.Value.Base(), b.Volume.Value.Base()}, {a.Volume.Bound.Base(), b.Volume.Bound.Base()},
		{a.Area.Value.Base(), b.Area.Value.Base()}, {a.Area.Bound.Base(), b.Area.Bound.Base()},
		{a.Centroid.Bound.Base(), b.Centroid.Bound.Base()}} {
		require.Equal(t, math.Float64bits(pair[0]), math.Float64bits(pair[1]), "body measurement %d", i)
	}
	require.Equal(t, a.Volume.Exactness, b.Volume.Exactness)
	require.Equal(t, a.Area.Exactness, b.Area.Exactness)
	require.Equal(t, a.Centroid.Exactness, b.Centroid.Exactness)
	// These vectors contain exactly three coordinates, never topology pointers.
	require.Equal(t, a.Centroid.Value, b.Centroid.Value)
	require.Equal(t, a.Bounds.Min, b.Bounds.Min)
	require.Equal(t, a.Bounds.Max, b.Bounds.Max)
	require.Equal(t, a.Origin.Producer, b.Origin.Producer)
	require.Equal(t, a.Origin.Role, b.Origin.Role)
	af, bf := topobuild.Faces(a), topobuild.Faces(b)
	require.Equal(t, len(af), len(bf))
	for i, f := range af {
		g := bf[i]
		require.Equal(t, math.Float64bits(f.Area), math.Float64bits(g.Area), "face %d", i)
		require.Equal(t, math.Float64bits(f.AreaBound), math.Float64bits(g.AreaBound), "face bound %d", i)
		require.Equal(t, f.Reversed, g.Reversed)
		require.Equal(t, len(f.Origins), len(g.Origins))
		for j, origin := range f.Origins {
			require.Equal(t, origin.Role, g.Origins[j].Role)
			require.Equal(t, origin.Producer, g.Origins[j].Producer)
		}
		require.Equal(t, len(f.Loops), len(g.Loops))
		for j, loop := range f.Loops {
			require.Equal(t, loop.Outer, g.Loops[j].Outer)
			require.Equal(t, len(loop.CoEdges), len(g.Loops[j].CoEdges))
		}
	}
	ae, be := topobuild.Edges(a), topobuild.Edges(b)
	require.Equal(t, len(ae), len(be))
	for i, e := range ae {
		f := be[i]
		require.Equal(t, math.Float64bits(e.Length), math.Float64bits(f.Length), "edge %d", i)
		require.Equal(t, math.Float64bits(e.LengthBound), math.Float64bits(f.LengthBound), "edge bound %d", i)
		require.Equal(t, e.Convex, f.Convex)
		require.Equal(t, len(e.Faces), len(f.Faces))
		require.Equal(t, e.Start.Position, f.Start.Position)
		require.Equal(t, e.End.Position, f.End.Position)
	}
}

func TestBuildAnalyticWalkCaptureFallbacks(t *testing.T) {
	plain := boxPayload(t)
	pointer := boxPayload(t)
	line := pointer.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
	pointer.Profile.Outer.Segments[0] = &line
	unit := boxPayload(t)
	unit.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Centimeters(1), CCW: true, TEnd: 1}}}}
	mixed := boxPayload(t)
	mixed.Profile.Outer.Segments[1] = sectionrecord.FitSplineSeg{
		Fit: []sectionrecord.Point2{{U: 4}, {U: 4, V: 4}}, TEnd: 1}
	for i, p := range []prismrecord.Payload{plain, pointer, unit, mixed} {
		// A nonnil unmetered cache forces the original uncaptured path without
		// changing the profile or any geometric calculation.
		fallback := p
		fallback.Walks = &momentinput.ProfileWalks{Profile: p.Profile}
		wa, wb := freeform.NewFreeformWork(), freeform.NewFreeformWork()
		got, err := prismrecord.Build(t.Context(), captureBuildEnv(), p, wa)
		require.NoError(t, err, "case %d", i)
		want, err := prismrecord.Build(t.Context(), captureBuildEnv(), fallback, wb)
		require.NoError(t, err, "case %d", i)
		requireCaptureBody(t, want, got)
		require.Equal(t, wa.Spent, wb.Spent)
		require.Equal(t, wa.ReconstructionSpent, wb.ReconstructionSpent)
		cache := got.Payload.(prismrecord.Payload).Walks
		require.True(t, cache.Reusable(p.Profile))
		// Stale records still resolve instead of reading another profile's walks.
		stale := p
		stale.Walks = &momentinput.ProfileWalks{Profile: momentinput.Profile{}, Metered: true}
		rebuilt, err := prismrecord.Build(t.Context(), captureBuildEnv(), stale, freeform.NewFreeformWork())
		require.NoError(t, err)
		requireCaptureBody(t, want, rebuilt)
	}
	// Analytic nil and exhausted counters must still succeed without a fresh
	// capture introducing Step(0) or touching either counter.
	for _, work := range []*freeform.FreeformWork{nil, {Spent: freeform.FreeformWorkLimit, ReconstructionSpent: freeform.ReconstructionWorkLimit}} {
		_, err := prismrecord.Build(t.Context(), captureBuildEnv(), plain, work)
		require.NoError(t, err)
		if work != nil {
			require.Equal(t, freeform.FreeformWorkLimit, work.Spent)
			require.Equal(t, freeform.ReconstructionWorkLimit, work.ReconstructionSpent)
		}
	}
}

func TestBuildAnalyticWalkCaptureKeepsReplayAndErrorOrder(t *testing.T) {
	p := boxPayload(t)
	cache, err := momentinput.ResolveProfileWalks(p.Profile, freeform.NewFreeformWork())
	require.NoError(t, err)
	// Synthetic nonzero recorded charges distinguish the existing replay branch
	// from a fresh capture that incorrectly replaces a reusable payload cache.
	cache.Spent, cache.ReconstructionSpent = 3, 5
	p.Walks = cache
	work := freeform.NewFreeformWork()
	body, err := prismrecord.Build(t.Context(), captureBuildEnv(), p, work)
	require.NoError(t, err)
	require.Equal(t, uint64(3), work.Spent)
	require.Equal(t, uint64(5), work.ReconstructionSpent)
	require.True(t, body.Payload.(prismrecord.Payload).Walks == cache)
	exhausted := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit, ReconstructionSpent: freeform.ReconstructionWorkLimit}
	_, err = prismrecord.Build(t.Context(), captureBuildEnv(), p, exhausted)
	require.ErrorIs(t, err, momentinput.ErrUnsupported)
	for _, existing := range []*momentinput.ProfileWalks{nil, cache} {
		invalid := p
		invalid.Walks = existing
		invalid.Z1 = invalid.Z0
		_, err = prismrecord.Build(t.Context(), captureBuildEnv(), invalid, exhausted)
		require.ErrorIs(t, err, momentinput.ErrDegenerate)
		require.Contains(t, err.Error(), "sweep interval is empty")
		bad := boxPayload(t)
		bad.Z1 = bad.Z0
		bad.Profile.Outer.Segments[0] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: math.NaN()}, TEnd: 1}
		bad.Walks = existing
		_, err = prismrecord.Build(t.Context(), captureBuildEnv(), bad, exhausted)
		require.ErrorIs(t, err, momentinput.ErrNotFinite)
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = prismrecord.Build(ctx, captureBuildEnv(), bad, exhausted)
		require.ErrorIs(t, err, context.Canceled)
	}
}

func captureWalkBits(w survey2d.SegmentWalk) []uint64 {
	values := []float64{w.StartU, w.StartV, w.EndU, w.EndV, w.StartBound.U, w.StartBound.V, w.EndBound.U, w.EndBound.V,
		w.TanInU, w.TanInV, w.TanOutU, w.TanOutV, w.TanInBound, w.TanOutBound, w.Length, w.LengthBound, w.LengthUpper,
		w.CoordUpper, w.AxisRadiusUpper, w.AxisMomentUpper, w.StartVBound, w.EndVBound, w.CVBound, w.CU, w.CV, w.Radius,
		w.RadiusBound, w.Th0, w.Th1}
	bits := make([]uint64, len(values))
	for i, v := range values {
		bits[i] = math.Float64bits(v)
	}
	return bits
}

func TestBuildAnalyticWalkCapturePlacementImmutability(t *testing.T) {
	p := boxPayload(t)
	body, err := prismrecord.Build(t.Context(), captureBuildEnv(), p, freeform.NewFreeformWork())
	require.NoError(t, err)
	cache := body.Payload.(prismrecord.Payload).Walks
	snapshots := append([]survey2d.SegmentWalk(nil), cache.Outer...)
	shift, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	for placementIndex, placement := range []r3.Transform{r3.Identity(), shift, r3.Identity(), shift} {
		// Copy and Transform both reevaluate this payload through Placed.
		placed, err := body.Payload.Placed(t.Context(), captureBuildEnv(), placement)
		require.NoError(t, err)
		require.True(t, placed.Payload.(prismrecord.Payload).Walks == cache)
		require.Equal(t, units.CubicMillimeters(64), placed.Volume.Value)
		if placementIndex%2 == 0 {
			requireCaptureBody(t, body, placed)
		} else {
			require.Equal(t, r3.NewVec(10, 0, 0), placed.Bounds.Min)
			require.Equal(t, r3.NewVec(14, 4, 4), placed.Bounds.Max)
		}
		for i, w := range snapshots {
			got := cache.Outer[i]
			a, b := captureWalkBits(w), captureWalkBits(got)
			for j := range a {
				require.Equal(t, a[j], b[j], "walk %d field %d", i, j)
			}
			require.Equal(t, w.Kind, got.Kind)
			require.Equal(t, w.Closed, got.Closed)
			require.Equal(t, w.Reversed, got.Reversed)
			require.Equal(t, w.FitInterpolated, got.FitInterpolated)
			require.Nil(t, got.Spans)
		}
	}
}

type buildCapturePollContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	calls, cancelAt int
}

func (c *buildCapturePollContext) Err() error {
	c.calls++
	if c.cancelAt > 0 && c.calls >= c.cancelAt {
		return context.Canceled
	}
	return nil
}

func TestBuildAnalyticWalkCapturePollOrder(t *testing.T) {
	p := boxPayload(t)
	fallback := p
	fallback.Walks = &momentinput.ProfileWalks{}
	counter := &buildCapturePollContext{Context: t.Context()}
	_, err := prismrecord.Build(counter, captureBuildEnv(), fallback, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.Less(t, counter.calls, 200, "bounded cancellation matrix")
	for stop := 1; stop <= counter.calls+1; stop++ {
		a := &buildCapturePollContext{Context: t.Context(), cancelAt: stop}
		b := &buildCapturePollContext{Context: t.Context(), cancelAt: stop}
		_, want := prismrecord.Build(a, captureBuildEnv(), fallback, freeform.NewFreeformWork())
		_, got := prismrecord.Build(b, captureBuildEnv(), p, freeform.NewFreeformWork())
		require.Equal(t, a.calls, b.calls, "cancel point %d", stop)
		require.Equal(t, want == nil, got == nil, "cancel point %d", stop)
		if want != nil {
			require.ErrorIs(t, got, context.Canceled)
		}
	}
	// Cap-frame failure still precedes cache selection or charge replay.
	invalid := p
	invalid.Frame = r3.Frame{}
	var plainErr error
	for _, walks := range []*momentinput.ProfileWalks{nil, {}} {
		invalid.Walks = walks
		_, err := prismrecord.Build(t.Context(), captureBuildEnv(), invalid, freeform.NewFreeformWork())
		require.Error(t, err)
		if plainErr == nil {
			plainErr = err
		} else {
			require.EqualError(t, err, plainErr.Error())
		}
	}
}
