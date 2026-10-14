package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChainBoundsCachedWalksMatchFreshResolution(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	s.Fix(p0)
	_, err = s.CreateSpline(p0, s.CreatePoint(10, 5), s.CreatePoint(20, 8), s.CreatePoint(30, 9))
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	doc := New()
	body, err := Extrude(t.Context(), doc, s, chains[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)

	check := func(t *testing.T, body *Body) {
		t.Helper()
		payload := body.n.Payload.(chainPayload)
		profile := payload.prism().Profile
		work := freeform.NewFreeformWork()
		captures := make([]chainWalkCapture, len(payload.chains))
		for ci, chain := range payload.chains {
			for _, segment := range chain.Segments {
				before, beforeRecon := boundarywalk.WorkSpent(work)
				walk, err := boundarywalk.WalkOf(segment, work)
				require.NoError(t, err)
				after, afterRecon := boundarywalk.WorkSpent(work)
				captures[ci].walks = append(captures[ci].walks, walk)
				captures[ci].charges = append(captures[ci].charges,
					momentinput.WalkReadCharge{Spent: after - before, ReconstructionSpent: afterRecon - beforeRecon})
			}
		}
		cachedWork, freshWork := *work, *work
		cachedWalks := chainBoundsWalks(profile, captures)
		cached, err := prismBoundsContext(t.Context(), payload.prism(), &cachedWork,
			cachedWalks)
		require.NoError(t, err)
		fresh, err := prismBoundsContext(t.Context(), payload.prism(), &freshWork, nil)
		require.NoError(t, err)
		require.Equal(t, fresh, cached)
		require.Equal(t, fresh, boxFromInternal(body.n.Bounds))
		for _, pair := range [][2]float64{
			{fresh.Min.X, cached.Min.X}, {fresh.Min.Y, cached.Min.Y}, {fresh.Min.Z, cached.Min.Z},
			{fresh.Max.X, cached.Max.X}, {fresh.Max.Y, cached.Max.Y}, {fresh.Max.Z, cached.Max.Z},
			{fresh.Bound.Base(), cached.Bound.Base()},
			{fresh.Min.X, body.n.Bounds.Min.X}, {fresh.Min.Y, body.n.Bounds.Min.Y}, {fresh.Min.Z, body.n.Bounds.Min.Z},
			{fresh.Max.X, body.n.Bounds.Max.X}, {fresh.Max.Y, body.n.Bounds.Max.Y}, {fresh.Max.Z, body.n.Bounds.Max.Z},
			{fresh.Bound.Base(), body.n.Bounds.Bound.Base()},
		} {
			require.Equal(t, math.Float64bits(pair[0]), math.Float64bits(pair[1]))
		}
		require.Equal(t, freshWork, cachedWork)
		cancelled, cancel := context.WithCancel(t.Context())
		cancel()
		cachedCancelled, freshCancelled := *work, *work
		_, cachedErr := prismBoundsContext(cancelled, payload.prism(), &cachedCancelled, cachedWalks)
		_, freshErr := prismBoundsContext(cancelled, payload.prism(), &freshCancelled, nil)
		require.ErrorIs(t, cachedErr, context.Canceled)
		require.EqualError(t, cachedErr, freshErr.Error())
		require.Equal(t, freshCancelled, cachedCancelled)
		for ci, chain := range payload.chains {
			for si, segment := range chain.Segments {
				charge := captures[ci].charges[si]
				if charge.Spent <= 1 {
					continue
				}
				near := freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - charge.Spent + 1}
				cachedNear, freshNear := near, near
				_, cachedErr := momentinput.ResolveOrRead(segment, &cachedNear, cachedWalks, ci, si)
				_, freshErr := boundarywalk.WalkOf(segment, &freshNear)
				require.ErrorIs(t, cachedErr, ErrUnsupported)
				require.ErrorIs(t, freshErr, ErrUnsupported)
				require.EqualError(t, cachedErr, freshErr.Error())
				require.Equal(t, freshNear, cachedNear)
			}
		}
	}
	check(t, body)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := Transform(t.Context(), body, rotation)
	require.NoError(t, err)
	check(t, placed)

	w2 := sketch.NewWorld()
	s2, err := w2.CreateSketch(w2.XY())
	require.NoError(t, err)
	a := s2.CreatePoint(0, 0)
	b := s2.CreatePoint(10, 0)
	c := s2.CreatePoint(15, 5)
	d := s2.CreatePoint(25, 5)
	s2.Fix(a)
	s2.CreateLine(a, b)
	s2.CreateArc(s2.CreatePoint(10, 5), b, c)
	s2.CreateLine(c, d)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s2.Chains(), 1)
	analytic, err := Extrude(t.Context(), New(), s2, s2.Chains()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	check(t, analytic)
}
