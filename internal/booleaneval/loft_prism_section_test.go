package booleaneval

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMatchingLoftPrismSection(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	lp := f.body.Payload.(loftrecord.Payload)
	pp := prismrecord.Payload{Profile: lp.Profile1, Frame: lp.Frame1, Xform: r3.Identity(), Z1: 3}
	prism, err := prismrecord.Build(t.Context(), topo.Env{Doc: new(topo.Document), Ref: 2, Alloc: topotest.Alloc{}},
		pp, freeform.NewFreeformWork())
	require.NoError(t, err)
	check := func(ctx context.Context, loft, prism *topo.Body) bool {
		ok, err := matchingLoftPrismSection(ctx, loft, prism, proofbound.NewStrictReplayBudget())
		require.NoError(t, err)
		return ok
	}
	require.True(t, check(t.Context(), f.body, prism))
	separated, err := separatedLoftPrismSections(t.Context(), f.body, prism, proofbound.NewStrictReplayBudget())
	require.NoError(t, err)
	require.True(t, separated)
	wider := pp
	wider.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), pp.Profile.Outer.Segments...)
	for i, segment := range wider.Profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		require.True(t, ok)
		line.Start.U *= 2
		line.End.U *= 2
		wider.Profile.Outer.Segments[i] = line
	}
	other, err := prismrecord.Build(t.Context(), topo.Env{Doc: new(topo.Document), Ref: 3, Alloc: topotest.Alloc{}},
		wider, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.False(t, check(t.Context(), f.body, other))
	changedLoft := lp
	changedLoft.Frame1 = lp.Frame0
	f.body.Payload = changedLoft
	require.False(t, check(t.Context(), f.body, prism))
	f.body.Payload = lp
	require.False(t, check(t.Context(), new(topo.Body), prism))
	require.False(t, check(t.Context(), f.body, new(topo.Body)))
	actual := prism.Payload.(prismrecord.Payload)
	changed := actual
	changed.Z0 = 0.01
	prism.Payload = changed
	require.False(t, check(t.Context(), f.body, prism))
	separated, err = separatedLoftPrismSections(t.Context(), f.body, prism, proofbound.NewStrictReplayBudget())
	require.NoError(t, err)
	require.False(t, separated)
	changed = actual
	changed.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), actual.Profile.Outer.Segments...)
	line := changed.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
	line.End.U += 0.01
	changed.Profile.Outer.Segments[0] = line
	prism.Payload = changed
	require.False(t, check(t.Context(), f.body, prism))
	prism.Payload = actual
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = matchingLoftPrismSection(canceled, f.body, prism, proofbound.NewStrictReplayBudget())
	require.ErrorIs(t, err, context.Canceled)
}
