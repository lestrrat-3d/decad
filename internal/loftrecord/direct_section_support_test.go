package loftrecord

// CI: isolated-tests

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDirectSectionLineSupportsBoundary(t *testing.T) {
	point := func(u, v float64) momentinput.Point2 { return momentinput.Point2{U: u, V: v} }
	lines := []momentinput.CurveSegment{
		momentinput.LineSeg{Start: point(0, 3), End: point(0, 1), TEnd: 1},
		momentinput.ArcSeg{Center: point(1, 1), Start: point(0, 1), End: point(1, 0), TEnd: 1},
		momentinput.LineSeg{Start: point(1, 0), End: point(3, 0), TEnd: 1},
		momentinput.ArcSeg{Center: point(3, 1), Start: point(3, 0), End: point(4, 1), TEnd: 1},
		momentinput.LineSeg{Start: point(4, 1), End: point(4, 3), TEnd: 1},
		momentinput.ArcSeg{Center: point(3, 3), Start: point(4, 3), End: point(3, 4), TEnd: 1},
		momentinput.LineSeg{Start: point(3, 4), End: point(1, 4), TEnd: 1},
		momentinput.ArcSeg{Center: point(1, 3), Start: point(1, 4), End: point(0, 3), TEnd: 1},
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	body := new(topo.Body)
	body.Origin.Producer = 1
	makePayload := func(segments []momentinput.CurveSegment) Payload {
		profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: segments}}
		p := Payload{Profile0: profile, Profile1: profile, Frame0: frame, Frame1: frame,
			AuthenticatedReconstruction: true, Xform: r3.Identity(), Alignment: []int{0}}
		p.SourceIdentity = &SourceIdentity{Origin: 1, direct: sealDirectSections(body, p)}
		return p
	}
	for _, tc := range []struct {
		name string
		edit func([]momentinput.CurveSegment)
		want bool
	}{
		{"rounded rectangle", nil, true},
		{"line crosses support", func(s []momentinput.CurveSegment) {
			s[4] = momentinput.LineSeg{Start: point(-1, 1), End: point(4, 3), TEnd: 1}
		}, false},
		{"arc crosses support", func(s []momentinput.CurveSegment) {
			s[1] = momentinput.ArcSeg{Center: point(1, 1), Start: point(-1, 1), End: point(1, -1), TEnd: 1}
		}, false},
		{"tangent away from source", func(s []momentinput.CurveSegment) {
			s[1] = momentinput.ArcSeg{Center: point(1, 0), Start: point(0, 0), End: point(1, -1), TEnd: 1}
		}, false},
		{"unequal arc radii", func(s []momentinput.CurveSegment) {
			s[1] = momentinput.ArcSeg{Center: point(1, 1), Start: point(0, 1), End: point(1, math.Nextafter(0, -1)), TEnd: 1}
		}, false},
		{"partial arc", func(s []momentinput.CurveSegment) {
			arc := s[1].(momentinput.ArcSeg)
			arc.TStart = 0.5
			s[1] = arc
		}, false},
		{"nonfinite", func(s []momentinput.CurveSegment) {
			s[4] = momentinput.LineSeg{Start: point(math.NaN(), 1), End: point(4, 3), TEnd: 1}
		}, false},
		{"extra collinear segment", func(s []momentinput.CurveSegment) {
			s[4] = momentinput.LineSeg{Start: point(0, 1), End: point(0, 3), TEnd: 1}
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			segments := append([]momentinput.CurveSegment(nil), lines...)
			if tc.edit != nil {
				tc.edit(segments)
			}
			p := makePayload(segments)
			got, err := p.DirectSectionLineSupportsBoundary(t.Context(), body, 0, 0, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
	p := makePayload(append([]momentinput.CurveSegment(nil), lines...))
	_, err = p.DirectSectionLineSupportsBoundary(nil, body, 0, 0, proofbound.NewWorkBudget(t.Context())) //nolint:staticcheck // Exercise the nil-context refusal.
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = p.DirectSectionLineSupportsBoundary(ctx, body, 0, 0, proofbound.NewWorkBudget(ctx))
	require.ErrorIs(t, err, context.Canceled)
	budget := &proofbound.WorkBudget{StepFn: func() error { return proofbound.ErrWallWorkBudget }, ErrFn: func() error { return nil }}
	_, err = p.DirectSectionLineSupportsBoundary(t.Context(), body, 0, 0, budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	p.Profile0.Outer.Segments = append([]sectionrecord.CurveSegment(nil), p.Profile0.Outer.Segments...)
	p.Profile0.Outer.Segments[0] = momentinput.LineSeg{Start: point(0, 4), End: point(0, 1), TEnd: 1}
	got, err := p.DirectSectionLineSupportsBoundary(t.Context(), body, 0, 0, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, got)
}
