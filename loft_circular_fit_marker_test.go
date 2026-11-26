package decad

import (
	"context"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type cancelAfterFirstProfile struct {
	checks int
	done   chan struct{}
}

func (c *cancelAfterFirstProfile) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *cancelAfterFirstProfile) Done() <-chan struct{}       { return c.done }
func (c *cancelAfterFirstProfile) Value(any) any               { return nil }

func (c *cancelAfterFirstProfile) Err() error {
	c.checks++
	if c.checks > 1 {
		if c.checks == 2 {
			close(c.done)
		}
		return context.Canceled
	}
	return nil
}

func TestCircularFitPrimitiveEligibility(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sections := make([]LoftSection, 4)
	for i := range sections {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i))
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		p := []*sketch.Point{point(1, 0), point(0, 1), point(-1, 0), point(0, -1)}
		if i < 2 {
			origin := point(0, 0)
			for j := range p {
				s.CreateArc(origin, p[j], p[(j+1)%4])
			}
		} else {
			_, err = s.CreateFitSpline(p[0], point(0.5, 0.5), p[1])
			require.NoError(t, err)
			for j := 1; j < 4; j++ {
				s.CreateLine(p[j], p[(j+1)%4])
			}
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Outer) == 4 && len(profile.Holes) == 0 {
				sections[i] = LoftSection{Sketch: s, Profile: profile}
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	doc := New()
	cancelled := &cancelAfterFirstProfile{done: make(chan struct{})}
	_, err := doc.buildCircularFitSections(cancelled, sections, nil, false)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 2, cancelled.checks)
	require.Empty(t, doc.Bodies())
	body, err := LoftSections(t.Context(), doc, sections...)
	require.NoError(t, err)
	payload, ok := body.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.True(t, payload.CircularFitPrimitive)
	require.Equal(t, payload.MeshBound, booloperand.ChordShare(t.Context(), &body.n).Floor)
	shift, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	placed, err := Copy(t.Context(), body, WithCopyPlacement(shift))
	require.NoError(t, err)
	placedPayload, ok := placed.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.True(t, placedPayload.CircularFitPrimitive)
	require.Equal(t, placedPayload.MeshBound, booloperand.ChordShare(t.Context(), &placed.n).Floor)

	plane, err := w.CreateOffsetPlane(w.YZ(), -2)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0.5)
	s.Fix(c)
	s.CreateCircle(c, 0.1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	cut, err := Cut(body, tool).Do(t.Context())
	require.NoError(t, err)
	result, ok := cut.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.False(t, result.CircularFitPrimitive)
	require.Zero(t, booloperand.ChordShare(t.Context(), &cut.n).Floor)
}
