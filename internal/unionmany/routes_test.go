package unionmany_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/unionmany"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type unusedMesher struct{ t *testing.T }

func (m unusedMesher) VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error) {
	m.t.Fatal("a line-only prism set must not request the mesh routes")
	return nil, decaderr.ErrUnsupported
}

func rectangle(t *testing.T, env topo.Env, x0, y0, x1, y1, height float64) *topo.Body {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	points := []sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	segments := make([]sectionrecord.CurveSegment, len(points))
	for i, point := range points {
		segments[i] = sectionrecord.LineSeg{Start: point, End: points[(i+1)%len(points)], TEnd: 1}
	}
	body, err := prismrecord.Build(t.Context(), env, prismrecord.Payload{
		Frame: frame, Xform: r3.Identity(), Z1: height,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}},
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	return body
}

func TestRoutesRealPrisms(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Alloc: topotest.Alloc{}}
	build := func(ref provenance.Producer, x0, y0, x1, y1, height float64) *topo.Body {
		env.Ref = ref
		return rectangle(t, env, x0, y0, x1, y1, height)
	}
	operands := []*topo.Body{
		build(1, 0, 0, 4, 4, 2),
		build(2, 3, 1, 6, 3, 2),
		build(3, 1, 3.5, 2.5, 6, 2),
	}
	// The explicit producer must supply result roles, even if Env carried an
	// earlier transient producer. No input is registered or consumed here.
	env.Ref = 999
	joined, span, err := unionmany.Routes(t.Context(), env, 11, operands, unusedMesher{t})
	require.NoError(t, err)
	require.Equal(t, provenance.Producer(1), span)
	require.True(t, joined.Doc == env.Doc)
	require.Equal(t, provenance.Producer(11), joined.Origin.Producer)
	require.InDelta(t, 46, joined.Volume.Value.Base(), joined.Volume.Bound.Base()+1e-12)
	for i, operand := range operands {
		require.Equal(t, provenance.Producer(i+1), operand.Origin.Producer)
	}
	require.Equal(t, 32.0, operands[0].Volume.Value.Base())
	require.Equal(t, 12.0, operands[1].Volume.Value.Base())
	require.Equal(t, 7.5, operands[2].Volume.Value.Base())

	t.Run("a certified route refusal stops dispatch", func(t *testing.T) {
		sharing := []*topo.Body{operands[0], build(4, 5, 0, 7, 4, 2), operands[2]}
		body, span, err := unionmany.Routes(t.Context(), env, 12, sharing, unusedMesher{t})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "needs distinct source line carriers")
		require.True(t, body == nil)
		require.Zero(t, span)
	})
	t.Run("every certified route declines", func(t *testing.T) {
		mixed := []*topo.Body{operands[0], operands[1], build(5, 1, 3.5, 2.5, 6, 3)}
		body, span, err := unionmany.Routes(t.Context(), env, 12, mixed, unusedMesher{t})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "Union of three or more bodies needs a certified prism")
		require.True(t, body == nil)
		require.Zero(t, span)
	})
	t.Run("cancellation propagates", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		body, span, err := unionmany.Routes(ctx, env, 12, operands, unusedMesher{t})
		require.ErrorIs(t, err, context.Canceled)
		require.True(t, body == nil)
		require.Zero(t, span)
	})
}
