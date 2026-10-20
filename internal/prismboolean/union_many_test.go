package prismboolean_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func manyRect(x0, y0, x1, y1 float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func manyPrism(t *testing.T, x0, x1 float64) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 2, Profile: manyRect(x0, 0, x1, 2)}
}

func manyBody(p prismrecord.Payload) *topo.Body { return &topo.Body{Payload: p} }

// The one-scene prism Union answers false, with no error, for every operand
// set outside its class, and refuses with ErrUnsupported when two sources share
// a line carrier.
func TestUnionManyDeclinesOutsideItsClass(t *testing.T) {
	t.Parallel()

	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	tests := map[string]func(a, b *prismrecord.Payload){
		"frame differs":         func(_, b *prismrecord.Payload) { b.Frame = turned },
		"placement differs":     func(_, b *prismrecord.Payload) { b.Xform = move },
		"start level differs":   func(_, b *prismrecord.Payload) { b.Z0 = 1 },
		"end level differs":     func(_, b *prismrecord.Payload) { b.Z1 = 3 },
		"first section moved":   func(a, _ *prismrecord.Payload) { a.SectionDelta = 1e-3 },
		"second section moved":  func(_, b *prismrecord.Payload) { b.SectionDelta = 1e-3 },
		"first has a hole":      func(a, _ *prismrecord.Payload) { a.Profile.Holes = []sectionrecord.LoopRecord{a.Profile.Outer} },
		"second has a hole":     func(_, b *prismrecord.Payload) { b.Profile.Holes = []sectionrecord.LoopRecord{b.Profile.Outer} },
		"first has no segments": func(a, _ *prismrecord.Payload) { a.Profile.Outer.Segments = nil },
		"second has an arc": func(_, b *prismrecord.Payload) {
			b.Profile.Outer.Segments = append(append([]sectionrecord.CurveSegment(nil), b.Profile.Outer.Segments...),
				sectionrecord.ArcSeg{})
		},
		"second line is part of a carrier": func(_, b *prismrecord.Payload) {
			segs := append([]sectionrecord.CurveSegment(nil), b.Profile.Outer.Segments...)
			l := segs[0].(sectionrecord.LineSeg)
			l.TEnd = 0.5
			segs[0] = l
			b.Profile.Outer.Segments = segs
		},
	}
	for name, edit := range tests {
		a, b := manyPrism(t, 0, 2), manyPrism(t, 10, 12)
		edit(&a, &b)
		_, ok, err := prismboolean.UnionMany(t.Context(), topo.Env{}, []*topo.Body{manyBody(a), manyBody(b)})
		require.NoError(t, err, name)
		require.False(t, ok, name)
	}

	_, ok, err := prismboolean.UnionMany(t.Context(), topo.Env{},
		[]*topo.Body{manyBody(manyPrism(t, 0, 2)), {Payload: loftrecord.Payload{}}})
	require.NoError(t, err)
	require.False(t, ok, "a loft is not a prism")

	// Squares side by side share the carrier of their touching sides.
	_, ok, err = prismboolean.UnionMany(t.Context(), topo.Env{},
		[]*topo.Body{manyBody(manyPrism(t, 0, 2)), manyBody(manyPrism(t, 2, 4))})
	require.True(t, ok)
	require.ErrorContains(t, err, "needs distinct source line carriers")

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = prismboolean.UnionMany(cancelled, topo.Env{}, []*topo.Body{manyBody(manyPrism(t, 0, 2))})
	require.True(t, ok)
	require.ErrorIs(t, err, cancelled.Err())
}
