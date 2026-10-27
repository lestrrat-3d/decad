package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// sharedArcMeridian revolves a radius-10 meridian about the u axis: the
// half-disk over the u-axis chord from (−10, 0) to (10, 0), a ball, or with
// dome set the quarter-disk bounded by the chord from the origin to (10, 0),
// the counter-clockwise arc to (0, 10) and the line back down, a dome over
// u ≥ 0.
func sharedArcMeridian(t *testing.T, doc *decad.Document, dome bool) *decad.Body {
	t.Helper()
	const r = 10
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	o := fixed(0, 0)
	start := fixed(r, 0)
	if dome {
		top := fixed(0, r)
		s.CreateLine(o, start)
		s.CreateArc(o, start, top)
		s.CreateLine(top, o)
	} else {
		end := fixed(-r, 0)
		s.CreateLine(end, start)
		s.CreateArc(o, start, end)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// hollowBall revolves the half-annulus between radii ri and ro about the u
// axis: a hollow sphere whose meridian is one loop of two arcs and two axis
// lines.
func hollowBall(t *testing.T, doc *decad.Document, ri, ro float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	onAxis := func(u float64) *sketch.Point {
		p := s.CreatePoint(u, 0)
		s.Fix(p)
		return p
	}
	o := onAxis(0)
	outerStart, outerEnd := onAxis(ro), onAxis(-ro)
	innerStart, innerEnd := onAxis(ri), onAxis(-ri)
	s.CreateArc(o, outerStart, outerEnd)
	s.CreateLine(outerEnd, innerEnd)
	s.CreateArc(o, innerStart, innerEnd)
	s.CreateLine(innerStart, outerStart)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// torusOf revolves the circle of radius r centred at (0, rc) about the u
// axis. With half set it revolves only the half-disk of that circle on the
// far side of the line v = rc: the chord and the counter-clockwise arc from
// (r, rc) to (−r, rc).
func torusOf(t *testing.T, doc *decad.Document, rc, r float64, half bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, rc)
	s.Fix(c)
	if half {
		start, end := s.CreatePoint(r, rc), s.CreatePoint(-r, rc)
		s.Fix(start)
		s.Fix(end)
		s.CreateArc(c, start, end)
		s.CreateLine(end, start)
	} else {
		s.CreateCircle(c, r)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// Two common-axis revolves whose meridian arcs or circles lie on one exactly
// recorded carrier share that span. The Boolean classifies it in the meridian
// scene as a boundary of both operands, so the result is an exact revolve
// with sphere or torus faces. Each expected volume is closed form: a ball of
// radius R is 4πR³/3, its hemisphere 2πR³/3, a hollow hemisphere between radii
// ri and ro 2π(ro³ − ri³)/3, a torus 2π²Rr², and the outer half of that torus
// (Pappus: the half-disk's area πr²/2 swept about a centroid at R + 4r/(3π))
// π²r²R + 4πr³/3, the inner half π²r²R − 4πr³/3.
//
// Shown to fail with ExactCarriers' circular arm removed (exactSpans refusing
// every span whose losing entity is not a line): every row falls to the mesh
// path, which refuses at require.NoError with "this chord tolerance asks for
// more than 65536 facets in one revolve mesh".
func TestCommonAxisRevolveSharedArcSpan(t *testing.T) {
	t.Parallel()
	const (
		ball       = 4000 * math.Pi / 3
		hemisphere = 2000 * math.Pi / 3
	)
	type maker func(*testing.T, *decad.Document) *decad.Body
	ballOf := func(t *testing.T, doc *decad.Document) *decad.Body { return sharedArcMeridian(t, doc, false) }
	domeOf := func(t *testing.T, doc *decad.Document) *decad.Body { return sharedArcMeridian(t, doc, true) }
	shellOf := func(t *testing.T, doc *decad.Document) *decad.Body { return hollowBall(t, doc, 6, 10) }
	ringOf := func(t *testing.T, doc *decad.Document) *decad.Body { return torusOf(t, doc, 6, 3, false) }
	outerRingOf := func(t *testing.T, doc *decad.Document) *decad.Body { return torusOf(t, doc, 6, 3, true) }
	const (
		ring      = 108 * math.Pi * math.Pi
		outerRing = 54*math.Pi*math.Pi + 36*math.Pi
		innerRing = 54*math.Pi*math.Pi - 36*math.Pi
	)
	for _, tc := range []struct {
		name string
		a, b maker
		run  func(context.Context, *decad.Body, *decad.Body) (*decad.Body, error)
		want float64
		kind decad.SurfaceKind
	}{
		{name: "dome intersect ball", a: domeOf, b: ballOf, run: pairIntersect, want: hemisphere, kind: decad.KindSphere},
		{name: "ball intersect dome", a: ballOf, b: domeOf, run: pairIntersect, want: hemisphere, kind: decad.KindSphere},
		{name: "ball union dome", a: ballOf, b: domeOf, run: pairUnion, want: ball, kind: decad.KindSphere},
		{name: "ball cut dome", a: ballOf, b: domeOf, run: pairCut, want: hemisphere, kind: decad.KindSphere},
		{name: "ball union ball", a: ballOf, b: ballOf, run: pairUnion, want: ball, kind: decad.KindSphere},
		{name: "ball intersect ball", a: ballOf, b: ballOf, run: pairIntersect, want: ball, kind: decad.KindSphere},
		{name: "shell intersect dome", a: shellOf, b: domeOf, run: pairIntersect, want: 2 * math.Pi * (1000 - 216) / 3, kind: decad.KindSphere},
		{name: "shell union ball", a: shellOf, b: ballOf, run: pairUnion, want: ball, kind: decad.KindSphere},
		{name: "torus intersect outer half", a: ringOf, b: outerRingOf, run: pairIntersect, want: outerRing, kind: decad.KindTorus},
		{name: "torus union outer half", a: ringOf, b: outerRingOf, run: pairUnion, want: ring, kind: decad.KindTorus},
		{name: "torus cut outer half", a: ringOf, b: outerRingOf, run: pairCut, want: innerRing, kind: decad.KindTorus},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			result, err := tc.run(t.Context(), tc.a(t, doc), tc.b(t, doc))
			require.NoError(t, err)
			requireAnalyticVolume(t, result, tc.want)
			requireFaceKind(t, result, tc.kind)
			requireArcResultMesh(t, result)
			rep, err := doc.Verify(t.Context())
			require.NoError(t, err)
			br, err := rep.ForBody(result)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, br.Status)
		})
	}
}

// A dome inside a ball of its own radius leaves nothing when the ball is cut
// from it, and the shared span does not keep a sliver: the Cut reports
// BooleanEmpty.
func TestCommonAxisRevolveSharedArcSpanEmpty(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.ExactOnly())
	dome := sharedArcMeridian(t, doc, true)
	ball := sharedArcMeridian(t, doc, false)
	_, err := decad.Cut(t.Context(), dome, ball)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanEmpty, be.Code)
}

// A dome whose arc is centred 1e-13 mm along the axis from a ball's centre
// meets the ball's arc at both of its ends, and sketch resolves the two arcs
// onto one carrier within its identity band. Their recorded centres and radii
// differ, so the span is not one exact carrier: the common-axis route refuses
// it, and the pair continues to the mesh path's refusal.
//
// Shown to fail with exactCircleSpan's carrier comparisons removed (the
// centre and squared-radius checks): the pair is admitted and Intersect
// returns a body.
func TestCommonAxisRevolveSharedArcSpanNearCarrier(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.ExactOnly())
	ball := sharedArcMeridian(t, doc, false)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	o, start, top := fixed(0, 0), fixed(10, 0), fixed(0, 10)
	s.CreateLine(o, start)
	s.CreateArc(fixed(1e-13, 0), start, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	dome, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	before := doc.Bodies()
	_, err = decad.Intersect(t.Context(), ball, dome)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "more than 65536 facets")
	require.Equal(t, before, doc.Bodies())
}
