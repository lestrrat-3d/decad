package reportvocab_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// simplePayload is a payload whose ProvesSimple answer the test states.
type simplePayload struct{ answer bool }

func (simplePayload) Transform() r3.Transform { return r3.Identity() }
func (simplePayload) Placed(context.Context, topo.Env, r3.Transform) (*topo.Body, error) {
	return &topo.Body{}, nil
}
func (p simplePayload) ProvesSimple(context.Context) bool { return p.answer }

// bodyOf links faces into one lump with one shell.
func bodyOf(faces ...*topo.Face) *topo.Body {
	return &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
}

// loopOf returns a loop walking each edge once, in the stated directions.
func loopOf(edges []*topo.Edge, forward ...bool) *topo.Loop {
	l := &topo.Loop{Outer: true}
	for i, e := range edges {
		l.CoEdges = append(l.CoEdges, topo.CoEdge{Edge: e, Forward: forward[i]})
	}
	return l
}

// lens returns two planar faces closed along one shared edge, walked once
// forward and once backward: the smallest boundary every audit leg admits.
func lens() (*topo.Face, *topo.Face, *topo.Edge) {
	e := &topo.Edge{}
	a := &topo.Face{Surface: surfacegeom.Plane{}}
	b := &topo.Face{Surface: surfacegeom.Plane{}}
	a.Loops = []*topo.Loop{loopOf([]*topo.Edge{e}, true)}
	b.Loops = []*topo.Loop{loopOf([]*topo.Edge{e}, false)}
	e.Faces = []*topo.Face{a, b}
	return a, b, e
}

// TestAuditSolidBoundary reads each closed-skin leg off hand-built nodes: no
// faces, a loopless face, an empty loop and a one-face edge each fail, while a
// loopless sphere, torus or audited faceted image is admitted.
func TestAuditSolidBoundary(t *testing.T) {
	t.Parallel()

	a, b, _ := lens()
	require.True(t, reportvocab.AuditSolidBoundary(bodyOf(a, b), false), "two faces closing one edge")
	require.False(t, reportvocab.AuditSolidBoundary(bodyOf(), true), "a body with no faces")

	t.Run("an edge bounding one face", func(t *testing.T) {
		t.Parallel()
		a, b, e := lens()
		e.Faces = e.Faces[:1]
		require.False(t, reportvocab.AuditSolidBoundary(bodyOf(a, b), false))
	})
	t.Run("an empty loop", func(t *testing.T) {
		t.Parallel()
		a, b, _ := lens()
		a.Loops = append(a.Loops, &topo.Loop{})
		require.False(t, reportvocab.AuditSolidBoundary(bodyOf(a, b), false))
	})
	t.Run("loopless faces", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			name    string
			surface surfacegeom.Surface
			faceted bool
			want    bool
		}{
			{name: "plane", surface: surfacegeom.Plane{}, faceted: true, want: false},
			{name: "sphere", surface: surfacegeom.Sphere{Radius: units.Millimeters(1)}, want: true},
			{name: "torus", surface: surfacegeom.Torus{Major: units.Millimeters(2), Minor: units.Millimeters(1)}, want: true},
			{name: "audited faceted image", surface: surfacegeom.Faceted{}, faceted: true, want: true},
			{name: "unaudited faceted image", surface: surfacegeom.Faceted{}, faceted: false, want: false},
		} {
			got := reportvocab.AuditSolidBoundary(bodyOf(&topo.Face{Surface: tc.surface}), tc.faceted)
			require.Equal(t, tc.want, got, tc.name)
		}
	})
	t.Run("a faceted image is admitted only as the one face", func(t *testing.T) {
		t.Parallel()
		image := func() *topo.Face { return &topo.Face{Surface: surfacegeom.Faceted{}} }
		require.False(t, reportvocab.AuditSolidBoundary(bodyOf(image(), image()), true))
	})
}

// TestAuditSheetBoundary reads Table V's legs off hand-built nodes: each
// structural failure is Violated, and a sound structure is Proven only when the
// payload proves itself simple.
func TestAuditSheetBoundary(t *testing.T) {
	t.Parallel()
	ctx := t.Context()

	t.Run("leg 4 follows the payload", func(t *testing.T) {
		t.Parallel()
		a, b, _ := lens()
		body := bodyOf(a, b)
		require.Equal(t, reportvocab.SheetAuditUndecided, reportvocab.AuditSheetBoundary(ctx, body), "no payload")
		body.Payload = simplePayload{answer: false}
		require.Equal(t, reportvocab.SheetAuditUndecided, reportvocab.AuditSheetBoundary(ctx, body))
		body.Payload = simplePayload{answer: true}
		require.Equal(t, reportvocab.SheetAuditProven, reportvocab.AuditSheetBoundary(ctx, body))
	})
	t.Run("a free edge is admitted", func(t *testing.T) {
		t.Parallel()
		e := &topo.Edge{}
		f := &topo.Face{Loops: []*topo.Loop{loopOf([]*topo.Edge{e}, true)}}
		e.Faces = []*topo.Face{f}
		body := bodyOf(f)
		body.Payload = simplePayload{answer: true}
		require.Equal(t, reportvocab.SheetAuditProven, reportvocab.AuditSheetBoundary(ctx, body))
	})
	for _, tc := range []struct {
		name  string
		build func(a, b *topo.Face, e *topo.Edge) *topo.Body
	}{
		{name: "no faces", build: func(*topo.Face, *topo.Face, *topo.Edge) *topo.Body { return bodyOf() }},
		{name: "a loopless sphere", build: func(a, b *topo.Face, _ *topo.Edge) *topo.Body {
			return bodyOf(a, b, &topo.Face{Surface: surfacegeom.Sphere{Radius: units.Millimeters(1)}})
		}},
		{name: "an empty loop", build: func(a, b *topo.Face, _ *topo.Edge) *topo.Body {
			a.Loops = append(a.Loops, &topo.Loop{})
			return bodyOf(a, b)
		}},
		{name: "a non-manifold edge", build: func(a, b *topo.Face, e *topo.Edge) *topo.Body {
			// The third face walks e too, so the use count agrees with the
			// face count and only the manifold leg can refuse.
			c := &topo.Face{Loops: []*topo.Loop{loopOf([]*topo.Edge{e}, true)}}
			e.Faces = append(e.Faces, c)
			return bodyOf(a, b, c)
		}},
		{name: "an edge with no faces", build: func(a, b *topo.Face, e *topo.Edge) *topo.Body {
			e.Faces = nil
			return bodyOf(a, b)
		}},
		{name: "both uses forward", build: func(a, b *topo.Face, e *topo.Edge) *topo.Body {
			b.Loops = []*topo.Loop{loopOf([]*topo.Edge{e}, true)}
			return bodyOf(a, b)
		}},
		{name: "a use count beside the face count", build: func(a, b *topo.Face, e *topo.Edge) *topo.Body {
			e.Faces = e.Faces[:1]
			return bodyOf(a, b)
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			a, b, e := lens()
			body := tc.build(a, b, e)
			body.Payload = simplePayload{answer: true}
			require.Equal(t, reportvocab.SheetAuditViolated, reportvocab.AuditSheetBoundary(ctx, body))
		})
	}
}

// TestUndecidedPairReason maps each unmeasured-overlap outcome to its own code,
// and names a shared face plane only on a contact refusal whose operands have
// one.
func TestUndecidedPairReason(t *testing.T) {
	t.Parallel()

	planeAt := func(z float64) *topo.Body {
		frame, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		return bodyOf(&topo.Face{Surface: surfacegeom.Plane{Frame: frame}})
	}
	low, high, flipped := planeAt(0), planeAt(1), planeAt(0)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	flipped.Lumps[0].Shells[0].Faces[0].Surface = surfacegeom.Plane{Frame: frame}

	for _, tc := range []struct {
		outcome reportvocab.InterferenceOutcome
		verdict reportvocab.PairVerdict
		code    reportvocab.DiagnosticCode
		first   bool
	}{
		{outcome: reportvocab.InterferenceUnsupportedPayloadFirst, code: reportvocab.DiagUnsupportedPairPayload, first: true},
		{outcome: reportvocab.InterferenceUnsupportedPayloadSecond, code: reportvocab.DiagUnsupportedPairPayload},
		{outcome: reportvocab.InterferenceUnsupportedVolumeProofFirst, code: reportvocab.DiagUnsupportedPairPayload, first: true},
		{outcome: reportvocab.InterferenceUnsupportedVolumeProofSecond, code: reportvocab.DiagUnsupportedPairPayload},
		{outcome: reportvocab.InterferenceUnsupportedPipeline, code: reportvocab.DiagUnsupportedPairPipeline},
		{outcome: reportvocab.InterferenceUndecided, verdict: reportvocab.PairOverlapping, code: reportvocab.DiagUndecidedInterference},
		{outcome: reportvocab.InterferenceUndecided, verdict: reportvocab.PairUndecided, code: reportvocab.DiagUndecidedPair},
		{outcome: reportvocab.InterferenceMeasured, verdict: reportvocab.PairOverlapping, code: reportvocab.DiagUndecidedPair},
	} {
		code, msg := reportvocab.UndecidedPairReason(low, high, tc.verdict, tc.outcome)
		require.Equal(t, tc.code, code, "outcome %d", tc.outcome)
		require.NotEmpty(t, msg)
		if tc.code == reportvocab.DiagUnsupportedPairPayload {
			if tc.first {
				require.Contains(t, msg, "the first operand")
			} else {
				require.Contains(t, msg, "the second operand")
			}
		}
	}

	const shared = "the operands share a face plane"
	code, msg := reportvocab.UndecidedPairReason(low, high, reportvocab.PairUndecided, reportvocab.InterferenceUnsupportedContact)
	require.Equal(t, reportvocab.DiagUnsupportedPairContact, code)
	require.NotContains(t, msg, shared, "planes one apart")
	_, msg = reportvocab.UndecidedPairReason(low, flipped, reportvocab.PairUndecided, reportvocab.InterferenceUnsupportedContact)
	require.Contains(t, msg, shared, "one plane facing both ways")
	_, msg = reportvocab.UndecidedPairReason(low, planeAt(0), reportvocab.PairUndecided, reportvocab.InterferenceUnsupportedPipeline)
	require.NotContains(t, msg, shared, "only a contact refusal names the plane")
}
