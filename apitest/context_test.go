package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type documentBodyCall struct {
	name string
	call func(context.Context, *decad.Document) (*decad.Body, error)
}

func requireCancellationLeavesDocumentUnchanged(t *testing.T, tests []documentBodyCall) {
	t.Helper()
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			canceled, cancel := context.WithCancel(t.Context())
			cancel()
			_, err := tc.call(canceled, doc)
			require.Equal(t, context.Canceled, err)
			require.Empty(t, doc.Bodies())

			const maxChecks = 1000
			for n := 1; n <= maxChecks; n++ {
				ctx := &cancelAtCheckContext{Context: t.Context(), n: n}
				body, err := tc.call(ctx, doc)
				if err == nil {
					require.NotNil(t, body)
					require.Greater(t, n, 3, "the call checks its context at several steps before it commits")
					require.Len(t, doc.Bodies(), 1)
					return
				}
				require.Equal(t, context.Canceled, err, "check %d", n)
				require.Empty(t, doc.Bodies(), "check %d", n)
			}
			require.Fail(t, "the call never completed")
		})
	}
}

// Every context-taking operation refuses a nil context before inspecting its
// document, operands, or options.
func TestContextOperationsRejectNilBeforeOperands(t *testing.T) {
	t.Parallel()
	var doc *decad.Document
	var body *decad.Body
	tests := []struct {
		name string
		call func(context.Context) error
	}{
		{"Document.Verify", func(ctx context.Context) error { _, err := doc.Verify(ctx); return err }},
		{"Document.VerifyMotion", func(ctx context.Context) error { _, err := doc.VerifyMotion(ctx, nil, nil); return err }},
		{"PatchProfile", func(ctx context.Context) error { _, err := decad.PatchProfile(ctx, doc, nil, nil); return err }},
		{"Document.Loft", func(ctx context.Context) error { _, err := decad.Loft(ctx, doc, nil, nil, nil, nil); return err }},
		{"LoftChain", func(ctx context.Context) error { _, err := decad.LoftChain(ctx, doc, nil, nil, nil, nil); return err }},
		{"Sweep profile", func(ctx context.Context) error {
			_, err := decad.Sweep(ctx, doc, nil, (*sketch.Profile)(nil), nil)
			return err
		}},
		{"Sweep chain", func(ctx context.Context) error {
			_, err := decad.Sweep(ctx, doc, nil, (*sketch.Chain)(nil), nil)
			return err
		}},
		{"Extrude profile", func(ctx context.Context) error {
			_, err := decad.Extrude(ctx, doc, nil, (*sketch.Profile)(nil), nil)
			return err
		}},
		{"Extrude chain", func(ctx context.Context) error {
			_, err := decad.Extrude(ctx, doc, nil, (*sketch.Chain)(nil), nil)
			return err
		}},
		{"Revolve profile", func(ctx context.Context) error {
			_, err := decad.Revolve(ctx, doc, nil, (*sketch.Profile)(nil), nil, nil)
			return err
		}},
		{"Revolve chain", func(ctx context.Context) error {
			_, err := decad.Revolve(ctx, doc, nil, (*sketch.Chain)(nil), nil, nil)
			return err
		}},
		{"Split", func(ctx context.Context) error { _, err := decad.Split(ctx, nil, nil); return err }},
		{"Transform", func(ctx context.Context) error { _, err := decad.Transform(ctx, body, r3.Transform{}); return err }},
		{"Copy identity", func(ctx context.Context) error { _, err := decad.Copy(ctx, body); return err }},
		{"Copy placement", func(ctx context.Context) error {
			_, err := decad.Copy(ctx, body, decad.WithCopyPlacement(r3.Transform{}))
			return err
		}},
		{"PatchEdges", func(ctx context.Context) error { _, err := decad.PatchEdges(ctx, body, nil); return err }},
		{"Body.Fillet", func(ctx context.Context) error { _, err := decad.Fillet(ctx, body, nil, units.Value{}); return err }},
		{"Body.Chamfer", func(ctx context.Context) error { _, err := decad.Chamfer(ctx, body, nil, units.Value{}); return err }},
		{"Body.Shell", func(ctx context.Context) error { _, err := decad.Shell(ctx, body, nil, units.Value{}); return err }},
		{"Thicken", func(ctx context.Context) error { _, err := decad.Thicken(ctx, body, units.Value{}); return err }},
		{"Offset", func(ctx context.Context) error { _, err := decad.Offset(ctx, body, units.Value{}); return err }},
		{"Unstitch", func(ctx context.Context) error { _, err := decad.Unstitch(ctx, body); return err }},
		{"Body.Tessellate", func(ctx context.Context) error { _, err := body.Tessellate(ctx, units.Value{}); return err }},
		{"Trim", func(ctx context.Context) error { _, err := decad.Trim(ctx, body, nil, 0); return err }},
		{"Extend", func(ctx context.Context) error { _, err := decad.Extend(ctx, body, nil, nil); return err }},
		{"Stitch", func(ctx context.Context) error { _, err := decad.Stitch(ctx); return err }},
		{"Union", func(ctx context.Context) error { _, err := decad.Union(ctx, nil, nil); return err }},
		{"Cut", func(ctx context.Context) error { _, err := decad.Cut(ctx, nil, nil); return err }},
		{"Intersect", func(ctx context.Context) error { _, err := decad.Intersect(ctx, nil, nil); return err }},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			var nilContext context.Context
			err := tc.call(nilContext)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.ErrorContains(t, err, "nil context")
		})
	}

	t.Run("PopulatedDocument", func(t *testing.T) {
		populated, mover := extrudePlate(t)
		before := populated.Bodies()
		var nilContext context.Context
		report, err := populated.Verify(nilContext)
		require.Nil(t, report)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "nil context")

		motion := decad.Prismatic{
			Dir:  r3.NewVec(1, 0, 0),
			From: units.Millimeters(0),
			To:   units.Millimeters(1),
		}
		motionReport, err := populated.VerifyMotion(nilContext, []*decad.Body{mover}, motion)
		require.Nil(t, motionReport)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "nil context")
		require.Equal(t, before, populated.Bodies())
	})
}

// TestRevolveTypedNilSectionIsDegenerate pins that a typed-nil section of
// either kind returns ErrDegenerate and leaves the document unchanged.
func TestRevolveTypedNilSectionIsDegenerate(t *testing.T) {
	t.Parallel()
	s, _, axis := coneChainAtRadius(t, 0)
	doc := decad.New()

	_, err := decad.Revolve(t.Context(), doc, s, (*sketch.Profile)(nil), axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Revolve(t.Context(), doc, s, (*sketch.Chain)(nil), axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}

// cancelAtCheckContext reports context.Canceled from its n-th Err call on, so
// a test can cancel an operation at each of its cancellation checks in turn.
type cancelAtCheckContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	n               int
	calls           int
}

func (c *cancelAtCheckContext) Err() error {
	c.calls++
	if c.calls >= c.n {
		return context.Canceled
	}
	return nil
}

// TestRevolveCanceledContextLeavesDocumentUnchanged cancels a profile and a
// chain Revolve before the call and at each of its cancellation checks. Every
// cancellation returns context.Canceled unchanged and leaves Bodies() empty;
// the first uncanceled run commits one body, after several checks ran.
func TestRevolveCanceledContextLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()
	profileSketch, profile := solidSketch(t)
	chainSketch, chain, chainAxis := coneChainAtRadius(t, 0)
	profileAxis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	tests := []documentBodyCall{
		{"profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Revolve(ctx, doc, profileSketch, profile, profileAxis, decad.FullRevolution{})
		}},
		{"chain", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Revolve(ctx, doc, chainSketch, chain, chainAxis, decad.FullRevolution{})
		}},
	}
	requireCancellationLeavesDocumentUnchanged(t, tests)
}

// TestExtrudeTypedNilSectionIsDegenerate pins that a typed-nil section of
// either kind returns ErrDegenerate and leaves the document unchanged.
func TestExtrudeTypedNilSectionIsDegenerate(t *testing.T) {
	t.Parallel()
	s, _ := solidSketch(t)
	doc := decad.New()
	e := decad.Distance{D: units.Millimeters(5), Dir: decad.Along}

	_, err := decad.Extrude(t.Context(), doc, s, (*sketch.Profile)(nil), e)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Extrude(t.Context(), doc, s, (*sketch.Chain)(nil), e)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}

// TestExtrudeCanceledContextLeavesDocumentUnchanged cancels a profile, a
// tapered profile and a chain Extrude before the call and at each of its
// cancellation checks. Every cancellation returns context.Canceled unchanged
// and leaves Bodies() empty; the first uncanceled run commits one body, after
// several checks ran.
func TestExtrudeCanceledContextLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()
	profileSketch, profile := solidSketch(t)
	chainSketch, chain := lineChainSketch(t)
	e := decad.Distance{D: units.Millimeters(5), Dir: decad.Along}
	tests := []documentBodyCall{
		{"profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(ctx, doc, profileSketch, profile, e)
		}},
		{"tapered profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(ctx, doc, profileSketch, profile, e, decad.WithTaper(units.Degrees(5)))
		}},
		{"two-sided tapered profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(ctx, doc, profileSketch, profile, decad.Symmetric{D: units.Millimeters(5)},
				decad.WithTaper(units.Degrees(5)))
		}},
		{"chain", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(ctx, doc, chainSketch, chain, e)
		}},
	}
	requireCancellationLeavesDocumentUnchanged(t, tests)
}
