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
				ctx := newCancelAfter(t.Context(), int64(n-1))
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
		{"PatchProfile", func(ctx context.Context) error { _, err := decad.PatchProfile(doc, nil, nil).Do(ctx); return err }},
		{"Document.Loft", func(ctx context.Context) error { _, err := decad.Loft(doc, nil, nil, nil, nil).Do(ctx); return err }},
		{"LoftChain", func(ctx context.Context) error {
			_, err := decad.LoftChain(doc, nil, nil, nil, nil).Do(ctx)
			return err
		}},
		{"Sweep profile", func(ctx context.Context) error {
			_, err := decad.Sweep(doc, nil, (*sketch.Profile)(nil), nil).Do(ctx)
			return err
		}},
		{"Sweep chain", func(ctx context.Context) error {
			_, err := decad.Sweep(doc, nil, (*sketch.Chain)(nil), nil).Do(ctx)
			return err
		}},
		{"Extrude profile", func(ctx context.Context) error {
			_, err := decad.Extrude(doc, nil, (*sketch.Profile)(nil), nil).Do(ctx)
			return err
		}},
		{"Extrude chain", func(ctx context.Context) error {
			_, err := decad.Extrude(doc, nil, (*sketch.Chain)(nil), nil).Do(ctx)
			return err
		}},
		{"Revolve profile", func(ctx context.Context) error {
			_, err := decad.Revolve(doc, nil, (*sketch.Profile)(nil), nil, nil).Do(ctx)
			return err
		}},
		{"Revolve chain", func(ctx context.Context) error {
			_, err := decad.Revolve(doc, nil, (*sketch.Chain)(nil), nil, nil).Do(ctx)
			return err
		}},
		{"Copy identity", func(ctx context.Context) error { _, err := decad.Copy(body).Do(ctx); return err }},
		{"Copy placement", func(ctx context.Context) error {
			_, err := decad.Copy(body).Place(decad.CopyTransform{Transform: r3.Transform{}}).Do(ctx)
			return err
		}},
		{"Body.Fillet", func(ctx context.Context) error { _, err := decad.Fillet(body, nil, units.Value{}).Do(ctx); return err }},
		{"Body.Chamfer", func(ctx context.Context) error { _, err := decad.Chamfer(body, nil, units.Value{}).Do(ctx); return err }},
		{"Body.Shell", func(ctx context.Context) error { _, err := decad.Shell(body, nil, units.Value{}).Do(ctx); return err }},
		{"Body.FilletChord", func(ctx context.Context) error {
			_, err := decad.FilletChord(body, nil, units.Value{}).Do(ctx)
			return err
		}},
		{"Body.ChamferAngle", func(ctx context.Context) error {
			_, err := decad.ChamferAngle(body, nil, units.Value{}, nil, units.Value{}).Do(ctx)
			return err
		}},
		{"Body.ShellClosed", func(ctx context.Context) error {
			_, err := decad.ShellClosed(body, units.Value{}).Do(ctx)
			return err
		}},
		{"Body.Draft", func(ctx context.Context) error {
			_, err := decad.Draft(body, nil, decad.NeutralFace{}, units.Value{}).Do(ctx)
			return err
		}},
		{"Body.MoveFace", func(ctx context.Context) error {
			_, err := decad.MoveFace(body, nil, units.Value{}).Do(ctx)
			return err
		}},
		{"Body.ConeFromFace", func(ctx context.Context) error {
			_, err := decad.ConeFromFace(body, nil, units.Value{}).Do(ctx)
			return err
		}},
		{"Thicken", func(ctx context.Context) error { _, err := decad.Thicken(body, units.Value{}).Do(ctx); return err }},
		{"Offset", func(ctx context.Context) error { _, err := decad.Offset(body, units.Value{}).Do(ctx); return err }},
		{"Body.Tessellate", func(ctx context.Context) error { _, err := body.Tessellate(ctx, units.Value{}); return err }},
		{"Stitch", func(ctx context.Context) error { _, err := decad.Stitch().Do(ctx); return err }},
		{"Union", func(ctx context.Context) error { _, err := decad.Union(nil, nil).Do(ctx); return err }},
		{"Cut", func(ctx context.Context) error { _, err := decad.Cut(nil, nil).Do(ctx); return err }},
		{"Intersect", func(ctx context.Context) error { _, err := decad.Intersect(nil, nil).Do(ctx); return err }},
		{"Refine", func(ctx context.Context) error {
			_, err := decad.Refine(nil, units.Radians(1)).Do(ctx)
			return err
		}},
		{"Union.RefineTo", func(ctx context.Context) error {
			_, err := decad.Union(nil).RefineTo(units.Radians(1)).Do(ctx)
			return err
		}},
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
			return decad.Revolve(doc, profileSketch, profile, profileAxis, decad.FullRevolution{}).Do(ctx)
		}},
		{"chain", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Revolve(doc, chainSketch, chain, chainAxis, decad.FullRevolution{}).Do(ctx)
		}},
	}
	requireCancellationLeavesDocumentUnchanged(t, tests)
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
			return decad.Extrude(doc, profileSketch, profile, e).Do(ctx)
		}},
		{"tapered profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(doc, profileSketch, profile, e).Taper(units.Degrees(5)).Do(ctx)
		}},
		{"two-sided tapered profile", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(doc, profileSketch, profile, decad.Symmetric{D: units.Millimeters(5)}).Taper(units.Degrees(5)).Do(ctx)
		}},
		{"chain", func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return decad.Extrude(doc, chainSketch, chain, e).Do(ctx)
		}},
	}
	requireCancellationLeavesDocumentUnchanged(t, tests)
}
