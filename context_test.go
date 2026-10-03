package decad_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Every context-taking operation refuses a nil context before inspecting its
// receiver, operands, or options.
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
		{"Document.Patch", func(ctx context.Context) error { _, err := doc.Patch(ctx, nil, nil); return err }},
		{"Document.Loft", func(ctx context.Context) error { _, err := doc.Loft(ctx, nil, nil, nil, nil); return err }},
		{"Document.LoftChain", func(ctx context.Context) error { _, err := doc.LoftChain(ctx, nil, nil, nil, nil); return err }},
		{"Document.Sweep", func(ctx context.Context) error { _, err := doc.Sweep(ctx, nil, nil, nil); return err }},
		{"Document.SweepChain", func(ctx context.Context) error { _, err := doc.SweepChain(ctx, nil, nil, nil); return err }},
		{"Document.Split", func(ctx context.Context) error { _, err := doc.Split(ctx, nil, nil); return err }},
		{"Body.Placed", func(ctx context.Context) error { _, err := body.Placed(ctx, r3.Transform{}); return err }},
		{"Body.Duplicate", func(ctx context.Context) error { _, err := body.Duplicate(ctx); return err }},
		{"Body.PlacedCopy", func(ctx context.Context) error { _, err := body.PlacedCopy(ctx, r3.Transform{}); return err }},
		{"Body.Patch", func(ctx context.Context) error { _, err := body.Patch(ctx, nil); return err }},
		{"Body.Fillet", func(ctx context.Context) error { _, err := body.Fillet(ctx, nil, units.Value{}); return err }},
		{"Body.Chamfer", func(ctx context.Context) error { _, err := body.Chamfer(ctx, nil, units.Value{}); return err }},
		{"Body.Shell", func(ctx context.Context) error { _, err := body.Shell(ctx, nil, units.Value{}); return err }},
		{"Body.Thicken", func(ctx context.Context) error { _, err := body.Thicken(ctx, units.Value{}); return err }},
		{"Body.Offset", func(ctx context.Context) error { _, err := body.Offset(ctx, units.Value{}); return err }},
		{"Body.Unstitch", func(ctx context.Context) error { _, err := body.Unstitch(ctx); return err }},
		{"Body.Tessellate", func(ctx context.Context) error { _, err := body.Tessellate(ctx, units.Value{}); return err }},
		{"Body.Trim", func(ctx context.Context) error { _, err := body.Trim(ctx, nil, 0); return err }},
		{"Body.Extend", func(ctx context.Context) error { _, err := body.Extend(ctx, nil, nil); return err }},
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
