package export_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExportOperationsRejectNilContextBeforeOperands(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name string
		call func(context.Context) error
	}{
		{"STL", func(ctx context.Context) error { return export.STL(ctx, nil, nil, units.Value{}) }},
		{"OBJ", func(ctx context.Context) error { return export.OBJ(ctx, nil, nil, units.Value{}) }},
		{"STEP", func(ctx context.Context) error { return export.STEP(ctx, nil, nil, units.Value{}) }},
		{"NewSTEPFile", func(ctx context.Context) error {
			_, err := export.NewSTEPFile(ctx, nil, units.Value{}, step.Header{})
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
}
