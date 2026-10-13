package brepgeom_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func squareRim() [][]brepgeom.RimEdgeEnds {
	p := []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}}
	loop := make([]brepgeom.RimEdgeEnds, 4)
	for i := range loop {
		loop[i] = brepgeom.RimEdgeEnds{Start: p[i], End: p[(i+1)%4]}
	}
	return [][]brepgeom.RimEdgeEnds{loop}
}

func TestAuditMiddleRim(t *testing.T) {
	require.NoError(t, brepgeom.AuditMiddleRim(t.Context(), squareRim(), squareRim()))
	require.NoError(t, brepgeom.AuditMiddleRim(t.Context(), nil, nil))

	loopCount := append(squareRim(), squareRim()[0])
	edgeCount := squareRim()
	edgeCount[0] = edgeCount[0][:3]
	startMoved := squareRim()
	startMoved[0][2].Start.Z = 1e-12
	endMoved := squareRim()
	endMoved[0][1].End.Y = 2

	cases := []struct {
		name string
		to   [][]brepgeom.RimEdgeEnds
		msg  string
	}{
		{"loop count", loopCount, "two-sided taper halves have different middle loop counts"},
		{"edge count", edgeCount, "two-sided taper halves have different middle rim edge counts"},
		{"start moved", startMoved, "two-sided taper halves do not share the held middle rim"},
		{"end moved", endMoved, "two-sided taper halves do not share the held middle rim"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			err := brepgeom.AuditMiddleRim(t.Context(), squareRim(), tc.to)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.msg)
		})
	}

	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		require.Error(t, brepgeom.AuditMiddleRim(ctx, squareRim(), squareRim()))
	})
}
