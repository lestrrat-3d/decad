package proofbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestProvesAtMost(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		value proofbound.BoundedScalar
		limit proofbound.BoundedScalar
		want  bool
	}{
		{name: "equal exact values", value: proofbound.ExactScalar(100), limit: proofbound.ExactScalar(100), want: true},
		{name: "touching endpoints", value: proofbound.ExactScalar(100), limit: proofbound.MeasuredScalar(101, 1), want: true},
		{name: "overlapping intervals", value: proofbound.MeasuredScalar(100, 1), limit: proofbound.MeasuredScalar(101, 1)},
		{name: "value exceeds limit", value: proofbound.ExactScalar(101), limit: proofbound.ExactScalar(100)},
		{name: "subnormal bound does not round away", value: proofbound.MeasuredScalar(1, math.SmallestNonzeroFloat64), limit: proofbound.ExactScalar(1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, proofbound.ProvesAtMost(tc.value, tc.limit))
		})
	}
}

func TestProvesAtLeast(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		value proofbound.BoundedScalar
		limit proofbound.BoundedScalar
		want  bool
	}{
		{name: "equal exact values", value: proofbound.ExactScalar(100), limit: proofbound.ExactScalar(100), want: true},
		{name: "touching endpoints", value: proofbound.MeasuredScalar(102, 1), limit: proofbound.MeasuredScalar(100, 1), want: true},
		{name: "overlapping intervals", value: proofbound.MeasuredScalar(101, 1), limit: proofbound.MeasuredScalar(100, 1)},
		{name: "value is below limit", value: proofbound.ExactScalar(99), limit: proofbound.ExactScalar(100)},
		{name: "subnormal bound does not round away", value: proofbound.MeasuredScalar(1, math.SmallestNonzeroFloat64), limit: proofbound.ExactScalar(1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, proofbound.ProvesAtLeast(tc.value, tc.limit))
		})
	}
}
