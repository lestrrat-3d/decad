package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestProvePairSeparatedOnlyProvesStrictGaps(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 0, 0, 10, 10, 10)
	id := r3.Identity()
	request := contactRequest()
	query := func(ctx context.Context, pose r3.Transform, req decad.ContactRequest) bool {
		t.Helper()
		separated, err := doc.ProvePairSeparated(ctx, a, b, id, pose, req)
		require.NoError(t, err)
		return separated
	}

	require.True(t, query(t.Context(), contactPose(t, r3.Vec{Z: 13}), request))
	require.False(t, query(t.Context(), contactPose(t, r3.Vec{Z: 10}), request))
	require.False(t, query(t.Context(), contactPose(t, r3.Vec{Z: 9}), request))

	request.SupportBand = units.Millimeters(0.1)
	require.False(t, query(t.Context(), contactPose(t, r3.Vec{Z: 13}), request))
}

func TestProvePairSeparatedKeepsContactPairValidationOrder(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 1, 1, 1)
	b := boxBody(t, doc, 0, 0, 1, 1, 1)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	req := contactRequest()
	req.PointResolution = units.Radians(1)
	_, err := doc.ProvePairSeparated(ctx, a, b, r3.Identity(), r3.Identity(), req)
	require.ErrorIs(t, err, decad.ErrUnitKind)

	req = contactRequest()
	_, err = doc.ProvePairSeparated(ctx, a, b, r3.Identity(), r3.Identity(), req)
	require.ErrorIs(t, err, context.Canceled)
}
