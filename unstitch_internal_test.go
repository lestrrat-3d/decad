package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// TestUnstitchNoPayloadIsDegenerate is Table R row R12's other half: a live
// body this evaluator did not build (no featurePayload at all). decad's
// public seam admits no way to construct one — every real body a caller can
// reach already carries the payload of whatever built it — so this fixture
// is hand-built directly, the same treatment stitch_internal_test.go gives
// its own seam-unreachable fixtures.
func TestUnstitchNoPayloadIsDegenerate(t *testing.T) {
	t.Parallel()
	d := &Document{}
	b := &Body{doc: d, kind: BodySheet}
	d.bodies = append(d.bodies, b)

	_, err := b.UnstitchContext(t.Context())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Len(t, d.Bodies(), 1)
	require.Same(t, b, d.Bodies()[0])
}
