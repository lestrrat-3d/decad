package decaderr_test

import (
	"errors"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// TestSentinelsMatchTheirRootReExports pins the re-home: every root sentinel
// is the very value decaderr declares, so errors.Is matches across the two
// names, wrapped or not, and the message is exactly the public one.
func TestSentinelsMatchTheirRootReExports(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		root  error
		inner error
		msg   string
	}{
		{"ErrNoMatch", decad.ErrNoMatch, decaderr.ErrNoMatch, "decad: selector matched nothing"},
		{"ErrCardinality", decad.ErrCardinality, decaderr.ErrCardinality, "decad: cardinality assertion failed"},
		{"ErrBodyReportNotFound", decad.ErrBodyReportNotFound, decaderr.ErrBodyReportNotFound, "decad: body is not represented in this report"},
		{"ErrForeignBody", decad.ErrForeignBody, decaderr.ErrForeignBody, "decad: body is owned by a different document"},
		{"ErrForeignProfile", decad.ErrForeignProfile, decaderr.ErrForeignProfile, "decad: profile was built from a different sketch"},
		{"ErrStaleProfile", decad.ErrStaleProfile, decaderr.ErrStaleProfile, "decad: profile is stale"},
		{"ErrRetiredBody", decad.ErrRetiredBody, decaderr.ErrRetiredBody, "decad: body has been retired from its document"},
		{"ErrNegativeMagnitude", decad.ErrNegativeMagnitude, decaderr.ErrNegativeMagnitude, "decad: negative magnitude"},
		{"ErrUnrecordableProfile", decad.ErrUnrecordableProfile, decaderr.ErrUnrecordableProfile, "decad: profile boundary cannot be recorded exactly"},
		{"ErrNotSolid", decad.ErrNotSolid, decaderr.ErrNotSolid, "decad: body is not a solid"},
		{"ErrDegenerate", decad.ErrDegenerate, decaderr.ErrDegenerate, "decad: degenerate input"},
		{"ErrBooleanFailed", decad.ErrBooleanFailed, decaderr.ErrBooleanFailed, "decad: boolean operation failed"},
		{"ErrInvalidProfile", decad.ErrInvalidProfile, decaderr.ErrInvalidProfile, "decad: profile is not a valid region"},
		{"ErrUnitKind", decad.ErrUnitKind, decaderr.ErrUnitKind, "decad: wrong unit kind"},
		{"ErrNotFinite", decad.ErrNotFinite, decaderr.ErrNotFinite, "decad: non-finite value"},
		{"ErrUnsupported", decad.ErrUnsupported, decaderr.ErrUnsupported, "decad: not supported by the current evaluator"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			require.Same(t, c.inner, c.root, "the root sentinel must be the decaderr value itself")
			require.Equal(t, c.msg, c.root.Error())
			require.Equal(t, c.msg, c.inner.Error())
			require.ErrorIs(t, fmt.Errorf("wrapped: %w", c.inner), c.root)
			require.ErrorIs(t, fmt.Errorf("wrapped: %w", c.root), c.inner)
		})
	}
	// The sentinels stay distinct from one another.
	for i, a := range cases {
		for _, b := range cases[i+1:] {
			require.Falsef(t, errors.Is(a.root, b.root), "%s must not match %s", a.name, b.name)
		}
	}
}
