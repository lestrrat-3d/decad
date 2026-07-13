package decad_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestErrorVocabulary(t *testing.T) {
	sentinels := map[string]error{
		"ErrNoMatch":             decad.ErrNoMatch,
		"ErrCardinality":         decad.ErrCardinality,
		"ErrForeignBody":         decad.ErrForeignBody,
		"ErrForeignProfile":      decad.ErrForeignProfile,
		"ErrStaleProfile":        decad.ErrStaleProfile,
		"ErrRetiredBody":         decad.ErrRetiredBody,
		"ErrUnresolvedBody":      decad.ErrUnresolvedBody,
		"ErrNegativeMagnitude":   decad.ErrNegativeMagnitude,
		"ErrUnrecordableProfile": decad.ErrUnrecordableProfile,
		"ErrNotSolid":            decad.ErrNotSolid,
		"ErrDegenerate":          decad.ErrDegenerate,
		"ErrBooleanFailed":       decad.ErrBooleanFailed,
		"ErrInvalidProfile":      decad.ErrInvalidProfile,
		"ErrUnitKind":            decad.ErrUnitKind,
		"ErrNotFinite":           decad.ErrNotFinite,
	}

	// Every sentinel is a branchable identity: non-nil, prefixed with the
	// package name, and distinct from every other sentinel — a wrapped error
	// must match exactly one branch.
	seen := make(map[string]string, len(sentinels))
	for name, err := range sentinels {
		require.Error(t, err, `%s should be non-nil`, name)
		require.True(t, strings.HasPrefix(err.Error(), "decad: "), `%s message should carry the package prefix, got %q`, name, err.Error())

		prev, dup := seen[err.Error()]
		require.False(t, dup, `%s and %s should not share the message %q`, name, prev, err.Error())
		seen[err.Error()] = name
	}
}
