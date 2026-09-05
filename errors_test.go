package decad_test

import (
	"fmt"
	"maps"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestErrorVocabulary(t *testing.T) {
	t.Parallel()
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
		"ErrInvalidRecipe":       decad.ErrInvalidRecipe,
		"ErrUnitKind":            decad.ErrUnitKind,
		"ErrNotFinite":           decad.ErrNotFinite,
		"ErrUnsupported":         decad.ErrUnsupported,
	}

	// Every sentinel is a branchable identity: the public contract is that a
	// caller branches with errors.Is on a wrapped error, so that is what is
	// asserted — a wrap matches its own sentinel and no other.
	names := slices.Sorted(maps.Keys(sentinels))
	for _, name := range names {
		err := sentinels[name]
		require.Error(t, err, `%s should be non-nil`, name)
		require.True(t, strings.HasPrefix(err.Error(), "decad: "), `%s message should carry the package prefix, got %q`, name, err.Error())

		wrapped := fmt.Errorf(`extrude failed: %w`, err)
		require.ErrorIs(t, wrapped, err, `a wrapped %s should match it through errors.Is`, name)
		for _, other := range names {
			if other == name {
				continue
			}
			require.NotErrorIs(t, wrapped, sentinels[other], `a wrapped %s should not match %s`, name, other)
		}
	}

	// The rendered messages are distinct too — two sentinels must never read
	// as the same failure in a log line.
	seen := make(map[string]string, len(sentinels))
	for _, name := range names {
		msg := sentinels[name].Error()
		prev, dup := seen[msg]
		require.False(t, dup, `%s and %s should not share the message %q`, name, prev, msg)
		seen[msg] = name
	}
}
