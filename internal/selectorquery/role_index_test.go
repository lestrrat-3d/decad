package selectorquery_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/stretchr/testify/require"
)

func TestRoleIndex(t *testing.T) {
	got := selectorquery.RoleIndex([][]string{{"capStart"}, {"side(0,0)", "shared"}, nil, {"shared", "capEnd"}})
	require.Equal(t, map[string]int{"capStart": 0, "side(0,0)": 1, "shared": 3, "capEnd": 3}, got)
	require.Empty(t, selectorquery.RoleIndex(nil))
}
