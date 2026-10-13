package modifyoption_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/stretchr/testify/require"
)

func TestSideSegments(t *testing.T) {
	roles := []string{
		"side(1,0)", "side(1,2)", "side(2,5)", "capStart", "side(1,3,4)",
		"side(01,7)", "side(1,2)", "side(1,x)", "",
	}
	require.Equal(t, map[int]struct{}{0: {}, 2: {}}, modifyoption.SideSegments(roles, 1))
	require.Equal(t, map[int]struct{}{5: {}}, modifyoption.SideSegments(roles, 2))
	require.Empty(t, modifyoption.SideSegments(roles, 9))
	require.Empty(t, modifyoption.SideSegments(nil, 1))
}
