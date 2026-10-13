package revolveshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/stretchr/testify/require"
)

func TestSideSegments(t *testing.T) {
	segs, ok := revolveshell.SideSegments([]string{"capStart", "side(0,3)", "side(1,4)", "side(0,7)", "side(x,2)"})
	require.True(t, ok)
	require.Equal(t, []int{3, 7}, segs)

	segs, ok = revolveshell.SideSegments([]string{"capEnd", "side(2,1)", "side(3,0,2)"})
	require.False(t, ok)
	require.Empty(t, segs)

	_, ok = revolveshell.SideSegments(nil)
	require.False(t, ok)
}
