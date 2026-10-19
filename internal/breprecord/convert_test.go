package breprecord

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// The P8 plate is the 40x20x20 radius-3 box with one y-through radius-3 bore:
// eleven faces, and a record that differs in any stored field is not it.
func TestIsP8Rounded(t *testing.T) {
	t.Parallel()
	want, ok := p8RoundedRecord()
	require.True(t, ok)
	require.Len(t, want.Faces, 11)
	require.True(t, IsP8Rounded(want))

	moved := want
	moved.Faces = append([]Face(nil), want.Faces...)
	moved.Faces[0].Z1 = 21
	require.False(t, IsP8Rounded(moved))

	short := want
	short.Faces = want.Faces[:10]
	require.False(t, IsP8Rounded(short))
}
