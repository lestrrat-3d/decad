package facetedtopology_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/stretchr/testify/require"
)

func TestUniqueGroupFace(t *testing.T) {
	src, faceOf := []int{0, 1, 1, 2}, []int{0, 3, 3, 1}
	tests := []struct {
		name           string
		src, faceOf    []int
		faceCount, grp int
		wantFace       int
		wantOK         bool
	}{
		{"shared face", src, faceOf, 4, 1, 3, true},
		{"single facet", src, faceOf, 4, 2, 1, true},
		{"absent group", src, faceOf, 4, 5, -1, false},
		{"negative group", src, faceOf, 4, -1, -1, false},
		{"two faces", []int{1, 1}, []int{0, 2}, 4, 1, -1, false},
		{"index out of range", []int{1}, []int{4}, 4, 1, -1, false},
		{"length mismatch", []int{1, 1}, []int{0}, 4, 1, -1, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			face, ok := facetedtopology.UniqueGroupFace(tc.src, tc.faceOf, tc.faceCount, tc.grp)
			require.Equal(t, tc.wantOK, ok)
			require.Equal(t, tc.wantFace, face)
		})
	}
}

func TestGroupFace(t *testing.T) {
	tests := []struct {
		name        string
		src, faceOf []int
		faceCount   int
		wantFace    int
		wantOK      bool
	}{
		{"unique", []int{0, 1, 1}, []int{0, 3, 3}, 4, 3, true},
		{"two faces", []int{1, 1}, []int{0, 2}, 4, -1, false},
		{"index out of range", []int{1}, []int{4}, 4, -1, false},
		{"negative index", []int{1}, []int{-1}, 4, -1, false},
		{"no match", []int{0, 0}, []int{0, 0}, 4, -1, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			face, ok := facetedtopology.GroupFace(tc.src, tc.faceOf, tc.faceCount, 1)
			require.Equal(t, tc.wantOK, ok)
			require.Equal(t, tc.wantFace, face)
		})
	}
}

func TestTrimGroupFace(t *testing.T) {
	src, faceOf := []int{0, 1, 1}, []int{2, 0, 0}
	require.Equal(t, 0, facetedtopology.TrimGroupFace(src, faceOf, 3, 1))
	require.Equal(t, 2, facetedtopology.TrimGroupFace(src, faceOf, 3, 0))
	require.Equal(t, -1, facetedtopology.TrimGroupFace(src, faceOf, 3, 7))
	require.Equal(t, -1, facetedtopology.TrimGroupFace([]int{1, 1}, []int{0, 2}, 3, 1))
	require.Equal(t, -1, facetedtopology.TrimGroupFace(src, faceOf, 2, 0))
	require.Equal(t, -1, facetedtopology.TrimGroupFace([]int{1}, []int{0, 0}, 3, 1))
}

func TestCapGroupsPresent(t *testing.T) {
	src, faceOf := []int{0, 1, 2}, []int{0, 1, 2}
	require.True(t, facetedtopology.CapGroupsPresent(src, faceOf, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPresent(src, faceOf, 3, 0, 0))
	require.False(t, facetedtopology.CapGroupsPresent(src, faceOf, 3, 0, 3))
	require.False(t, facetedtopology.CapGroupsPresent(src, faceOf, 3, -1, 2))
	require.False(t, facetedtopology.CapGroupsPresent([]int{0, 0, 2}, faceOf, 3, 0, 1))
	require.False(t, facetedtopology.CapGroupsPresent([]int{1, 1, 2}, faceOf, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPresent(src, faceOf[:2], 3, 0, 2))
}

func TestCapGroupsPure(t *testing.T) {
	src := []int{0, 1, 2}
	require.True(t, facetedtopology.CapGroupsPure(src, []int{0, 1, 2}, 3, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPure(src, []int{0, 1, 0}, 3, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPure(src, []int{0, 0, 2}, 3, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPure(src, []int{0, 1, 2}, 4, 3, 0, 2))
	require.False(t, facetedtopology.CapGroupsPure(src, []int{0, 1, 3}, 3, 3, 0, 2))
}
