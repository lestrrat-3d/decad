package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolCircleOnFiniteSection(t *testing.T) {
	tool := &pointConeLimit{apex: big.NewRat(8, 1), far: big.NewRat(-2, 1),
		slope: big.NewRat(1, 1)}
	section := revolveSectionSpan{start: Point2{U: 5, V: 2}, end: Point2{U: 6, V: 4}}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Equal(t, big.NewRat(16, 3), x)
	require.Equal(t, big.NewRat(8, 3), radius)
	require.Zero(t, lfSub(lfAdd(x, radius), tool.apex).Sign())

	// A vertical Revolve side can also meet the cone at an interior circle.
	section = revolveSectionSpan{start: Point2{U: 6, V: 1}, end: Point2{U: 6, V: 3}}
	x, radius, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.True(t, crosses)
	require.Equal(t, big.NewRat(6, 1), x)
	require.Equal(t, big.NewRat(2, 1), radius)

	// A complete generator on one strict side has no contact circle.
	section = revolveSectionSpan{start: Point2{U: 5, V: 1}, end: Point2{U: 6, V: 1}}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.True(t, decided)
	require.False(t, crosses)

	// A shared endpoint needs a separate topology proof.
	section.end = Point2{U: 7, V: 1}
	_, _, crosses, decided = revolveLoftFacesToolCircleOnSpan(tool, section)
	require.False(t, decided)
	require.False(t, crosses)
}
