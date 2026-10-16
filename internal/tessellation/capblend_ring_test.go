package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

func TestCompactCapBlendRing(t *testing.T) {
	a := sectionrecord.Point2{U: 1, V: 2}
	b := sectionrecord.Point2{U: 3, V: 4}
	c := sectionrecord.Point2{U: 5, V: 6}
	cases := []struct {
		name         string
		points       []sectionrecord.Point2
		vertices     []int
		wantPoints   []sectionrecord.Point2
		wantVertices []int
	}{
		{"empty", nil, nil, []sectionrecord.Point2{}, []int{}},
		{"ordinary", []sectionrecord.Point2{a, b, c}, []int{10, 20, 30},
			[]sectionrecord.Point2{a, b, c}, []int{10, 20, 30}},
		{"consecutive", []sectionrecord.Point2{a, b, b, c}, []int{10, 20, 21, 30},
			[]sectionrecord.Point2{a, b, c}, []int{10, 20, 30}},
		{"closing", []sectionrecord.Point2{a, b, c, a}, []int{10, 20, 30, 40},
			[]sectionrecord.Point2{a, b, c}, []int{10, 20, 30}},
		{"all equal", []sectionrecord.Point2{a, a, a}, []int{10, 20, 30},
			[]sectionrecord.Point2{a}, []int{10}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			points, vertices := tessellation.CompactCapBlendRing(tc.points, tc.vertices)
			require.Equal(t, tc.wantPoints, points)
			require.Equal(t, tc.wantVertices, vertices)
		})
	}
}

func TestCompactCapBlendRingUsesPointEquality(t *testing.T) {
	minusZero := math.Copysign(0, -1)
	points, vertices := tessellation.CompactCapBlendRing(
		[]sectionrecord.Point2{{U: minusZero}, {U: 0}, {U: 1}}, []int{3, 4, 5})
	require.Len(t, points, 2)
	require.True(t, math.Signbit(points[0].U), "the first signed-zero point survives")
	require.Equal(t, []int{3, 5}, vertices)

	nan := math.NaN()
	points, vertices = tessellation.CompactCapBlendRing(
		[]sectionrecord.Point2{{U: nan}, {U: nan}, {U: 1}}, []int{3, 4, 5})
	require.Len(t, points, 3, "NaN points do not compare equal")
	require.True(t, math.IsNaN(points[0].U))
	require.True(t, math.IsNaN(points[1].U))
	require.Equal(t, 1.0, points[2].U)
	require.Equal(t, []int{3, 4, 5}, vertices)
}

func TestCompactCapBlendRingRetainsIndexingAndOutputOwnership(t *testing.T) {
	a := sectionrecord.Point2{U: 1}
	b := sectionrecord.Point2{U: 2}
	points, vertices := tessellation.CompactCapBlendRing([]sectionrecord.Point2{a, a}, []int{7})
	require.Equal(t, []sectionrecord.Point2{a}, points,
		"a skipped duplicate does not read its missing vertex index")
	require.Equal(t, []int{7}, vertices)
	require.Panics(t, func() {
		tessellation.CompactCapBlendRing([]sectionrecord.Point2{a, b}, []int{7})
	}, "a kept point still reads its missing vertex index")

	inputPoints := []sectionrecord.Point2{a, b}
	inputVertices := []int{7, 8}
	points, vertices = tessellation.CompactCapBlendRing(inputPoints, inputVertices)
	points[0] = sectionrecord.Point2{U: 9}
	vertices[0] = 99
	require.Equal(t, a, inputPoints[0], "the output point slice owns its backing array")
	require.Equal(t, 7, inputVertices[0], "the output index slice owns its backing array")
}
