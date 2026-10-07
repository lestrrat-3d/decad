package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Two C shapes facing each other with overlapping tips enclose an empty 24 mm²
// cell between them. Their union is 48 + 54 − 4 = 98 mm² of section; filling
// the cell would publish 122. The analytic paths' select-all merge keeps every
// bounded cell, so they decline a pair with such a cell and the mesh path
// answers it (docs/prism-boolean-design.md §4.2).
var (
	voidCShapeA = [][2]float64{{0, 0}, {6, 0}, {6, 3}, {3, 3}, {3, 7}, {6, 7}, {6, 10}, {0, 10}}
	voidCShapeB = [][2]float64{{5, -1}, {11, -1}, {11, 11}, {5, 11}, {5, 8}, {8, 8}, {8, 2}, {5, 2}}
)

func requireMeshPathVerdict(t *testing.T, err error) {
	t.Helper()
	// Every operand shares the z = 0 cap plane, which the mesh path refuses as
	// a coplanar contact; an analytic fill would have returned a body.
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
}

func TestPrismUnionEnclosedVoidTakesMeshPath(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := polyPrismBody(t, doc, voidCShapeA, 1)
	b := polyPrismBody(t, doc, voidCShapeB, 1)
	got, err := decad.Union(t.Context(), a, b)
	if err == nil {
		volume, verr := got.Volume()
		require.NoError(t, verr)
		require.InDelta(t, 98.0, volumeMM(t, volume), boundMM3(t, volume), "the union's volume, void left empty")
	}
	requireMeshPathVerdict(t, err)
}

// The same pair inside A1's stacked union: the C shapes share the slab
// z = 0..10, and a 2×2×5 block on the first C's spine makes the union a stack.
func TestStackedUnionEnclosedVoidTakesMeshPath(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := polyPrismBody(t, doc, voidCShapeA, 10)
	block := boxBodyAtZ(t, doc, 0.5, 4, 2.5, 6, 10, 5)
	stack, err := decad.Union(t.Context(), plate, block)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(stack))
	other := polyPrismBody(t, doc, voidCShapeB, 10)
	got, err := decad.Union(t.Context(), stack, other)
	if err == nil {
		volume, verr := got.Volume()
		require.NoError(t, verr)
		require.InDelta(t, 980.0+20, volumeMM(t, volume), boundMM3(t, volume), "the union's volume, void left empty")
	}
	requireMeshPathVerdict(t, err)
}

// Patterned's rule 3 combines overlapping instances by Union. The C shape
// below has slanted arms so no two copies share a carrier line. Its copy
// 5 mm along x overlaps the first copy's arms, and the copy's spine closes
// the first copy's notch into a 16/3 mm² cell between x = 3 and 5, so the
// chained Union meets the same void and returns the mesh path's verdict.
func TestPatternedEnclosedVoidTakesMeshPath(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	c := polyPrismBody(t, doc, [][2]float64{{0, 0}, {6, 1}, {6, 4}, {3, 3.5}, {3, 6.5}, {6, 6}, {6, 9}, {0, 10}}, 1)
	got, err := c.Patterned(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(5), Count: 2})
	if err == nil {
		volume, verr := got.Volume()
		require.NoError(t, verr)
		// Two 46.5 mm² sections less their 6 mm² overlap; filling the void
		// would publish 87 + 16/3.
		require.InDelta(t, 87.0, volumeMM(t, volume), boundMM3(t, volume), "the union's volume, void left empty")
	}
	requireMeshPathVerdict(t, err)
}
