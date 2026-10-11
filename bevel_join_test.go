package decad

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBevelTrimmedTeethRequireSeparatedCertifiedRanges(t *testing.T) {
	motion, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	first := facetedPayload{
		verts:     []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 2)},
		meshBound: 0.1,
	}
	second := facetedPayload{
		verts:      []r3.Vec{r3.NewVec(0, 0, 3), r3.NewVec(1, 0, 4)},
		meshBound:  0.1,
		bevelTooth: &bevelToothPlacement{motion: motion},
	}
	require.True(t, bevelTrimmedSourcesSeparateZ(first, second))
	second.meshBound = 1
	require.False(t, bevelTrimmedSourcesSeparateZ(first, second))
	second.meshBound = 0.1
	second.verts[0].Z = 2
	require.False(t, bevelTrimmedSourcesSeparateZ(first, second))
}

func TestBevelAxisRotationRequiresPureNonidentityXTurn(t *testing.T) {
	xTurn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	require.True(t, bevelAxisRotation(xTurn))
	require.False(t, bevelAxisRotation(r3.Identity()))
	yTurn, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(45))
	require.NoError(t, err)
	require.False(t, bevelAxisRotation(yTurn))
}

func TestBevelSharedRingLeavesOnlyToothOwnedRootSectorsOpen(t *testing.T) {
	blank := &bevelBlankSource{
		toeAxis: Point2{U: 0}, heelAxis: Point2{U: 10},
		heelOuter: Point2{U: 10, V: 4}, ded: Point2{U: 8, V: 3},
		toeRoot: Point2{U: 2, V: 3}, toeOuter: Point2{U: 0, V: 4},
	}
	const lo, hi = -0.1, 0.1
	const oppositeLo, oppositeHi = math.Pi - 0.1, math.Pi + 0.1
	angles := make([]float64, 0, 260)
	for i := range 256 {
		angles = append(angles, -math.Pi/8+math.Pi*float64(i)/128)
	}
	angles = append(angles, lo, hi, oppositeLo, oppositeHi)
	sort.Float64s(angles)
	whole := &bevelJoinedMesh{}
	wholeRing, wholeBound, err := bevelBuildBlankMesh(whole, blank, angles, nil)
	require.NoError(t, err)
	sewn := &bevelJoinedMesh{}
	sewnRing, sewnBound, err := bevelBuildBlankMesh(sewn, blank, angles, [][2]float64{{lo, hi}})
	require.NoError(t, err)
	paired := &bevelJoinedMesh{}
	pairedRing, pairedBound, err := bevelBuildBlankMesh(paired, blank, angles,
		[][2]float64{{lo, hi}, {oppositeLo, oppositeHi}})
	require.NoError(t, err)

	require.Equal(t, whole.verts, sewn.verts)
	require.Equal(t, whole.verts, paired.verts)
	require.Equal(t, wholeRing.angles, sewnRing.angles)
	require.Equal(t, wholeRing.angles, pairedRing.angles)
	require.Equal(t, wholeBound, sewnBound)
	require.Equal(t, wholeBound, pairedBound)
	require.Greater(t, sewnBound, 0.0)
	require.Less(t, sewnBound, 0.01)
	require.Len(t, sewn.verts, 2+4*len(angles))
	loIndex := sort.SearchFloat64s(angles, lo)
	hiIndex := sort.SearchFloat64s(angles, hi)
	oppositeLoIndex := sort.SearchFloat64s(angles, oppositeLo)
	oppositeHiIndex := sort.SearchFloat64s(angles, oppositeHi)
	require.Equal(t, r3.NewVec(2, 3*math.Cos(lo), 3*math.Sin(lo)),
		sewn.verts[sewnRing.rings[2][loIndex]])
	require.Equal(t, r3.NewVec(2, 3*math.Cos(hi), 3*math.Sin(hi)),
		sewn.verts[sewnRing.rings[2][hiIndex]])

	count := func(mesh *bevelJoinedMesh, group int) int {
		n := 0
		for _, source := range mesh.src {
			if source == group {
				n++
			}
		}
		return n
	}
	require.Equal(t, 2*len(angles), count(whole, 3))
	require.Equal(t, 2*(len(angles)-(hiIndex-loIndex)), count(sewn, 3))
	require.Equal(t, 2*(len(angles)-(hiIndex-loIndex)-(oppositeHiIndex-oppositeLoIndex)),
		count(paired, 3))
	for _, group := range []int{1, 2, 4, 5} {
		require.Equal(t, count(whole, group), count(sewn, group))
		require.Equal(t, count(whole, group), count(paired, group))
	}
}
