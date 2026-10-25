package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPlaneSignsExactLiftsKeepSubnormalSide(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {}}
	exact := []proof.Xpt{
		proof.XptOf(verts[0]), proof.XptOf(verts[1]), proof.XptOf(verts[2]),
		{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(1), W: new(big.Int).Lsh(big.NewInt(1), 1076)},
	}
	tris := [][3]int{{0, 1, 2}, {3, 1, 2}}
	data := NewLoftAuditDataWithExact(verts, exact, tris)
	xother, other := data.xtri(1), data.corners(1)
	want := TrianglePlaneSigns(data.Planes[0], xother)
	require.Equal(t, [3]int{1, 0, 0}, want)
	require.Equal(t, want, data.planeSigns(0, xother, other))
	_, _, broadSigns := loftPlaneSeparatedSigns(data, 0, 1)
	require.Equal(t, want, broadSigns)
}

func TestPlaneSignsSharedAndEqualObliqueCorners(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{0x1p-400, 1, 0x1p400} {
		for _, upward := range []bool{false, true} {
			inf := math.Inf(-1)
			wantSide := -1
			if upward {
				inf, wantSide = math.Inf(1), 1
			}
			for _, reversed := range []bool{false, true} {
				verts := []r3.Vec{
					{X: 1, Y: 2, Z: 3}, {X: 2, Y: 3, Z: 4}, {X: -1, Y: 3, Z: 8},
					{X: 1, Y: 2, Z: math.Nextafter(3, inf)},
					{X: -1, Y: 3, Z: 8}, {X: 1, Y: 2, Z: 3},
				}
				for i := range verts {
					verts[i] = verts[i].Scale(scale)
				}
				tris := [][3]int{{0, 1, 2}, {2, 0, 3}, {4, 5, 3}}
				side := wantSide
				if reversed {
					tris[0][1], tris[0][2] = tris[0][2], tris[0][1]
					side = -side
				}
				data := NewLoftAuditData(verts, tris)
				for _, other := range []int{1, 2} {
					x, corners := data.xtri(other), data.corners(other)
					want := TrianglePlaneSigns(data.Planes[0], x)
					require.Equal(t, [3]int{0, 0, side}, want)
					require.Equal(t, want, data.planeSigns(0, x, corners),
						"scale %g, upward %t, reversed %t, other %d", scale, upward, reversed, other)
				}
			}
		}
	}
}

func TestCompactLoftAuditDataMatchesFullPairResults(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0},
		{X: 0, Y: 0, Z: 1}, {X: 2, Y: 2, Z: 0},
		{X: 1, Y: -1, Z: 0}, {X: 1, Y: 1, Z: 0}, {X: 1, Y: 0, Z: 1},
		{X: 10, Y: 0, Z: 0}, {X: 11, Y: 0, Z: 0}, {X: 10, Y: 1, Z: 0},
	}
	tris := [][3]int{
		{0, 1, 2},  // Base triangle.
		{0, 1, 3},  // Noncoplanar shared edge.
		{1, 4, 2},  // Coplanar shared edge.
		{5, 6, 7},  // Crossing triangle with no shared indices.
		{8, 9, 10}, // Disjoint triangle.
	}
	full := NewLoftAuditData(verts, tris)
	compact := newCompactLoftAuditData(verts, tris)
	require.Empty(t, compact.Corners)
	require.Empty(t, compact.Xtris)
	require.Empty(t, compact.Projections)
	for i := range tris {
		require.Equal(t, full.Corners[i], compact.corners(i))
		require.Equal(t, full.Xtris[i], compact.xtri(i))
		require.Equal(t, full.Projections[i], compact.projectionTriple(i))
		for j := i + 1; j < len(tris); j++ {
			require.Equal(t, loftPlaneSeparatedPrepared(full, i, j),
				loftPlaneSeparatedPrepared(compact, i, j), "pair %d,%d", i, j)
			for _, shortcuts := range []LoftAuditShortcuts{{}, loftGenericShortcuts} {
				want, wantErr := AuditLoftPairData(full, tris, i, j, shortcuts)
				got, gotErr := AuditLoftPairData(compact, tris, i, j, shortcuts)
				require.Equal(t, want, got, "pair %d,%d", i, j)
				if wantErr == nil {
					require.NoError(t, gotErr, "pair %d,%d", i, j)
				} else {
					require.EqualError(t, gotErr, wantErr.Error(), "pair %d,%d", i, j)
				}
			}
		}
	}
}

func TestPreparedPlaneSeparationMatchesUnprepared(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x8c4d2af1, 0x719cf4ad))
	var verts []r3.Vec
	var tris [][3]int
	for i := range 60 {
		x, y := 20*rng.Float64()-10, 20*rng.Float64()-10
		z := 0.0
		switch i % 3 {
		case 1:
			z = math.Ldexp(1, -45)
		case 2:
			z = 20*rng.Float64() - 10
		}
		start := len(verts)
		verts = append(verts,
			r3.Vec{X: x, Y: y, Z: z},
			r3.Vec{X: x + 1, Y: y, Z: z},
			r3.Vec{X: x, Y: y + 1, Z: z},
		)
		tris = append(tris, [3]int{start, start + 1, start + 2})
	}
	data := NewLoftAuditData(verts, tris)
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			want := LoftPlaneSeparated(data.Corners[i], data.Corners[j])
			require.Equal(t, want, loftPlaneSeparatedPrepared(data, i, j), "pair %d,%d", i, j)
		}
	}
}

// TestPlaneSignsMatchTrianglePlaneSigns compares the adaptive per-pair plane
// signs, and the triples the broad phase hands back, against the exact cached
// plane on coplanar, nearly coplanar, separated and crossing triangle pairs.
func TestPlaneSignsMatchTrianglePlaneSigns(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x51d3a9c7, 0x2b6e40f1))
	var verts []r3.Vec
	var tris [][3]int
	for i := range 40 {
		x, y := 4*rng.Float64()-2, 4*rng.Float64()-2
		z := 0.0
		tilt := 0.0
		switch i % 5 {
		case 1:
			z = math.Ldexp(1, -45)
		case 2:
			z = 4*rng.Float64() - 2
		case 3:
			tilt = math.Ldexp(1, -50)
		case 4:
			tilt = 1
		}
		start := len(verts)
		verts = append(verts,
			r3.Vec{X: x, Y: y, Z: z},
			r3.Vec{X: x + 1, Y: y, Z: z + tilt},
			r3.Vec{X: x, Y: y + 1, Z: z - tilt},
		)
		tris = append(tris, [3]int{start, start + 1, start + 2})
	}
	data := NewLoftAuditData(verts, tris)
	for i := range tris {
		for j := range tris {
			if i == j {
				continue
			}
			ta, tb := data.corners(i), data.corners(j)
			xta, xtb := data.xtri(i), data.xtri(j)
			require.Equal(t, TrianglePlaneSigns(data.Planes[i], xtb), data.planeSigns(i, xtb, tb), "pair %d,%d", i, j)
			require.Equal(t, TrianglePlaneSigns(data.Planes[j], xta), data.planeSigns(j, xta, ta), "pair %d,%d", i, j)
			if j < i {
				continue
			}
			separated, sa, sb := loftPlaneSeparatedSigns(data, i, j)
			require.Equal(t, loftPlaneSeparatedPrepared(data, i, j), separated, "pair %d,%d", i, j)
			require.Equal(t, TrianglePlaneSigns(data.Planes[i], xtb), sb, "pair %d,%d", i, j)
			if meshbool.AllOneSide(sb) {
				continue
			}
			require.Equal(t, TrianglePlaneSigns(data.Planes[j], xta), sa, "pair %d,%d", i, j)
		}
	}
}
