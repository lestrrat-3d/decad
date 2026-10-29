package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyLoftAuditDataSelected is the serial table builder the parallel one
// replaced, kept as the reference the tests compare against.
func legacyLoftAuditDataSelected(verts []r3.Vec, exact []proof.Xpt, tris [][3]int,
	selected []bool, compact bool) *LoftAuditData {
	d := &LoftAuditData{
		Xverts:     exact,
		Norms:      make([]proof.Xpt, len(tris)),
		floatNorms: make([]r3.Vec, len(tris)),
		Planes:     make([]LoftExactPlane, len(tris)),
		compact:    compact,
	}
	if compact {
		d.verts, d.tris = verts, tris
		d.projAxes = make([]uint8, len(tris))
	} else {
		d.Corners = make([][3]r3.Vec, len(tris))
		d.Xtris = make([][3]proof.Xpt, len(tris))
		d.Projections = make([][3]meshbool.Xp2, len(tris))
	}
	var projected [3][]meshbool.Xp2
	axes := [3][2]int{{0, 1}, {2, 0}, {1, 2}}
	for i, tri := range tris {
		if selected != nil && !selected[i] {
			continue
		}
		xt := [3]proof.Xpt{d.Xverts[tri[0]], d.Xverts[tri[1]], d.Xverts[tri[2]]}
		if !compact {
			d.Corners[i] = LoftTriCorners(verts, tri)
			d.Xtris[i] = xt
		}
		d.Norms[i] = proof.Xcross(proof.Xsub(xt[1], xt[0]), proof.Xsub(xt[2], xt[0]))
		d.floatNorms[i] = d.Norms[i].Vec()
		d.Planes[i] = NewLoftExactPlane(xt[0], d.Norms[i])
		projection := ProjectionPairIndex(meshbool.ProjAxes(d.Norms[i]))
		if compact {
			d.projAxes[i] = uint8(projection)
		}
		if projected[projection] == nil {
			projected[projection] = make([]meshbool.Xp2, len(exact))
		}
		for j, vi := range tri {
			if projected[projection][vi].U == nil {
				uv := axes[projection]
				projected[projection][vi] = meshbool.NewXP2FromXpt(exact[vi], uv[0], uv[1])
			}
			if !compact {
				d.Projections[i][j] = projected[projection][vi]
			}
		}
	}
	if compact {
		d.projected = projected
	}
	return d
}

// legacyLift lifts the vertices of the selected triangles (all when selected
// is nil) the way the three constructors did before the lift was shared.
func legacyLift(verts []r3.Vec, tris [][3]int, selected []bool) []proof.Xpt {
	exact := make([]proof.Xpt, len(verts))
	if selected == nil {
		for i, v := range verts {
			exact[i] = proof.XptOf(v)
		}
		return exact
	}
	for i, tri := range tris {
		if !selected[i] {
			continue
		}
		for _, vertex := range tri {
			if exact[vertex].W == nil {
				exact[vertex] = proof.XptOf(verts[vertex])
			}
		}
	}
	return exact
}

// loftTableTube is a closed mesh like a two-ring loft with fan caps, rotated
// by 0.3 rad about the x axis.
func loftTableTube(stations int) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	rot := func(x, y, z float64) r3.Vec {
		c, s := math.Cos(0.3), math.Sin(0.3)
		return r3.Vec{X: x, Y: y*c - z*s, Z: y*s + z*c}
	}
	for ring := range 2 {
		for k := range stations {
			a := 2 * math.Pi * float64(k) / float64(stations)
			verts = append(verts, rot(3*math.Cos(a), 2*math.Sin(a), float64(ring)*5))
		}
	}
	bottom, top := len(verts), len(verts)+1
	verts = append(verts, rot(0, 0, 0), rot(0, 0, 5))
	var tris [][3]int
	for k := range stations {
		n := (k + 1) % stations
		tris = append(tris,
			[3]int{k, n, stations + k}, [3]int{n, stations + n, stations + k},
			[3]int{bottom, n, k}, [3]int{top, stations + k, stations + n})
	}
	return verts, tris
}

func loftTableSoup(vertices, triangles int) ([]r3.Vec, [][3]int) {
	rng := rand.New(rand.NewPCG(11, 12))
	verts := make([]r3.Vec, vertices)
	for i := range verts {
		verts[i] = r3.Vec{X: rng.Float64()*10 - 5, Y: rng.Float64()*10 - 5, Z: rng.Float64()*10 - 5}
	}
	tris := make([][3]int, triangles)
	for i := range tris {
		tris[i] = [3]int{rng.IntN(vertices), rng.IntN(vertices), rng.IntN(vertices)}
	}
	return verts, tris
}

// sameInt reports whether every check of requireSameInt passes.
func sameInt(want, got *big.Int) bool {
	if (want == nil) != (got == nil) {
		return false
	}
	return want == nil || want.Cmp(got) == 0
}

// requireSameInt compares with plain Go first and runs the checks below only on a
// mismatch, so a passing call skips t.Helper and require.
func requireSameInt(t *testing.T, want, got *big.Int) {
	if sameInt(want, got) {
		return
	}
	t.Helper()
	require.Equal(t, want == nil, got == nil)
	if want != nil {
		require.Zero(t, want.Cmp(got))
	}
}

// requireSameXpt takes the same fast path as requireSameInt.
func requireSameXpt(t *testing.T, want, got proof.Xpt) {
	if sameInt(want.X, got.X) && sameInt(want.Y, got.Y) && sameInt(want.Z, got.Z) && sameInt(want.W, got.W) {
		return
	}
	t.Helper()
	requireSameInt(t, want.X, got.X)
	requireSameInt(t, want.Y, got.Y)
	requireSameInt(t, want.Z, got.Z)
	requireSameInt(t, want.W, got.W)
}

// sameXp2 reports whether every check of requireSameXp2 passes.
func sameXp2(want, got meshbool.Xp2) bool {
	if (want.U == nil) != (got.U == nil) || (want.V == nil) != (got.V == nil) {
		return false
	}
	if want.U != nil && (want.U.Cmp(got.U) != 0 || want.V.Cmp(got.V) != 0) {
		return false
	}
	return sameInt(want.Hu, got.Hu) && sameInt(want.Hv, got.Hv) && sameInt(want.Hw, got.Hw) &&
		math.Float64bits(want.Fu) == math.Float64bits(got.Fu) &&
		math.Float64bits(want.Fv) == math.Float64bits(got.Fv) &&
		want.FloatFinite == got.FloatFinite
}

// requireSameXp2 takes the same fast path as requireSameInt.
func requireSameXp2(t *testing.T, want, got meshbool.Xp2) {
	if sameXp2(want, got) {
		return
	}
	t.Helper()
	require.Equal(t, want.U == nil, got.U == nil)
	require.Equal(t, want.V == nil, got.V == nil)
	if want.U != nil {
		require.Zero(t, want.U.Cmp(got.U))
		require.Zero(t, want.V.Cmp(got.V))
	}
	requireSameInt(t, want.Hu, got.Hu)
	requireSameInt(t, want.Hv, got.Hv)
	requireSameInt(t, want.Hw, got.Hw)
	require.Equal(t, math.Float64bits(want.Fu), math.Float64bits(got.Fu))
	require.Equal(t, math.Float64bits(want.Fv), math.Float64bits(got.Fv))
	require.Equal(t, want.FloatFinite, got.FloatFinite)
}

// sameVec reports whether every check of requireSameVec passes.
func sameVec(want, got r3.Vec) bool {
	return math.Float64bits(want.X) == math.Float64bits(got.X) &&
		math.Float64bits(want.Y) == math.Float64bits(got.Y) &&
		math.Float64bits(want.Z) == math.Float64bits(got.Z)
}

// requireSameVec takes the same fast path as requireSameInt.
func requireSameVec(t *testing.T, want, got r3.Vec) {
	if sameVec(want, got) {
		return
	}
	t.Helper()
	require.Equal(t, math.Float64bits(want.X), math.Float64bits(got.X))
	require.Equal(t, math.Float64bits(want.Y), math.Float64bits(got.Y))
	require.Equal(t, math.Float64bits(want.Z), math.Float64bits(got.Z))
}

func requireSameLoftTables(t *testing.T, tris [][3]int, selected []bool, want, got *LoftAuditData) {
	t.Helper()
	require.Equal(t, want.compact, got.compact)
	require.Equal(t, want.projAxes, got.projAxes)
	require.Len(t, got.Norms, len(want.Norms))
	for i := range want.Xverts {
		requireSameXpt(t, want.Xverts[i], got.Xverts[i])
	}
	for i := range tris {
		if selected != nil && !selected[i] {
			continue
		}
		requireSameXpt(t, want.Norms[i], got.Norms[i])
		requireSameVec(t, want.floatNorms[i], got.floatNorms[i])
		wp, gp := want.Planes[i], got.Planes[i]
		requireSameInt(t, wp.A, gp.A)
		requireSameInt(t, wp.B, gp.B)
		requireSameInt(t, wp.C, gp.C)
		requireSameInt(t, wp.D, gp.D)
		if want.compact {
			continue
		}
		for j := range 3 {
			requireSameVec(t, want.Corners[i][j], got.Corners[i][j])
			requireSameXpt(t, want.Xtris[i][j], got.Xtris[i][j])
			requireSameXp2(t, want.Projections[i][j], got.Projections[i][j])
		}
	}
	for p := range want.projected {
		require.Equal(t, want.projected[p] == nil, got.projected[p] == nil)
		require.Len(t, got.projected[p], len(want.projected[p]))
		for v := range want.projected[p] {
			requireSameXp2(t, want.projected[p][v], got.projected[p][v])
		}
	}
	if want.compact {
		return
	}
	// A (plane, vertex) pair shares one value across the triangles using it.
	type key struct{ p, v int }
	seen := map[key]*big.Rat{}
	for i, tri := range tris {
		if selected != nil && !selected[i] {
			continue
		}
		p := ProjectionPairIndex(meshbool.ProjAxes(got.Norms[i]))
		for j, vi := range tri {
			if u, ok := seen[key{p, vi}]; ok && u != got.Projections[i][j].U {
				require.Same(t, u, got.Projections[i][j].U)
			}
			seen[key{p, vi}] = got.Projections[i][j].U
		}
	}
}

func TestLoftAuditTablesMatchSerialReference(t *testing.T) {
	tubeVerts, tubeTris := loftTableTube(600)
	soupVerts, soupTris := loftTableSoup(900, 3000)
	meshes := []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}{
		{"tube", tubeVerts, tubeTris},
		{"soup", soupVerts, soupTris},
	}
	for _, m := range meshes {
		thirds := make([]bool, len(m.tris))
		sparse := make([]bool, len(m.tris))
		for i := range m.tris {
			thirds[i] = i%3 == 0
			sparse[i] = i%7 == 2 || i%11 == 5
		}
		selections := []struct {
			name     string
			selected []bool
		}{{"all", nil}, {"thirds", thirds}, {"candidates", sparse}}
		for _, sel := range selections {
			for _, compact := range []bool{false, true} {
				for _, workers := range []int{1, 4} {
					t.Run(m.name+"/"+sel.name, func(t *testing.T) {
						exact := legacyLift(m.verts, m.tris, sel.selected)
						want := legacyLoftAuditDataSelected(m.verts, exact, m.tris, sel.selected, compact)
						var need []bool
						if sel.selected != nil {
							need = make([]bool, len(m.verts))
							for i, tri := range m.tris {
								if sel.selected[i] {
									need[tri[0]], need[tri[1]], need[tri[2]] = true, true, true
								}
							}
						}
						lifted := liftLoftVertices(m.verts, need, workers)
						for i := range exact {
							requireSameXpt(t, exact[i], lifted[i])
						}
						got := newLoftAuditDataSelected(m.verts, lifted, m.tris, sel.selected, compact, workers)
						requireSameLoftTables(t, m.tris, sel.selected, want, got)
					})
				}
			}
		}
	}
}

func TestLoftAuditConstructorsMatchSerialReference(t *testing.T) {
	verts, tris := loftTableTube(520)
	require.GreaterOrEqual(t, len(tris), loftDataParallelMin)
	want := legacyLoftAuditDataSelected(verts, legacyLift(verts, tris, nil), tris, nil, false)
	requireSameLoftTables(t, tris, nil, want, NewLoftAuditData(verts, tris))
	wantCompact := legacyLoftAuditDataSelected(verts, legacyLift(verts, tris, nil), tris, nil, true)
	requireSameLoftTables(t, tris, nil, wantCompact, newCompactLoftAuditData(verts, tris))
}

func TestLoftAuditNormalsUseExactCoordinates(t *testing.T) {
	t.Parallel()
	var exact []proof.Xpt
	for _, p := range []r3.Vec{{X: 0.1}, {Y: 0.3}, {Z: 0.7}, {X: -0.4, Y: 0.6}} {
		exact = append(exact, proof.ExactAffineImage(r3.Vec{X: 0.1, Y: 0.2}, r3.Vec{Y: 0.3, Z: 0.1},
			r3.Vec{X: 0.7, Z: 0.9}, r3.Vec{X: 0.4, Y: 0.2}, p))
	}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	for _, denominator := range []int64{2, 3} {
		points := slices.Clone(exact)
		points[0] = proof.Xlerp(exact[0], exact[1], big.NewInt(1), big.NewInt(denominator))
		verts := make([]r3.Vec, len(points))
		for i, p := range points {
			verts[i] = p.Vec()
		}
		require.False(t, proof.XptDenotes(points[0], verts[0]), "the exact producer differs from its rounded image")
		for _, compact := range []bool{false, true} {
			want := legacyLoftAuditDataSelected(verts, points, tris, nil, compact)
			got := newLoftAuditDataSelected(verts, points, tris, nil, compact, 1)
			requireSameLoftTables(t, tris, nil, want, got)
		}
		want := legacyLoftAuditDataSelected(verts, points, tris, nil, false)
		requireSameLoftTables(t, tris, nil, want, NewLoftAuditDataWithExact(verts, points, tris))
	}
}

func TestLoftDataWorkersSerialOnNonFiniteVertex(t *testing.T) {
	verts, tris := loftTableTube(520)
	require.GreaterOrEqual(t, len(tris), loftDataParallelMin)
	require.Equal(t, 1, loftDataWorkers(verts[:3], 10))
	verts[7].Y = math.NaN()
	require.Equal(t, 1, loftDataWorkers(verts, len(tris)))
	verts[7].Y = math.Inf(1)
	require.Equal(t, 1, loftDataWorkers(verts, len(tris)))
}

// loftTableLatticeSoup is a soup of small triangles over a jittered lattice,
// so a box scan finds a modest candidate list.
func loftTableLatticeSoup(side, triangles int) ([]r3.Vec, [][3]int) {
	rng := rand.New(rand.NewPCG(21, 22))
	var verts []r3.Vec
	for x := range side {
		for y := range side {
			for z := range side {
				verts = append(verts, r3.Vec{
					X: float64(x) + rng.Float64()*0.4, Y: float64(y) + rng.Float64()*0.4,
					Z: float64(z) + rng.Float64()*0.4,
				})
			}
		}
	}
	tris := make([][3]int, triangles)
	for i := range tris {
		a := rng.IntN(len(verts) - 2*side*side)
		tris[i] = [3]int{a, a + 1 + rng.IntN(2)*side, a + side*side + rng.IntN(2)*side}
	}
	return verts, tris
}

func TestLoftAuditDataForCandidatesMatchesSerialReference(t *testing.T) {
	tubeVerts, tubeTris := loftTableTube(600)
	soupVerts, soupTris := loftTableLatticeSoup(13, 3000)
	meshes := []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}{{"tube", tubeVerts, tubeTris}, {"soup", soupVerts, soupTris}}
	for _, m := range meshes {
		for _, workers := range []int{1, 4} {
			t.Run(m.name, func(t *testing.T) {
				boxes := make([][2]r3.Vec, len(m.tris))
				members := make([]int, len(m.tris))
				for i, tri := range m.tris {
					boxes[i] = meshbool.TriBox(m.verts, tri)
					members[i] = i
				}
				order := newPairScan(boxes, members, math.MaxUint64)
				budget := proofbound.NewWorkBudget(t.Context())
				// A thinned list leaves triangles unselected.
				keep := func(i, j int) bool { return i%5 == 0 && j%3 != 1 }
				sc, err := sweepCandidateCounts(budget, order, len(m.tris), keep, math.MaxUint64)
				require.NoError(t, err)
				cands, err := sweepCandidates(budget, order, sc.starts, sc.count, keep)
				require.NoError(t, err)
				require.NotEmpty(t, cands.js)

				selected := make([]bool, len(m.tris))
				for i := range len(cands.starts) - 1 {
					if cands.starts[i] != cands.starts[i+1] {
						selected[i] = true
					}
				}
				for _, j := range cands.js {
					selected[j] = true
				}
				require.True(t, slices.Contains(selected, false))

				want := legacyLoftAuditDataSelected(m.verts, legacyLift(m.verts, m.tris, selected),
					m.tris, selected, true)
				got := newLoftAuditDataForCandidatesWorkers(m.verts, m.tris, cands, workers)
				requireSameLoftTables(t, m.tris, selected, want, got)
				require.Nil(t, got.Corners)
				require.Nil(t, got.Xtris)
				require.Nil(t, got.Projections)
				require.NotSame(t, &m.verts[0], &got.verts[0])
				require.NotSame(t, &m.tris[0], &got.tris[0])
				for i := range selected {
					if selected[i] {
						continue
					}
					require.Nil(t, got.Norms[i].X)
					require.Nil(t, got.Planes[i].A)
					require.Zero(t, got.floatNorms[i])
					require.Zero(t, got.floatPlanes[i])
					require.Zero(t, got.projAxes[i])
				}
			})
		}
	}
}
