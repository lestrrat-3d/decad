package loftmesh

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// capsMesh is a mesh the way a producer assembles it, with the layout it
// would record beside it.
type capsMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	layout LoftCapLayout
}

func (m capsMesh) clone() capsMesh {
	out := capsMesh{verts: slices.Clone(m.verts), tris: slices.Clone(m.tris), layout: m.layout}
	out.layout.Cap0 = slices.Clone(m.layout.Cap0)
	out.layout.Cap1 = slices.Clone(m.layout.Cap1)
	return out
}

// capsGrid rounds to multiples of 1/1024 so every midpoint and offset the
// tests use is an exact float.
func capsGrid(x float64) float64 { return math.Round(x*1024) / 1024 }

// capsRing returns n clockwise ring points at height z.
func capsRing(n int, radius, z float64) []r3.Vec {
	ring := make([]r3.Vec, n)
	for k := range ring {
		angle := -2 * math.Pi * float64(k) / float64(n)
		ring[k] = r3.NewVec(capsGrid(radius*math.Cos(angle)), capsGrid(radius*math.Sin(angle)), z)
	}
	return ring
}

// capsTestCone is an apex at (0, 0, 2) over an n-gon at z = 0: n apex walls
// (0, k+1, k+2) and a far cap fanned from ring vertex 1.
func capsTestCone(n int) capsMesh {
	verts := append([]r3.Vec{r3.NewVec(0, 0, 2)}, capsRing(n, 1, 0)...)
	var tris [][3]int
	for k := range n {
		tris = append(tris, [3]int{0, k + 1, (k+1)%n + 1})
	}
	layout := LoftCapLayout{Walls: n, Loops0: [][]int{make([]int, n)}, Loops1: [][]int{{0}}}
	for k := range n {
		layout.Loops0[0][k] = k + 1
	}
	for r := 1; r < n-1; r++ {
		layout.Cap0 = append(layout.Cap0, len(tris))
		tris = append(tris, [3]int{1, r + 2, r + 1})
	}
	return capsMesh{verts: verts, tris: tris, layout: layout}
}

// capsTestFrustum is rings at z = 0 (radius 1) and z = 1 (radius 0.6) with
// the walls buildPointPlaneSlab emits and the caps interleaved upper/lower.
func capsTestFrustum(n int) capsMesh {
	verts := append(capsRing(n, 1, 0), capsRing(n, 0.6, 1)...)
	var tris [][3]int
	for b := range n {
		c := (b + 1) % n
		tris = append(tris, [3]int{b, n + b, n + c}, [3]int{b, n + c, c})
	}
	layout := LoftCapLayout{Walls: len(tris), Loops0: [][]int{make([]int, n)}, Loops1: [][]int{make([]int, n)}}
	for k := range n {
		layout.Loops0[0][k], layout.Loops1[0][k] = n+k, k
	}
	for r := 1; r < n-1; r++ {
		layout.Cap0 = append(layout.Cap0, len(tris))
		tris = append(tris, [3]int{n, n + r + 1, n + r})
		layout.Cap1 = append(layout.Cap1, len(tris))
		tris = append(tris, [3]int{0, r, r + 1})
	}
	return capsMesh{verts: verts, tris: tris, layout: layout}
}

// capsTestBanded is a loft of `rings` rings at z = 0, 1, 2, ... with radii 1, 0.9, ...,
// the walls band by band as assembleLoftFaces emits them and the start and
// end caps interleaved. The rings between them are Interior.
func capsTestBanded(n, rings int) capsMesh {
	var verts []r3.Vec
	for i := range rings {
		verts = append(verts, capsRing(n, 1-0.1*float64(i), float64(i))...)
	}
	bands := rings - 1
	var tris [][3]int
	for band := range bands {
		lo, hi := band*n, (band+1)*n
		for b := range n {
			c := (b + 1) % n
			tris = append(tris, [3]int{lo + b, hi + b, hi + c}, [3]int{lo + b, hi + c, lo + c})
		}
	}
	layout := LoftCapLayout{Walls: len(tris), Loops0: [][]int{make([]int, n)}, Loops1: [][]int{make([]int, n)}}
	for k := range n {
		layout.Loops0[0][k], layout.Loops1[0][k] = k, bands*n+k
	}
	for r := 1; r < n-1; r++ {
		layout.Cap0 = append(layout.Cap0, len(tris))
		tris = append(tris, [3]int{0, r + 1, r})
		layout.Cap1 = append(layout.Cap1, len(tris))
		tris = append(tris, [3]int{bands * n, bands*n + r, bands*n + r + 1})
	}
	for v := n; v < bands*n; v++ {
		layout.Interior = append(layout.Interior, v)
	}
	return capsMesh{verts: verts, tris: tris, layout: layout}
}

func capsGeneric(t *testing.T, m capsMesh) error {
	t.Helper()
	return LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), m.verts, m.tris)
}

// requireSameVerdict requires the caps entry to return what the generic
// audit returns: the same nil, or an error with the same text and the same
// refused pair.
func requireSameVerdict(t *testing.T, m capsMesh) (LoftAuditWork, bool, error) {
	t.Helper()
	want := capsGeneric(t, m)
	work, structured, got := LoftCrossingAuditWithCapsWork(t.Context(), m.verts, m.tris, m.layout)
	if want == nil {
		require.NoError(t, got)
		return work, structured, got
	}
	require.Error(t, got)
	require.Equal(t, want.Error(), got.Error())
	var wantContact, gotContact *LoftContactError
	if errors.As(want, &wantContact) {
		require.True(t, errors.As(got, &gotContact))
		require.Equal(t, wantContact.I, gotContact.I)
		require.Equal(t, wantContact.J, gotContact.J)
	}
	return work, structured, got
}

func capsBases() map[string]capsMesh {
	return map[string]capsMesh{
		"cone":    capsTestCone(64),
		"frustum": capsTestFrustum(65),
		"banded":  capsTestBanded(40, 5),
	}
}

func TestLoftCrossingAuditWithCapsDecidesCaps(t *testing.T) {
	t.Parallel()
	proofs := map[string]int{"cone": 1, "frustum": 2, "banded": 2}
	for name, m := range capsBases() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			work, structured, err := LoftCrossingAuditWithCapsWork(t.Context(), m.verts, m.tris, m.layout)
			require.NoError(t, err)
			require.True(t, structured)
			require.Equal(t, proofs[name], work.CapProofs)
			require.NoError(t, capsGeneric(t, m))
			generic, err := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()), m.verts, m.tris,
				loftGenericShortcuts)
			require.NoError(t, err)
			require.Less(t, work.Candidates, generic.Candidates)
			require.Less(t, work.Classifications, generic.Classifications)
		})
	}
}

// capsMutation changes one thing about a mesh.
type capsMutation struct {
	name   string
	mutate func(m *capsMesh)
}

func capsMutations() []capsMutation {
	return []capsMutation{
		{"collapse a cap triangle", func(m *capsMesh) {
			// Move one vertex of a middle cap triangle onto the midpoint of
			// its opposite edge.
			tri := m.tris[m.layout.Cap0[len(m.layout.Cap0)/2]]
			a, c := m.verts[tri[0]], m.verts[tri[2]]
			m.verts[tri[1]] = r3.NewVec((a.X+c.X)/2, (a.Y+c.Y)/2, (a.Z+c.Z)/2)
		}},
		{"push a cap vertex up", func(m *capsMesh) {
			tri := m.tris[m.layout.Cap0[len(m.layout.Cap0)/2]]
			v := m.verts[tri[1]]
			m.verts[tri[1]] = r3.NewVec(v.X, v.Y, v.Z+0.25)
		}},
		{"swap wall indices", func(m *capsMesh) {
			i, j := 0, m.layout.Walls/2
			m.tris[i][1], m.tris[j][1] = m.tris[j][1], m.tris[i][1]
		}},
		{"move a ring across a cap", func(m *capsMesh) {
			// For the banded mesh: ring 1 below the start cap; for the
			// others: the first wall vertex pulled across the cap plane.
			if len(m.layout.Interior) == 0 {
				v := m.verts[m.tris[0][2]]
				m.verts[m.tris[0][2]] = r3.NewVec(v.X/2, v.Y/2, -0.5)
				return
			}
			n := len(m.layout.Loops0[0])
			for k := range n {
				v := m.verts[n+k]
				m.verts[n+k] = r3.NewVec(v.X/2, v.Y/2, -0.5)
			}
		}},
		{"put the apex on the cap plane", func(m *capsMesh) {
			if len(m.layout.Loops1[0]) != 1 {
				v := m.verts[m.layout.Loops1[0][0]]
				m.verts[m.layout.Loops1[0][0]] = r3.NewVec(v.X, v.Y, v.Z-1)
				return
			}
			m.verts[0] = r3.NewVec(0.25, 0.25, 0)
		}},
	}
}

func TestLoftCrossingAuditWithCapsMatchesGenericRefusals(t *testing.T) {
	t.Parallel()
	for name, base := range capsBases() {
		for _, mutation := range capsMutations() {
			t.Run(name+"/"+mutation.name, func(t *testing.T) {
				t.Parallel()
				m := base.clone()
				mutation.mutate(&m)
				want := capsGeneric(t, m)
				_, _, got := requireSameVerdict(t, m)
				require.Equal(t, want == nil, got == nil)
			})
		}
	}
}

// TestLoftCrossingAuditWithCapsRefusalsAreReached pins that the mutations
// above are not all passing meshes: each of the cases named here is refused
// by the generic audit, so the caps entry has to refuse it too.
func TestLoftCrossingAuditWithCapsRefusalsAreReached(t *testing.T) {
	t.Parallel()
	refused := map[string][]string{
		"cone":    {"collapse a cap triangle", "swap wall indices", "put the apex on the cap plane"},
		"frustum": {"collapse a cap triangle", "swap wall indices"},
		"banded":  {"collapse a cap triangle", "swap wall indices", "move a ring across a cap"},
	}
	bases := capsBases()
	for name, names := range refused {
		for _, mutation := range capsMutations() {
			if !slices.Contains(names, mutation.name) {
				continue
			}
			t.Run(name+"/"+mutation.name, func(t *testing.T) {
				t.Parallel()
				m := bases[name].clone()
				mutation.mutate(&m)
				require.Error(t, capsGeneric(t, m))
				_, structured, err := LoftCrossingAuditWithCapsWork(t.Context(), m.verts, m.tris, m.layout)
				require.Error(t, err)
				require.True(t, structured, "the refusal comes through the structured pass and its fallback")
			})
		}
	}
}

// TestLoftCrossingAuditWithCapsRandomMeshes compares the caps entry with the
// generic audit on seeded random mutations of the three meshes: a vertex
// moved by a dyadic offset, two wall vertices swapped, a cap triangle
// reassigned to a random ring vertex.
func TestLoftCrossingAuditWithCapsRandomMeshes(t *testing.T) {
	t.Parallel()
	for name, base := range capsBases() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			rng := rand.New(rand.NewPCG(7, uint64(len(name))))
			offsets := []float64{-0.5, -0.25, -0.125, 0.125, 0.25, 0.5}
			refusals, passes := 0, 0
			for range 40 {
				m := base.clone()
				switch rng.IntN(3) {
				case 0:
					v := rng.IntN(len(m.verts))
					axis := rng.IntN(3)
					d := offsets[rng.IntN(len(offsets))]
					p := m.verts[v]
					c := [3]float64{p.X, p.Y, p.Z}
					c[axis] += d
					m.verts[v] = r3.NewVec(c[0], c[1], c[2])
				case 1:
					i, j := rng.IntN(m.layout.Walls), rng.IntN(m.layout.Walls)
					k, l := rng.IntN(3), rng.IntN(3)
					m.tris[i][k], m.tris[j][l] = m.tris[j][l], m.tris[i][k]
				default:
					capTris := slices.Concat(m.layout.Cap0, m.layout.Cap1)
					tri := capTris[rng.IntN(len(capTris))]
					m.tris[tri][rng.IntN(3)] = len(m.verts) - 1 - rng.IntN(len(m.verts)/2)
				}
				work, structured, err := requireSameVerdict(t, m)
				if err != nil {
					refusals++
					continue
				}
				passes++
				if structured {
					generic, genericErr := LoftCrossingAuditWork(proofbound.NewWorkBudget(t.Context()),
						m.verts, m.tris, loftGenericShortcuts)
					require.NoError(t, genericErr)
					require.LessOrEqual(t, work.Candidates, generic.Candidates)
				}
			}
			require.Positive(t, refusals, "the random mutations must reach a refusal")
			t.Logf("%s: %d refusals, %d passes", name, refusals, passes)
		})
	}
}

func TestCapFamilyProofInteriorSide(t *testing.T) {
	t.Parallel()
	prove := func(m capsMesh, which int, interior []int) LoftCapCondition {
		order, ok := capsReorder(len(m.verts), m.tris, m.layout)
		require.True(t, ok)
		structure := LoftAuditStructure{Walls: m.layout.Walls, CapStartCount: len(m.layout.Cap0),
			Loops0: m.layout.Loops0, Loops1: m.layout.Loops1, Interior: interior}
		return CapFamilyProof(NewLoftAuditData(m.verts, order), order, structure, which)
	}
	banded := capsTestBanded(40, 5)
	for which := range 2 {
		require.Equal(t, LoftCapProofHolds, prove(banded, which, banded.layout.Interior))
		require.Equal(t, LoftCapStructure, prove(banded, which, nil),
			"without Interior a band between interior rings has no vertex of the other cap")
	}

	crossing := banded.clone()
	n := len(crossing.layout.Loops0[0])
	for k := range n {
		v := crossing.verts[n+k]
		crossing.verts[n+k] = r3.NewVec(v.X/2, v.Y/2, -0.5)
	}
	require.Equal(t, LoftCapOtherSide, prove(crossing, 0, crossing.layout.Interior))

	repeated := slices.Clone(banded.layout.Interior)
	repeated[0] = banded.layout.Loops0[0][3]
	require.Equal(t, LoftCapStructure, prove(banded, 0, repeated))
	outOfRange := slices.Clone(banded.layout.Interior)
	outOfRange[0] = len(banded.verts)
	require.Equal(t, LoftCapStructure, prove(banded, 0, outOfRange))

	cone := capsTestCone(64)
	require.Equal(t, LoftCapProofHolds, prove(cone, 0, nil))
	require.Equal(t, LoftCapStructure, prove(cone, 1, nil), "an apex fan has no second cap to prove")
	onPlane := cone.clone()
	onPlane.verts[0] = r3.NewVec(0.25, 0.25, 0)
	require.Equal(t, LoftCapOtherSide, prove(onPlane, 0, nil), "an apex on the cap plane is not strictly on one side")
}

func TestLoftCrossingAuditWithCapsFallsBack(t *testing.T) {
	t.Parallel()
	generic := func(t *testing.T, m capsMesh) {
		t.Helper()
		_, structured, _ := requireSameVerdict(t, m)
		require.False(t, structured)
	}
	t.Run("duplicate cap index", func(t *testing.T) {
		t.Parallel()
		m := capsTestFrustum(20)
		m.layout.Cap1[0] = m.layout.Cap0[0]
		generic(t, m)
	})
	t.Run("count mismatch", func(t *testing.T) {
		t.Parallel()
		m := capsTestFrustum(20)
		m.layout.Cap1 = m.layout.Cap1[1:]
		generic(t, m)
	})
	t.Run("cap index below the walls", func(t *testing.T) {
		t.Parallel()
		m := capsTestFrustum(20)
		m.layout.Cap0[0] = 0
		generic(t, m)
	})
	t.Run("cap index past the mesh", func(t *testing.T) {
		t.Parallel()
		m := capsTestFrustum(20)
		m.layout.Cap0[0] = len(m.tris)
		generic(t, m)
	})
	t.Run("negative walls", func(t *testing.T) {
		t.Parallel()
		m := capsTestFrustum(20)
		m.layout.Walls = -1
		generic(t, m)
	})
	t.Run("tilted caps", func(t *testing.T) {
		t.Parallel()
		m := capsTestCone(64)
		// A shear by a slope that is not a power of two rounds the far-cap
		// vertices off any one plane.
		for i, v := range m.verts {
			m.verts[i] = r3.NewVec(v.X, v.Y, v.Z+0.1*v.X+0.3*v.Y)
		}
		generic(t, m)
	})
	t.Run("past the triangle guard", func(t *testing.T) {
		t.Parallel()
		// 4002 disjoint triangles in the plane z = 0: the generic audit has
		// no candidate, and the last one is declared a cap that is exactly
		// planar, so only the guard keeps this call on the generic path.
		var m capsMesh
		for i := range 4002 {
			x := float64(i)
			base := len(m.verts)
			m.verts = append(m.verts, r3.NewVec(x, 0, 0), r3.NewVec(x+0.5, 0, 0), r3.NewVec(x, 0.5, 0))
			m.tris = append(m.tris, [3]int{base, base + 1, base + 2})
		}
		m.layout = LoftCapLayout{Walls: len(m.tris) - 1, Cap0: []int{len(m.tris) - 1}}
		generic(t, m)

		small := capsMesh{verts: m.verts[:30], tris: m.tris[:10]}
		small.layout = LoftCapLayout{Walls: 9, Cap0: []int{9}}
		_, structured, err := LoftCrossingAuditWithCapsWork(t.Context(), small.verts, small.tris, small.layout)
		require.NoError(t, err)
		require.True(t, structured, "the same shape under the guard takes the structured pass")
	})
}

func TestCapsAuditFitsStaysBelowTheStreamedPreflight(t *testing.T) {
	t.Parallel()
	require.True(t, capsAuditFits(0))
	require.True(t, capsAuditFits(1))
	require.True(t, capsAuditFits(1414))
	require.False(t, capsAuditFits(1415))
	require.True(t, uint64(1414)*1413/2 <= loftAuditCandidateBatchSize)
	require.True(t, uint64(1415)*1414/2 > loftAuditCandidateBatchSize)
}

func TestFanRing(t *testing.T) {
	t.Parallel()
	forward := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1}}
	ring, ok := FanRing(forward)
	require.True(t, ok)
	require.Equal(t, []int{0, 1, 2, 3}, ring)

	reversed := [][3]int{{0, 2, 1}, {0, 3, 2}, {0, 4, 3}, {0, 1, 4}}
	ring, ok = FanRing(reversed)
	require.True(t, ok)
	require.Equal(t, []int{0, 3, 2, 1}, ring)

	for name, sides := range map[string][][3]int{
		"too few sides":  {{0, 1, 2}, {0, 2, 1}},
		"repeated b":     {{0, 1, 2}, {0, 1, 3}, {0, 3, 4}, {0, 4, 1}},
		"two cycles":     {{0, 1, 2}, {0, 2, 1}, {0, 3, 4}, {0, 4, 3}},
		"first not zero": {{0, 1, 2}, {0, 2, 3}, {1, 3, 4}, {0, 4, 1}},
		"out of range":   {{0, 1, 2}, {0, 2, 3}, {0, 3, 5}, {0, 4, 1}},
		"b equals c":     {{0, 1, 1}, {0, 2, 3}, {0, 3, 4}, {0, 4, 2}},
	} {
		_, ok := FanRing(sides)
		require.False(t, ok, name)
	}
}

// capsDenseCone is the review's valid cone: a centre vertex 0, an apex 1 and
// 2000 square-ring points, 2000 apex walls then 2000 cap triangles. Every
// triangle's box contains the centre, so the generic audit has C(4000, 2)
// candidates, streams them and refuses at its row preflight, which charges
// 4000 root visits beside the pairs.
func capsDenseCone() capsMesh {
	const side = 500
	verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)}
	for j := range side {
		verts = append(verts, r3.NewVec(side, -side+2*float64(j), 0))
	}
	for j := range side {
		verts = append(verts, r3.NewVec(side-2*float64(j), side, 0))
	}
	for j := range side {
		verts = append(verts, r3.NewVec(-side, side-2*float64(j), 0))
	}
	for j := range side {
		verts = append(verts, r3.NewVec(-side+2*float64(j), -side, 0))
	}
	ring := len(verts) - 2
	m := capsMesh{verts: verts}
	loop := make([]int, ring)
	for k := range ring {
		loop[k] = 2 + k
		m.tris = append(m.tris, [3]int{1, 2 + k, 2 + (k+1)%ring})
	}
	m.layout = LoftCapLayout{Walls: ring, Loops0: [][]int{loop}, Loops1: [][]int{{1}}}
	for k := range ring {
		m.layout.Cap0 = append(m.layout.Cap0, len(m.tris))
		m.tris = append(m.tris, [3]int{0, 2 + (k+1)%ring, 2 + k})
	}
	return m
}

func TestLoftCrossingAuditWithCapsKeepsTheGenericRowCeiling(t *testing.T) {
	t.Parallel()
	m := capsDenseCone()
	require.Len(t, m.tris, 4000)
	want := capsGeneric(t, m)
	require.Error(t, want, "the generic audit refuses this mesh at its row preflight")
	_, structured, got := requireSameVerdict(t, m)
	require.False(t, structured)
	require.Error(t, got)
}

// capsLeadingZero is the review's five-vertex mesh: the apex (vertex 0) lies
// on the cap's plane and Interior holds one vertex above it. Its three walls
// tile the cap triangle, so skipping the wall-cap pairs would hide an
// overlap.
func capsLeadingZero() capsMesh {
	return capsMesh{
		verts: []r3.Vec{r3.NewVec(0.5, 0.5, 0), r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0),
			r3.NewVec(0, 2, 0), r3.NewVec(0, 0, 1)},
		tris: [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 2, 3}},
		layout: LoftCapLayout{Walls: 3, Cap0: []int{3}, Loops0: [][]int{{1, 2, 3}},
			Loops1: [][]int{{0}}, Interior: []int{4}},
	}
}

func TestCapFamilyProofRejectsLeadingZeroSign(t *testing.T) {
	t.Parallel()
	m := capsLeadingZero()
	order, ok := capsReorder(len(m.verts), m.tris, m.layout)
	require.True(t, ok)
	structure := LoftAuditStructure{Walls: 3, CapStartCount: 1, Loops0: m.layout.Loops0,
		Loops1: m.layout.Loops1, Interior: m.layout.Interior}
	require.Equal(t, LoftCapOtherSide,
		CapFamilyProof(NewLoftAuditData(m.verts, order), order, structure, 0))
	require.Error(t, capsGeneric(t, m), "the walls overlap the cap")
	_, _, err := requireSameVerdict(t, m)
	require.Error(t, err)
}

// TestLoftCrossingAuditWithCapsNearPlaneApex moves the cone's apex onto the
// cap plane, then one float step to either side of it: the exact sign has no
// tolerance, so a representable separation is a strict side, and the caps
// entry must still return the generic verdict at each position.
func TestLoftCrossingAuditWithCapsNearPlaneApex(t *testing.T) {
	t.Parallel()
	for _, z := range []float64{0, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.Nextafter(0.5, 0), 0.5} {
		m := capsTestCone(32)
		m.verts[0] = r3.NewVec(0.25, 0.25, z)
		requireSameVerdict(t, m)
	}
	m := capsTestCone(32)
	m.verts[0] = r3.NewVec(0.25, 0.25, 0)
	_, _, err := requireSameVerdict(t, m)
	require.Error(t, err)
}

// TestLoftCrossingAuditWithCapsMislabeledPartition relabels triangles under
// a layout that is a complete, unique partition: a wall named as a cap, and
// a cap named as a wall. The proof owns the meaning of the labels, so the
// verdict must still be the generic audit's, including on corrupted meshes.
func TestLoftCrossingAuditWithCapsMislabeledPartition(t *testing.T) {
	t.Parallel()
	mutations := append([]capsMutation{{"intact", func(*capsMesh) {}}}, capsMutations()...)
	for name, base := range capsBases() {
		for _, mutation := range mutations {
			t.Run(name+"/"+mutation.name, func(t *testing.T) {
				t.Parallel()
				wallAsCap := base.clone()
				mutation.mutate(&wallAsCap)
				wallAsCap.layout.Walls--
				wallAsCap.layout.Cap0 = append([]int{wallAsCap.layout.Walls}, wallAsCap.layout.Cap0...)
				requireSameVerdict(t, wallAsCap)

				capAsWall := base.clone()
				mutation.mutate(&capAsWall)
				first := capAsWall.layout.Cap0[0]
				capAsWall.tris[first], capAsWall.tris[capAsWall.layout.Walls] =
					capAsWall.tris[capAsWall.layout.Walls], capAsWall.tris[first]
				capAsWall.layout.Walls++
				capAsWall.layout.Cap0 = capAsWall.layout.Cap0[1:]
				requireSameVerdict(t, capAsWall)
			})
		}
	}
}

func TestLoftCrossingAuditWithCapsInvalidIndexNeverPanics(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	m := capsTestFrustum(20)
	m.tris[m.layout.Cap0[0]][1] = len(m.verts) + 5
	require.NotPanics(t, func() {
		_, structured, err := LoftCrossingAuditWithCapsWork(ctx, m.verts, m.tris, m.layout)
		require.False(t, structured)
		require.ErrorIs(t, err, context.Canceled)
	})
	m = capsTestFrustum(20)
	m.tris[m.layout.Cap1[0]][2] = -1
	require.NotPanics(t, func() {
		_, _, err := LoftCrossingAuditWithCapsWork(ctx, m.verts, m.tris, m.layout)
		require.ErrorIs(t, err, context.Canceled)
	})
}

// cancelAfter is a budget whose context reports cancellation from the
// n-th step on and stays cancelled, as a deadline does.
func cancelAfter(n int) *proofbound.WorkBudget {
	steps, cancelled := 0, false
	return &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps >= n {
				cancelled = true
			}
			if cancelled {
				return context.DeadlineExceeded
			}
			return nil
		},
		ErrFn: func() error {
			if cancelled {
				return context.DeadlineExceeded
			}
			return nil
		},
	}
}

// TestLoftCrossingAuditWithCapsCancelledStructuredPass cancels the budget at
// every step of the structured pass of a mesh the generic audit refuses on
// geometry. A cancelled context stays cancelled, so the generic rerun on the
// same budget must report the cancellation, never the geometric refusal from
// a fresh, uncancelled count.
func TestLoftCrossingAuditWithCapsCancelledStructuredPass(t *testing.T) {
	t.Parallel()
	m := capsTestFrustum(20)
	capsMutations()[0].mutate(&m)
	require.Error(t, capsGeneric(t, m))
	sawCancel := false
	for n := 1; n < 400; n++ {
		_, _, err := loftCrossingAuditWithCaps(cancelAfter(n), m.verts, m.tris, m.layout)
		require.Error(t, err)
		if errors.Is(err, context.DeadlineExceeded) {
			sawCancel = true
			continue
		}
		var contact *LoftContactError
		if !errors.As(err, &contact) {
			// An S6 refusal is the generic audit's text.
			require.Equal(t, capsGeneric(t, m).Error(), err.Error())
		}
	}
	require.True(t, sawCancel)
}

// TestLoftCrossingAuditWithCapsCollapsedWallMatchesGeneric is the review's
// mesh with one collapsed wall: without cancellation the caps entry returns
// the generic audit's refusal, text and triangle index included.
func TestLoftCrossingAuditWithCapsCollapsedWallMatchesGeneric(t *testing.T) {
	t.Parallel()
	m := capsMesh{
		verts: []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		tris:  [][3]int{{0, 1, 1}, {0, 2, 3}, {0, 3, 1}, {1, 2, 3}},
		layout: LoftCapLayout{Walls: 3, Cap0: []int{3}, Loops0: [][]int{{1, 2, 3}},
			Loops1: [][]int{{0}}},
	}
	want := capsGeneric(t, m)
	require.ErrorContains(t, want, "loft triangle 0 has collapsed to zero area")
	_, structured, got := requireSameVerdict(t, m)
	require.True(t, structured)
	require.Equal(t, want.Error(), got.Error())
}

// TestLoftCrossingAuditWithCapsCollapsedCapNamesOriginalIndex collapses a
// triangle of the interleaved second cap: the structured pass numbers it in
// its reordered copy, so only the generic rerun names the caller's index.
func TestLoftCrossingAuditWithCapsCollapsedCapNamesOriginalIndex(t *testing.T) {
	t.Parallel()
	m := capsTestFrustum(20)
	index := m.layout.Cap1[3]
	m.tris[index][1] = m.tris[index][0]
	want := capsGeneric(t, m)
	require.ErrorContains(t, want, fmt.Sprintf("loft triangle %d has collapsed to zero area", index))
	_, structured, got := requireSameVerdict(t, m)
	require.True(t, structured)
	require.Equal(t, want.Error(), got.Error())
}
