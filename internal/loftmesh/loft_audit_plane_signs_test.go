package loftmesh

import (
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceLoftPlaneSeparatedSigns is loftPlaneSeparatedSigns over
// referencePlaneSigns alone, the differential reference for liftedPlaneSigns.
func referenceLoftPlaneSeparatedSigns(data *LoftAuditData, i, j int) (bool, [3]int, [3]int) {
	ta, tb := data.corners(i), data.corners(j)
	xta, xtb := data.xtri(i), data.xtri(j)
	var sa, sb [3]int
	sb = referencePlaneSigns(data, i, xtb, tb)
	if meshbool.AllOneSide(sb) {
		return true, sa, sb
	}
	sa = referencePlaneSigns(data, j, xta, ta)
	return meshbool.AllOneSide(sa), sa, sb
}

// referencePlaneSigns is a copy of planeSigns: the identity test on the exact
// lifts first, then the float plane, the float expansion and the exact
// numerator.
func referencePlaneSigns(d *LoftAuditData, i int, xother [3]proof.Xpt, other [3]r3.Vec) [3]int {
	if d.exactLifts {
		return TrianglePlaneSigns(d.Planes[i], xother)
	}
	var local proof.FloatOrientationPlane
	plane := &local
	if d.floatPlanes != nil {
		plane = &d.floatPlanes[i]
	} else {
		t := d.corners(i)
		local = proof.NewFloatOrientationPlane(t[0], t[1], t[2])
	}
	xt := d.xtri(i)
	var signs [3]int
	var sum, term big.Int
	var corners [3]r3.Vec
	loadedCorners := false
	for k := range signs {
		p := xother[k]
		if p == xt[0] || p == xt[1] || p == xt[2] {
			continue
		}
		if s, certain := plane.Sign(other[k]); certain {
			signs[k] = s
			continue
		}
		if !loadedCorners {
			corners = d.corners(i)
			loadedCorners = true
		}
		if s, certain := proof.OrientSignExpansion(corners[0], corners[1], corners[2], other[k]); certain {
			signs[k] = s
			continue
		}
		signs[k] = d.Planes[i].Sign(p, &sum, &term)
	}
	return signs
}

type planeSignMesh struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// torusMesh is a faceted torus with nu by nv quads, each split in two, scaled
// by scale and moved by offset. With seam set, the last column of vertices
// repeats the first column's coordinates under its own indices, so triangles
// across the seam share points but no vertex index.
func torusMesh(name string, nu, nv int, major, minor, scale float64, offset r3.Vec, seam bool) planeSignMesh {
	m := planeSignMesh{name: name}
	cols := nu
	if seam {
		cols = nu + 1
	}
	for u := range cols {
		a := 2 * math.Pi * float64(u%nu) / float64(nu)
		for v := range nv {
			b := 2 * math.Pi * float64(v) / float64(nv)
			r := major + minor*math.Cos(b)
			p := r3.Vec{X: r * math.Cos(a), Y: r * math.Sin(a), Z: minor * math.Sin(b)}
			m.verts = append(m.verts, p.Scale(scale).Add(offset))
		}
	}
	at := func(u, v int) int { return (u%cols)*nv + v%nv }
	for u := range nu {
		for v := range nv {
			a, b, c, d := at(u, v), at(u+1, v), at(u+1, v+1), at(u, v+1)
			m.tris = append(m.tris, [3]int{a, b, c}, [3]int{a, c, d})
		}
	}
	return m
}

// twistedBandMesh joins levels rings of n points on a cylinder, each ring
// turned by twist from the one below, with two triangles per quad. Its long
// slanted triangles have large boxes, so each row has many partners.
func twistedBandMesh(n, levels int, radius, height, twist float64) planeSignMesh {
	m := planeSignMesh{name: "twisted band"}
	for l := range levels {
		for k := range n {
			a := 2*math.Pi*float64(k)/float64(n) + twist*float64(l)
			m.verts = append(m.verts, r3.Vec{X: radius * math.Cos(a), Y: radius * math.Sin(a), Z: height * float64(l)})
		}
	}
	for l := range levels - 1 {
		for k := range n {
			a, b := l*n+k, l*n+(k+1)%n
			c, d := (l+1)*n+(k+1)%n, (l+1)*n+k
			m.tris = append(m.tris, [3]int{a, b, c}, [3]int{a, c, d})
		}
	}
	return m
}

// flatGridMesh is an n by n grid of unit squares on z = 0, split in two. Every
// third vertex is lifted by one ulp of its height scale or left flat, so the
// pairs' plane signs are mostly exact zeros or one-ulp decisions.
func flatGridMesh(n int, rng *rand.Rand) planeSignMesh {
	m := planeSignMesh{name: "near-flat grid"}
	for x := range n + 1 {
		for y := range n + 1 {
			z := 0.0
			if (x+y)%3 == 0 {
				z = math.Nextafter(0, float64(rng.IntN(2)*2-1))
				if rng.IntN(2) == 0 {
					z = float64(rng.IntN(2)*2-1) * 0x1p-52
				}
			}
			m.verts = append(m.verts, r3.Vec{X: float64(x), Y: float64(y), Z: z})
		}
	}
	at := func(x, y int) int { return x*(n+1) + y }
	for x := range n {
		for y := range n {
			a, b, c, d := at(x, y), at(x+1, y), at(x+1, y+1), at(x, y+1)
			m.tris = append(m.tris, [3]int{a, b, c}, [3]int{a, c, d})
		}
	}
	return m
}

// randomTriangleMesh is count independent random triangles in a small box, so
// many pairs cross and neither plane separates them.
func randomTriangleMesh(count int, rng *rand.Rand) planeSignMesh {
	m := planeSignMesh{name: "random crossing triangles"}
	for k := range count {
		for range 3 {
			m.verts = append(m.verts, r3.Vec{X: 4*rng.Float64() - 2, Y: 4*rng.Float64() - 2, Z: 4*rng.Float64() - 2})
		}
		m.tris = append(m.tris, [3]int{3 * k, 3*k + 1, 3*k + 2})
	}
	return m
}

// planeSignMeshes covers every step of planeSigns: float-certain signs on a
// torus at unit scale; the expansion on near-flat grids and seam duplicates;
// the exact numerator on scales outside the float plane's and the expansion's
// ranges and on a far offset, whose differences round.
func planeSignMeshes() []planeSignMesh {
	rng := rand.New(rand.NewPCG(0x6a1c, 0x3e97))
	return []planeSignMesh{
		torusMesh("torus", 48, 24, 10, 3, 1, r3.Vec{}, false),
		torusMesh("torus seam", 32, 12, 10, 3, 1, r3.Vec{X: 0.25, Y: -0.5}, true),
		torusMesh("torus 2^-290", 16, 8, 10, 3, 0x1p-290, r3.Vec{}, true),
		torusMesh("torus 2^240", 16, 8, 7, 3, 0x1p240, r3.Vec{}, false),
		torusMesh("torus 2^400", 12, 6, 10, 3, 0x1p400, r3.Vec{}, false),
		torusMesh("torus 2^-1000", 12, 6, 10, 2, 0x1p-1000, r3.Vec{}, false),
		torusMesh("torus far offset", 16, 8, 10, 3, 1, r3.Vec{X: 1e15, Y: -1e15, Z: 3e14}, true),
		twistedBandMesh(60, 4, 10, 4, 0.6),
		flatGridMesh(12, rng),
		randomTriangleMesh(120, rng),
	}
}

// The table layouts planeSignLayouts builds.
const (
	tableFull       = "full table"
	tableCompact    = "compact table"
	tableCandidates = "candidate table"
	tableExact      = "exact-lift table"
	tableNoFloat    = "table without float planes"
)

// planeSignLayouts builds every table loftPlaneSeparatedSigns reads: the full,
// compact and candidate tables, the exact-lift table, and a full table without
// its float planes, as the private audit builds it.
func planeSignLayouts(m planeSignMesh) map[string]*LoftAuditData {
	all := loftCandidates{starts: make([]int, len(m.tris)+1), js: make([]int32, len(m.tris))}
	for i := range m.tris {
		all.starts[i+1] = i + 1
		all.js[i] = int32(i)
	}
	exact := make([]proof.Xpt, len(m.verts))
	for i, v := range m.verts {
		exact[i] = proof.XptOf(v)
	}
	noFloat := NewLoftAuditData(m.verts, m.tris)
	noFloat.floatPlanes = nil
	return map[string]*LoftAuditData{
		tableFull:       NewLoftAuditData(m.verts, m.tris),
		tableCompact:    newCompactLoftAuditData(m.verts, m.tris),
		tableCandidates: newLoftAuditDataForCandidates(m.verts, m.tris, all),
		tableExact:      NewLoftAuditDataWithExact(m.verts, exact, m.tris),
		tableNoFloat:    noFloat,
	}
}

// requireSameSeparation checks one pair's verdict and sign triples.
func requireSameSeparation(t *testing.T, wantSep, gotSep bool, wantA, gotA, wantB, gotB [3]int,
	msgAndArgs ...any) {
	t.Helper()
	require.Equal(t, wantSep, gotSep, msgAndArgs...)
	require.Equal(t, wantB, gotB, msgAndArgs...)
	require.Equal(t, wantA, gotA, msgAndArgs...)
}

// TestLoftPlaneSeparatedSignsMatchesReference compares loftPlaneSeparatedSigns
// and loftPlaneSeparatedSignsMemo with the reference on every ordered pair
// whose boxes overlap, of every mesh and table, including pairs that share
// vertices (whose shared corners the reference zeroes by identity). The memo
// sees the pairs in row order, as a parallelLoftPairs worker does, and then
// shuffled with a fresh memo. It also counts which planeSigns step decides
// each lifted sign and how many memo signs are reused, so every step is shown
// to run.
func TestLoftPlaneSeparatedSignsMatchesReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x3b, 0x5d))
	var floatSigns, expansionSigns, numeratorSigns, identitySigns, separated, crossing, reused int
	for _, m := range planeSignMeshes() {
		boxes := make([][2]r3.Vec, len(m.tris))
		for i, tri := range m.tris {
			boxes[i] = meshbool.TriBox(m.verts, tri)
		}
		var pairs [][2]int
		for i := range m.tris {
			for j := range m.tris {
				if i != j && meshbool.BoxesOverlap(boxes[i], boxes[j]) {
					pairs = append(pairs, [2]int{i, j})
				}
			}
		}
		shuffled := slices.Clone(pairs)
		rng.Shuffle(len(shuffled), func(a, b int) { shuffled[a], shuffled[b] = shuffled[b], shuffled[a] })
		for layout, data := range planeSignLayouts(m) {
			memo := newLoftPlaneSignMemo(data)
			require.Equal(t, layout == tableExact || layout == tableNoFloat, memo == nil, "%s %s", m.name, layout)
			for _, pair := range shuffled {
				i, j := pair[0], pair[1]
				wantSep, wantA, wantB := referenceLoftPlaneSeparatedSigns(data, i, j)
				gotSep, gotA, gotB := loftPlaneSeparatedSignsMemo(data, memo, i, j)
				requireSameSeparation(t, wantSep, gotSep, wantA, gotA, wantB, gotB,
					"%s %s shuffled memo pair %d,%d", m.name, layout, i, j)
			}
			memo = newLoftPlaneSignMemo(data)
			for _, pair := range pairs {
				i, j := pair[0], pair[1]
				wantSep, wantA, wantB := referenceLoftPlaneSeparatedSigns(data, i, j)
				gotSep, gotA, gotB := loftPlaneSeparatedSigns(data, i, j)
				requireSameSeparation(t, wantSep, gotSep, wantA, gotA, wantB, gotB,
					"%s %s pair %d,%d", m.name, layout, i, j)
				if memo != nil {
					for _, v := range data.tris[j] {
						if memo.entry[v]&^3 == uint32(i+1)<<2 {
							reused++
						}
					}
				}
				gotSep, gotA, gotB = loftPlaneSeparatedSignsMemo(data, memo, i, j)
				requireSameSeparation(t, wantSep, gotSep, wantA, gotA, wantB, gotB,
					"%s %s memo pair %d,%d", m.name, layout, i, j)
				if wantSep {
					separated++
				} else {
					crossing++
				}
				if layout != tableFull {
					continue
				}
				_, shared := tessellation.SharedVertexIndices(m.tris[i], m.tris[j])
				corners := data.corners(i)
				for k, p := range data.corners(j) {
					switch _, certain := data.floatPlanes[i].Sign(p); {
					case shared > 0 && data.xtri(j)[k] == data.xtri(i)[0] ||
						shared > 0 && data.xtri(j)[k] == data.xtri(i)[1] ||
						shared > 0 && data.xtri(j)[k] == data.xtri(i)[2]:
						identitySigns++
					case certain:
						floatSigns++
					default:
						if _, ok := proof.OrientSignExpansion(corners[0], corners[1], corners[2], p); ok {
							expansionSigns++
						} else {
							numeratorSigns++
						}
					}
				}
			}
		}
	}
	require.Positive(t, floatSigns)
	require.Positive(t, expansionSigns)
	require.Positive(t, numeratorSigns)
	require.Positive(t, identitySigns)
	require.Positive(t, separated)
	require.Positive(t, crossing)
	require.Positive(t, reused)
	t.Logf("float %d, expansion %d, numerator %d, identity %d; separated %d, not %d; memo reused %d",
		floatSigns, expansionSigns, numeratorSigns, identitySigns, separated, crossing, reused)
}

// TestLiftedPlaneSignsMatchesExactPlane checks liftedPlaneSigns against the
// exact numerator alone on every ordered pair of the meshes' full tables.
func TestLiftedPlaneSignsMatchesExactPlane(t *testing.T) {
	t.Parallel()
	for _, m := range planeSignMeshes() {
		data := NewLoftAuditData(m.verts, m.tris)
		step := 1
		if len(m.tris) > 600 {
			step = 7
		}
		for i := 0; i < len(m.tris); i += step {
			for j := range m.tris {
				if i == j {
					continue
				}
				want := TrianglePlaneSigns(data.Planes[i], data.xtri(j))
				require.Equal(t, want, data.liftedPlaneSigns(i, j), "%s pair %d,%d", m.name, i, j)
			}
		}
	}
}
