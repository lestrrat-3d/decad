package loftmesh

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceMassAdd is the three-lift, three-coordinate implementation before
// AddMesh. Keep it independent of prepareVertex and addPrepared.
func referenceMassAdd(m *MassAccumulator, a, b, c r3.Vec, wall bool) {
	sa := proofarith.Xsub(proofarith.XptOf(a), m.anchor)
	sb := proofarith.Xsub(proofarith.XptOf(b), m.anchor)
	sc := proofarith.Xsub(proofarith.XptOf(c), m.anchor)
	triVol6 := proofarith.XdotRat(sa, proofarith.Xcross(sb, sc))
	proofarith.AddRat(m.Vol6, m.Vol6, triVol6)
	saX, saY, saZ := proofarith.XhpRat(proofarith.Xhp(sa))
	sbX, sbY, sbZ := proofarith.XhpRat(proofarith.Xhp(sb))
	scX, scY, scZ := proofarith.XhpRat(proofarith.Xhp(sc))
	sumX := proofbound.RatAdd(saX, sbX, scX)
	sumY := proofbound.RatAdd(saY, sbY, scY)
	sumZ := proofbound.RatAdd(saZ, sbZ, scZ)
	proofarith.AddRat(m.MomX, m.MomX, proofarith.MulRat(new(big.Rat), triVol6, sumX))
	proofarith.AddRat(m.MomY, m.MomY, proofarith.MulRat(new(big.Rat), triVol6, sumY))
	proofarith.AddRat(m.MomZ, m.MomZ, proofarith.MulRat(new(big.Rat), triVol6, sumZ))
	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)
	if m.delta > 0 {
		m.PerturbAreaSum = proofbound.UpRound(m.PerturbAreaSum + proofbound.PerturbedTriangleAreaAllow(a, b, c, m.delta))
	}
	if !wall {
		return
	}
	lo, hi := WallTriangleArea(proofarith.Xsub(sb, sa), proofarith.Xsub(sc, sa))
	m.WallAreaSum += lo
	m.WallAreaAbs = proofbound.UpRound(m.WallAreaAbs + lo)
	m.WallAreaSlack = proofbound.UpRound(m.WallAreaSlack + proofbound.UpRound(hi-lo))
	m.WallTerms++
}

func massFloatBits(t *testing.T, want, got float64) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got))
}

func massVecBits(t *testing.T, want, got r3.Vec) {
	t.Helper()
	massFloatBits(t, want.X, got.X)
	massFloatBits(t, want.Y, got.Y)
	massFloatBits(t, want.Z, got.Z)
}

func TestMassAccumulatorAddMeshMatchesThreeLiftReference(t *testing.T) {
	box := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 3, 0), r3.NewVec(0, 3, 0),
		r3.NewVec(0, 0, 5), r3.NewVec(2, 0, 5), r3.NewVec(2, 3, 5), r3.NewVec(0, 3, 5),
	}
	boxTris := [][3]int{
		{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
		{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
		{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7},
	}
	near := math.Nextafter(1, 2)
	tiny := math.Ldexp(1, -900)
	cases := []struct {
		name           string
		verts          []r3.Vec
		tris           [][3]int
		walls          int
		anchor         r3.Vec
		delta, section float64
	}{
		{name: "empty and unused nonfinite vertex", verts: []r3.Vec{r3.NewVec(math.NaN(), 0, 0)}},
		{name: "closed box", verts: box, tris: boxTris, walls: 8},
		{name: "offset anchor and displacement", verts: box, tris: boxTris, walls: 8,
			anchor: r3.NewVec(-4, 8, 0.5), delta: 1e-9, section: 1e-9},
		{name: "reversed winding", verts: box, tris: reverseMassTris(boxTris), walls: 8,
			anchor: r3.NewVec(near, -tiny, 7)},
		{name: "signed cancellation", verts: box[:3], tris: [][3]int{{0, 1, 2}, {0, 2, 1}}, walls: 1},
		{name: "degenerate and repeated index", verts: box[:3], tris: [][3]int{{0, 0, 1}, {2, 2, 2}}, walls: 2},
		{name: "long dyadic denominators", verts: []r3.Vec{
			r3.NewVec(near, tiny, -tiny), r3.NewVec(2, tiny, 0),
			r3.NewVec(0, near, tiny), r3.NewVec(tiny, 0, near),
		}, tris: [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 1, 3}, {1, 2, 3}}, walls: 2,
			anchor: r3.NewVec(-near, tiny, -2)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			original := append([]r3.Vec(nil), tc.verts...)
			originalTris := append([][3]int(nil), tc.tris...)
			want := NewMassAccumulator(tc.anchor, tc.delta, tc.section, tc.section)
			got := NewMassAccumulator(tc.anchor, tc.delta, tc.section, tc.section)
			for k, tri := range tc.tris {
				referenceMassAdd(want, tc.verts[tri[0]], tc.verts[tri[1]], tc.verts[tri[2]], k < tc.walls)
			}
			got.AddMesh(tc.verts, tc.tris, tc.walls)
			for i := range tc.verts {
				massVecBits(t, original[i], tc.verts[i])
			}
			require.Equal(t, originalTris, tc.tris)
			for _, pair := range [][2]*big.Rat{{want.Vol6, got.Vol6}, {want.MomX, got.MomX},
				{want.MomY, got.MomY}, {want.MomZ, got.MomZ}} {
				require.Zero(t, pair[0].Cmp(pair[1]))
			}
			massFloatBits(t, want.PerturbAreaSum, got.PerturbAreaSum)
			massFloatBits(t, want.WallAreaSum, got.WallAreaSum)
			massFloatBits(t, want.WallAreaAbs, got.WallAreaAbs)
			massFloatBits(t, want.WallAreaSlack, got.WallAreaSlack)
			require.Equal(t, want.WallTerms, got.WallTerms)
			wv, wb := want.Volume(tc.verts, tc.tris)
			gv, gb := got.Volume(tc.verts, tc.tris)
			massFloatBits(t, wv, gv)
			massFloatBits(t, wb, gb)
			wc, wcb, we := want.Centroid(tc.verts, tc.tris)
			gc, gcb, ge := got.Centroid(tc.verts, tc.tris)
			require.Equal(t, we == nil, ge == nil)
			if we != nil {
				require.EqualError(t, ge, we.Error())
			}
			massVecBits(t, wc, gc)
			massFloatBits(t, wcb, gcb)
			wlo, whi, wbb, wok := want.Bounds()
			glo, ghi, gbb, gok := got.Bounds()
			require.Equal(t, wok, gok)
			massVecBits(t, wlo, glo)
			massVecBits(t, whi, ghi)
			massFloatBits(t, wbb, gbb)
			wa, wab := want.Area()
			ga, gab := got.Area()
			massFloatBits(t, wa, ga)
			massFloatBits(t, wab, gab)

			wallAreas := make([]WallArea, tc.walls)
			for k, tri := range tc.tris[:tc.walls] {
				a := proofarith.XptOf(tc.verts[tri[0]])
				b := proofarith.XptOf(tc.verts[tri[1]])
				c := proofarith.XptOf(tc.verts[tri[2]])
				lo, hi := WallTriangleArea(proofarith.Xsub(b, a), proofarith.Xsub(c, a))
				wallAreas[k] = WallArea{Lo: lo, Hi: hi}
			}
			cached := NewMassAccumulator(tc.anchor, tc.delta, tc.section, tc.section)
			cached.AddMeshWithWallAreas(tc.verts, tc.tris, wallAreas)
			cv, cb := cached.Volume(tc.verts, tc.tris)
			massFloatBits(t, gv, cv)
			massFloatBits(t, gb, cb)
			cc, ccb, ce := cached.Centroid(tc.verts, tc.tris)
			require.Equal(t, ge == nil, ce == nil)
			massVecBits(t, gc, cc)
			massFloatBits(t, gcb, ccb)
			ca, cab := cached.Area()
			massFloatBits(t, ga, ca)
			massFloatBits(t, gab, cab)
			require.Equal(t, got, cached)
		})
	}
}

func reverseMassTris(tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for i, tri := range tris {
		out[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	return out
}

// BenchmarkMassAccumulatorIndexed isolates the mass fold on a reused 64-cell
// shell; BenchmarkLoftCurvedCircle covers the complete public build.
func BenchmarkMassAccumulatorIndexed(b *testing.B) {
	const n = 64
	verts := make([]r3.Vec, 0, 2*n)
	for j := range n {
		angle := 2 * math.Pi * float64(j) / n
		c, s := math.Cos(angle), math.Sin(angle)
		verts = append(verts, r3.NewVec(10*c, 10*s, 0))
	}
	for j := range n {
		angle := 2 * math.Pi * float64(j) / n
		c, s := math.Cos(angle), math.Sin(angle)
		verts = append(verts, r3.NewVec(5*c, 5*s, 10))
	}
	tris := make([][3]int, 0, 4*n-4)
	for j := range n {
		next := (j + 1) % n
		tris = append(tris, [3]int{j, next, n + next}, [3]int{j, n + next, n + j})
	}
	walls := len(tris)
	for j := 1; j < n-1; j++ {
		tris = append(tris, [3]int{0, j + 1, j}, [3]int{n, n + j, n + j + 1})
	}
	wallAreas := make([]WallArea, walls)
	for k, tri := range tris[:walls] {
		a, c, d := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		ax := proofarith.XptOf(a)
		lo, hi := WallTriangleArea(proofarith.Xsub(proofarith.XptOf(c), ax), proofarith.Xsub(proofarith.XptOf(d), ax))
		wallAreas[k] = WallArea{Lo: lo, Hi: hi}
	}
	for _, mode := range []string{"reference", "indexed", "cached"} {
		b.Run(mode, func(b *testing.B) {
			for b.Loop() {
				m := NewMassAccumulator(r3.NewVec(0, 0, 0), 0, 0, 0)
				switch mode {
				case "indexed":
					m.AddMesh(verts, tris, walls)
				case "cached":
					m.AddMeshWithWallAreas(verts, tris, wallAreas)
				default:
					for k, tri := range tris {
						referenceMassAdd(m, verts[tri[0]], verts[tri[1]], verts[tri[2]], k < walls)
					}
				}
				if m.Vol6.Sign() == 0 {
					b.Fatal("empty mass")
				}
			}
		})
	}
}
