package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceFacetBox is BuildFacetBox with the area decided only by the float
// filter and then the rational cross over freshly lifted projections: the
// construction the float-side exact legs (projectedAreaSign) must reproduce
// field for field.
func referenceFacetBox(pm *ParityMesh, u, v, ti int) ParityFacetBox {
	tri := pm.Tris[ti]
	a, b, c := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
	au, av := CoordOf(a, u), CoordOf(a, v)
	bu, bv := CoordOf(b, u), CoordOf(b, v)
	cu, cv := CoordOf(c, u), CoordOf(c, v)
	box := ParityFacetBox{
		MinU:  math.Min(au, math.Min(bu, cu)),
		MaxU:  math.Max(au, math.Max(bu, cu)),
		MinV:  math.Min(av, math.Min(bv, cv)),
		MaxV:  math.Max(av, math.Max(bv, cv)),
		Built: true,
	}
	if proofbound.IsNonFinite(box.MinU) || proofbound.IsNonFinite(box.MaxU) ||
		proofbound.IsNonFinite(box.MinV) || proofbound.IsNonFinite(box.MaxV) {
		return box
	}
	left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
	bound := ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + ParityAreaFloor
	if det := left - right; det > bound || det < -bound {
		box.Nondegenerate = true
		return box
	}
	box.Nondegenerate = Cross2xSign(newXP2FromFloats(au, av), newXP2FromFloats(bu, bv), newXP2FromFloats(cu, cv)) != 0
	return box
}

// paritySlivers is triangles a, a + d, a + 2^20 d + e on a 1/64 lattice: a
// third with e = 0, exactly collinear on a slanted line; a third with e one
// 2^-30 step off that line, a true area the float filter cannot prove; and a
// third where a sits 2^-40 off the lattice, so a difference rounds and only
// the rational leg can answer.
func paritySlivers(rng *rand.Rand, n int) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	lattice := func(span int) float64 { return float64(rng.IntN(2*span+1)-span) / 64 }
	for len(tris) < n {
		a := r3.Vec{X: lattice(256), Y: lattice(256), Z: lattice(256)}
		d := r3.Vec{X: lattice(64), Y: lattice(64), Z: lattice(64)}
		if d.X == 0 || d.Y == 0 || d.Z == 0 {
			continue
		}
		var e r3.Vec
		switch len(tris) % 3 {
		case 1:
			e = r3.Vec{X: 0x1p-30 * float64(rng.IntN(3)-1), Y: 0x1p-30 * float64(rng.IntN(3)-1), Z: 0x1p-30}
		case 2:
			a.X += 0x1p-40
		}
		b := a.Add(d)
		c := a.Add(d.Scale(0x1p20)).Add(e)
		k := len(verts)
		verts = append(verts, a, b, c)
		tris = append(tris, [3]int{k, k + 1, k + 2})
	}
	return verts, tris
}

func parityScanFixtures() []parityIndexFixture {
	rng := rand.New(rand.NewPCG(0x51c3, 0x0b0e))
	fx := parityIndexFixtures()
	v, t := parityGridCube(4)
	fx = append(fx, parityIndexFixture{`grid cube`, v, t})
	v, t = paritySlivers(rng, 150)
	fx = append(fx, parityIndexFixture{`slivers`, v, t})
	return fx
}

// TestParityFacetBoxMatchesTheRationalArea requires every box, on every axis
// and every fixture, to equal the rational-area construction, and requires
// the fixtures to reach every leg of the area decision, so a leg that
// answered wrongly could not go unseen.
func TestParityFacetBoxMatchesTheRationalArea(t *testing.T) {
	t.Parallel()
	var floatLeg, shortcutLeg, expansionZero, expansionArea, rationalLeg int
	for _, fx := range parityScanFixtures() {
		pm := NewParityMesh(fx.verts, fx.tris)
		for axis := range 3 {
			ray := AxisRays[2*axis]
			for ti := range fx.tris {
				want := referenceFacetBox(pm, ray.U, ray.V, ti)
				got := pm.BuildFacetBox(ray.Axis, ray.U, ray.V, ti)
				require.Equal(t, want, got, `%s: axis %d facet %d`, fx.name, ray.Axis, ti)

				tri := fx.tris[ti]
				a, b, c := fx.verts[tri[0]], fx.verts[tri[1]], fx.verts[tri[2]]
				au, av := CoordOf(a, ray.U), CoordOf(a, ray.V)
				bu, bv := CoordOf(b, ray.U), CoordOf(b, ray.V)
				cu, cv := CoordOf(c, ray.U), CoordOf(c, ray.V)
				left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
				bound := ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + ParityAreaFloor
				if det := left - right; det > bound || det < -bound {
					floatLeg++
					continue
				}
				_, shortcut := projectedAreaSign(au, av, bu, bv, cu, cv)
				s, expansion := proof.Orient2SignExpansion(au, av, bu, bv, cu, cv)
				switch {
				case au == bu && bu == cu || av == bv && bv == cv ||
					au == bu && av == bv || au == cu && av == cv || bu == cu && bv == cv:
					shortcutLeg++
				case expansion && s == 0:
					expansionZero++
				case expansion:
					expansionArea++
				default:
					require.False(t, shortcut)
					rationalLeg++
				}
			}
		}
	}
	require.Positive(t, floatLeg, `the float filter must decide some areas`)
	require.Positive(t, shortcutLeg, `some projections must collapse by coordinate equality`)
	require.Positive(t, expansionZero, `some slanted projections must collapse exactly`)
	require.Positive(t, expansionArea, `some areas must need the expansion`)
	require.Positive(t, rationalLeg, `some areas must still need the rational leg`)
}

// TestSingleUseParityScanMatchesThePreparedScan is the end-to-end
// differential for the plain scan: MeshParityContext, whose single-use cache
// builds every box without storing it, against one prepared cache kept warm
// across every query and against the unfiltered exact kernel. Queries include
// every vertex (on the boundary, at shared vertices), centroids, points on a
// degenerate facet's line beyond its segment, points on and a hair beside
// edges, and random points.
func TestSingleUseParityScanMatchesThePreparedScan(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	rng := rand.New(rand.NewPCG(0x7a11, 0x5ca9))
	for _, fx := range parityScanFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			queries := parityIndexQueries(rng, fx)
			hair := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 70))
			for _, tri := range fx.tris {
				a, b := fx.verts[tri[0]], fx.verts[tri[1]]
				mid := func(p, q float64, off *big.Rat) *big.Rat {
					m := new(big.Rat).Add(fr(p), fr(q))
					m.Quo(m, big.NewRat(2, 1))
					return m.Add(m, off)
				}
				zero := new(big.Rat)
				minus := new(big.Rat).Neg(hair)
				queries = append(queries,
					ratXpt(mid(a.X, b.X, zero), mid(a.Y, b.Y, zero), mid(a.Z, b.Z, zero)),
					ratXpt(mid(a.X, b.X, hair), mid(a.Y, b.Y, minus), mid(a.Z, b.Z, zero)),
					ratXpt(mid(a.X, b.X, zero), mid(a.Y, b.Y, hair), mid(a.Z, b.Z, minus)))
			}
			subset := parityAllFacets(len(fx.tris))
			prepared := NewParityMesh(fx.verts, fx.tris)
			plain := NewParityMesh(fx.verts, fx.tris)
			plain.Unfiltered = true
			for qi, p := range queries {
				want := answerOf(MeshParityPreparedContext(ctx, p, plain, subset))
				require.Equal(t, want, answerOf(MeshParityContext(ctx, p, fx.verts, fx.tris, subset)),
					`single-use, query %d`, qi)
				require.Equal(t, want, answerOf(MeshParityPreparedContext(ctx, p, prepared, subset)),
					`prepared, query %d`, qi)
			}
		})
	}
}
