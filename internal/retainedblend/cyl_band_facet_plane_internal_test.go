package retainedblend

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// placedBand is flatBand cutting the facet q, with the band's planes moved by
// place to where q lies.
func placedBand(q [3]r3.Vec, place func(r3.Vec) r3.Vec) (*work, *wallBand, faceLoops) {
	w, b, f := flatBand(q[:]...)
	for _, e := range append([]*bandEnd{b.main, b.ends[0]}, b.others...) {
		o := place(e.g.o)
		n, _ := place(e.g.n.Add(e.g.o)).Sub(o).Normalize()
		e.g = plane{n: n, o: o}
	}
	return w, b, f
}

// tiltedEndFace carries the flat facet's (x, y, z) to (z, x, y), the end face
// x = 0, then tilts it by theta about y. permY carries the result's x to y, so
// the facet normal lies near y.
func tiltedEndFace(theta float64, permY bool) func(r3.Vec) r3.Vec {
	c, s := math.Cos(theta), math.Sin(theta)
	return func(v r3.Vec) r3.Vec {
		x, y, z := v.Z, v.X, v.Y
		p := r3.NewVec(c*x+s*z, y, -s*x+c*z)
		if permY {
			return r3.NewVec(p.Z, p.X, p.Y)
		}
		return p
	}
}

func exactCoords(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

// exactNormalOf is the exact normal of the triangle a, b, c of float corners.
func exactNormalOf(a, b, c r3.Vec) [3]*big.Rat {
	A, B, C := exactCoords(a), exactCoords(b), exactCoords(c)
	var e1, e2 [3]*big.Rat
	for i := range 3 {
		e1[i], e2[i] = new(big.Rat).Sub(B[i], A[i]), new(big.Rat).Sub(C[i], A[i])
	}
	mul := func(p, q, r, s *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(p, q), new(big.Rat).Mul(r, s))
	}
	return [3]*big.Rat{mul(e1[1], e2[2], e1[2], e2[1]), mul(e1[2], e2[0], e1[0], e2[2]), mul(e1[0], e2[1], e1[1], e2[0])}
}

func ratDot(a, b [3]*big.Rat) int {
	sum := new(big.Rat)
	for i := range 3 {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum.Sign()
}

// requireWoundAsFacet requires every triangle w emits to wind as the facet q
// does in exact arithmetic.
func requireWoundAsFacet(t *testing.T, w *work, q [3]r3.Vec) {
	t.Helper()
	fn := exactNormalOf(q[0], q[1], q[2])
	for _, tr := range w.out {
		tn := exactNormalOf(w.verts[tr[0]], w.verts[tr[1]], w.verts[tr[2]])
		require.Equal(t, 1, ratDot(tn, fn), "triangle %v is not wound as its facet", tr)
	}
}

// TestCutBandFacetWithoutProvenNormalHasNoPlane cuts near-collinear facets of
// edges about 100 mm whose exact normal is 1e-14 to 1e-13 long. The float
// cross product of each is rounding noise of the wrong sign, 1.8e-12 long, past
// Normalize's cutoff: framed from it, the facet would emit triangles wound
// against it. Each refuses, on the raw path and on the unit-normal retry
// alike (retained edge blend §4.3).
func TestCutBandFacetWithoutProvenNormalHasNoPlane(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name  string
		q     [3]r3.Vec
		place func(r3.Vec) r3.Vec
	}{
		{"end face tilted 1e-10", [3]r3.Vec{
			r3.NewVec(-2.0064278552988663e-09, -46.494035296706805, -20.06427855298866),
			r3.NewVec(9.902060252362147e-09, 48.41142816784313, 99.02060252362146),
			r3.NewVec(9.16119167639879e-09, 42.507028303439704, 91.61191676398789)}, tiltedEndFace(1e-10, false)},
		{"end face tilted 1e-14", [3]r3.Vec{
			r3.NewVec(-4.557109819819535e-13, -42.354631120916444, -45.57109819819535),
			r3.NewVec(6.07014501371204e-13, 38.90042164821563, 60.701450137120396),
			r3.NewVec(7.638812992618639e-13, 50.89431860466475, 76.3881299261864)}, tiltedEndFace(1e-14, false)},
		{"end face tilted 1e-14, other facet", [3]r3.Vec{
			r3.NewVec(-4.2755319249774267e-13, -27.379226970045224, -42.75531924977427),
			r3.NewVec(7.490233123789102e-13, 28.522501038133104, 74.90233123789102),
			r3.NewVec(9.366073773090374e-13, 37.435031020611184, 93.66073773090375)}, tiltedEndFace(1e-14, false)},
		{"end face tilted 1e-6", [3]r3.Vec{
			r3.NewVec(-1.9440248339392064e-05, -39.5933538058627, -19.440248339385583),
			r3.NewVec(9.262732918975647e-05, 49.566292035990465, 92.6273291897256),
			r3.NewVec(7.573808039058763e-05, 36.12940444843489, 75.73808039056239)}, tiltedEndFace(1e-6, false)},
		{"normal near y, tilted 1e-6", [3]r3.Vec{
			r3.NewVec(-43.47405765326163, -4.347405765327612e-05, -25.745338906862695),
			r3.NewVec(80.96166431324053, 8.096166431326752e-05, 30.926182611051818),
			r3.NewVec(72.22898454353346, 7.222898454355753e-05, 26.949075059803505)}, tiltedEndFace(1e-6, true)},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			w, b, f := placedBand(c.q, c.place)
			_, err := w.cutBand(f, b)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, "a wall facet has no plane")
		})
	}
}

// TestCutBandFacetAboveTheNormalBoundIsWoundAsItself cuts near-collinear
// facets whose float normal is only just longer than twice its proven
// rounding bound and checks, against exact arithmetic, that none refuses for
// want of a plane and every emitted triangle winds as its facet.
func TestCutBandFacetAboveTheNormalBoundIsWoundAsItself(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name  string
		place func(r3.Vec) r3.Vec
	}{
		{"flat", func(v r3.Vec) r3.Vec { return v }},
		{"end face", tiltedEndFace(0, false)},
		{"end face tilted", tiltedEndFace(1e-10, false)},
		{"normal near y", tiltedEndFace(1e-6, true)},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			rng := rand.New(rand.NewPCG(3260, 1))
			framed := 0
			for range 20000 {
				p0 := r3.NewVec(-rng.Float64()*50, -rng.Float64()*50, 0)
				p1 := r3.NewVec(rng.Float64()*50, rng.Float64()*100, 0)
				d := p1.Sub(p0)
				// Offset the third corner so the float normal is 2 to 8 times its bound.
				rel := math.Pow(10, -13+3*rng.Float64())
				if rng.IntN(2) == 0 {
					rel = -rel
				}
				p2 := p0.Add(d.Scale(rng.Float64()*1.4 - 0.2)).Add(r3.NewVec(-d.Y, d.X, 0).Scale(rel))
				q := [3]r3.Vec{c.place(p0), c.place(p1), c.place(p2)}
				nrm := q[1].Sub(q[0]).Cross(q[2].Sub(q[0]))
				ratio := nrm.Len() / proofbound.FacetAreaTermSlop(q[0], q[1], q[2])
				if ratio <= 2 || ratio > 8 {
					continue
				}
				w, b, f := placedBand(q, c.place)
				_, err := w.cutBand(f, b)
				if err != nil {
					// The cut may leave a degenerate piece, but never lacks a plane.
					require.NotContains(t, err.Error(), "no plane")
					continue
				}
				if len(w.out) == 1 && w.out[0] == [3]int{0, 1, 2} {
					continue
				}
				framed++
				requireWoundAsFacet(t, w, q)
			}
			require.Positive(t, framed, "no facet reached the frame")
		})
	}
}
