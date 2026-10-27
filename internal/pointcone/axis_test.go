package pointcone

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// General-axis point-cone arithmetic (agent-docs/revolve-axis-design.md §3.4,
// §3.5, §5.4). The fixtures use the irrational-length axis (1, 2, 0), the
// exactly congruent (0, 1, 0), and axes along X of length 2 and 1/2, on which
// every millimetre conversion is exact.

func mustAxis(t testing.TB, v r3.Vec) coneaxis.Axis {
	t.Helper()
	axis, ok := coneaxis.New(v)
	require.True(t, ok)
	return axis
}

// bigF is the 512-bit float of an exact rational, used only as an oracle.
func bigF(q *big.Rat) *big.Float { return new(big.Float).SetPrec(512).SetRat(q) }

// exactCrossing returns q = (s + k·σ)/c for p about axis at 512 bits.
func exactCrossing(p r3.Vec, axis coneaxis.Axis, apex, slope *big.Rat) *big.Float {
	sigma := new(big.Float).SetPrec(512).Sqrt(bigF(axis.RadialSq(p).Rat()))
	g := new(big.Float).SetPrec(512).Mul(bigF(slope), sigma)
	g.Add(g, bigF(axis.Axial(p).Rat()))
	return g.Quo(g, bigF(apex))
}

// floatGap is |a - b| less the oracle's own 512-bit rounding slack, a few
// units of 2^-500 relative to b, so a held value that the exact form says is
// exact is never charged for the oracle's rounding.
func floatGap(a *big.Float, b float64) float64 {
	d, _ := new(big.Float).SetPrec(512).Sub(a, new(big.Float).SetPrec(512).SetFloat64(b)).Float64()
	return max(0, math.Abs(d)-math.Ldexp(math.Max(1, math.Abs(b)), -500))
}

// About a general axis, FactorDyadic and FactorRat agree bit for bit, and the
// held crossing and point lie within their published errors of the exact
// crossing c/(s + k·σ) computed independently at 512 bits.
func TestPointConeFactorAboutAxisEnclosesExactCrossing(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(31, 32))
	for _, v := range []r3.Vec{{X: 1, Y: 2}, {Y: 1}, {X: 3, Y: 4}, {X: 1, Z: 2}} {
		axis := mustAxis(t, v)
		cones := pointConeTestCones(r, [2]*big.Rat{big.NewRat(45, 4), big.NewRat(1, 1)},
			[2]*big.Rat{big.NewRat(55, 2), big.NewRat(3, 7)})
		for i := range 600 {
			cone := cones[i%len(cones)]
			p := r3.Vec{X: (r.Float64()*2 - 0.5) * 10, Y: (r.Float64()*2 - 1) * 10, Z: (r.Float64()*2 - 1) * 10}
			if i%7 == 1 {
				// A point on the axis line: σ = 0 exactly.
				p = v.Scale(float64(1 + i%5))
			}
			sample, _ := ConstantsOf(axis, cone[0], cone[1])
			got, gotErr := FactorDyadic(p, axis, sample)
			want, wantErr := FactorRat(p, axis, cone[0], cone[1])
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error(), "%v about %v", p, v)
				continue
			}
			require.NoError(t, gotErr, "%v about %v", p, v)
			require.Equal(t, math.Float64bits(want.Q), math.Float64bits(got.Q), "q %v", p)
			require.Equal(t, math.Float64bits(want.QError), math.Float64bits(got.QError), "qError %v", p)
			require.Equal(t, math.Float64bits(want.PointError), math.Float64bits(got.PointError), "pointError %v", p)

			q := exactCrossing(p, axis, cone[0], cone[1])
			require.LessOrEqual(t, floatGap(q, got.Q), got.QError, "q %v about %v", p, v)
			alpha := new(big.Float).SetPrec(512).Quo(new(big.Float).SetPrec(512).SetInt64(1), q)
			held := p.Scale(got.Alpha)
			for _, c := range [][2]float64{{p.X, held.X}, {p.Y, held.Y}, {p.Z, held.Z}} {
				exact := new(big.Float).SetPrec(512).Mul(alpha, new(big.Float).SetPrec(512).SetFloat64(c[0]))
				require.LessOrEqual(t, floatGap(exact, c[1]), got.PointError, "point %v about %v", p, v)
			}
		}
	}
}

// The far-plane test widens each vertex by meshBound·|A|↑ in A-scaled units.
// About (1, 2, 0) a vertex whose margin above the far plane lies between
// meshBound and meshBound·√5 must refuse.
func TestAboveFarCapChargesAxisLength(t *testing.T) {
	t.Parallel()
	axis := mustAxis(t, r3.Vec{X: 1, Y: 2})
	far := big.NewRat(-5, 1)
	// s(p) = p·A = x for p on world X, so the margin is x + 5.
	require.False(t, AboveFarCap([]r3.Vec{{X: -3}}, 1, far, axis), "margin 2 lies below √5")
	require.True(t, AboveFarCap([]r3.Vec{{X: -2.5}}, 1, far, axis), "margin 2.5 clears √5")
	require.False(t, AboveFarCap([]r3.Vec{{X: -2.5}, {X: -5}}, 0, far, axis), "a vertex on the far plane")
}

// Ordered charges the mesh-bound move at meshBound·|A|↑ per unit of |dA| and
// |dR|.
func TestOrderedAboutAxisChargesAxisLength(t *testing.T) {
	t.Parallel()
	axis := mustAxis(t, r3.Vec{X: 1, Y: 2})
	one := big.NewRat(1, 1)
	four, eight := big.NewRat(4, 1), big.NewRat(8, 1)
	// For (1, 0, 0), s = 1 and σ = |(0, 0, 2)| = 2, so with slope 1 the
	// margin is dA·3 against the move 2·dA·meshBound·√5.
	verts := []r3.Vec{{}, {X: 1}}
	require.True(t, Ordered(four, one, eight, one, verts, 0.5, axis), "3 > 2·0.5·√5")
	require.False(t, Ordered(four, one, eight, one, verts, 1, axis), "3 < 2·√5")
	require.False(t, Ordered(eight, one, four, one, verts, 0, axis), "reversed cones")
}

// ToeHeelOrdered widens the exact axial range by meshBound·|A|↑.
func TestToeHeelOrderedAboutAxisChargesAxisLength(t *testing.T) {
	t.Parallel()
	axis := mustAxis(t, r3.Vec{X: 1, Y: 2})
	// With toe (4, 1) and heel (10, 2) the margin is 2 + s, positive for
	// s > -2. The vertex has s = -1.
	toe, heel := big.NewRat(4, 1), big.NewRat(10, 1)
	one, two := big.NewRat(1, 1), big.NewRat(2, 1)
	verts := []r3.Vec{{X: -1}}
	require.True(t, ToeHeelOrdered(toe, one, heel, two, verts, 0.4, axis), "-1 - 0.4·√5 > -2")
	require.False(t, ToeHeelOrdered(toe, one, heel, two, verts, 0.6, axis), "-1 - 0.6·√5 < -2")
}

// straddlingCells returns random cells whose Y range contains zero and whose Z
// range is positive, so the canonical radial minimum is the smallest Z.
func straddlingCells(r *rand.Rand, n int) [][3]r3.Vec {
	cells := make([][3]r3.Vec, n)
	for i := range cells {
		for j := range 3 {
			y := 0.25 + 2*r.Float64()
			if j == 0 {
				y = -y
			}
			cells[i][j] = r3.Vec{X: 3 + 5*r.Float64(), Y: y, Z: 1 + 3*r.Float64()}
		}
	}
	return cells
}

// About an axis along X of length 2 or 1/2, every conversion of §3.4 is
// exact: the axial minimum, the radial minimum of a cell straddling y = 0, and
// the bound apex a↑ = c/|A|. The cell geometry and the triangle bound then
// equal the canonical ones bit for bit. The reach stays the millimetre distance
// |p| on every axis.
func TestTriangleBoundAboutScaledAxisMatchesCanonical(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(41, 42))
	cells := straddlingCells(r, 200)
	for _, length := range []float64{2, 0.5} {
		axis := mustAxis(t, r3.Vec{X: length})
		for _, cone := range [][2]*big.Rat{{big.NewRat(5, 1), big.NewRat(1, 1)}, {big.NewRat(12, 1), big.NewRat(3, 7)}} {
			scaledApex := new(big.Rat).Mul(cone[0], proofarith.FloatRat(length))
			canonical, _ := ConstantsOf(coneaxis.X(), cone[0], cone[1])
			_, bound := ConstantsOf(axis, scaledApex, cone[1])
			require.NotNil(t, bound)
			for _, tri := range cells {
				wantGeom, gotGeom := CellGeometry(tri, coneaxis.X()), CellGeometry(tri, axis)
				require.Equal(t, wantGeom, gotGeom, "cell %v about %v", tri, length)
				wantCap, wantAlpha, wantLip, wantErr := TriangleBound(wantGeom, canonical)
				gotCap, gotAlpha, gotLip, gotErr := TriangleBound(gotGeom, bound)
				require.NoError(t, wantErr)
				require.NoError(t, gotErr)
				require.Equal(t, math.Float64bits(wantCap), math.Float64bits(gotCap))
				require.Equal(t, math.Float64bits(wantAlpha), math.Float64bits(gotAlpha))
				require.Equal(t, math.Float64bits(wantLip), math.Float64bits(gotLip))
			}
		}
	}
}

// About the irrational-length axis (1, 2, 0), and the same axis scaled by
// 1/16, the cell bound encloses the exact departure of the crossing map
// F(p) = p·c/(s + k·σ) from the chord through the cell's vertex images. The
// departure is computed independently at 512 bits on a barycentric grid.
func TestTriangleBoundAboutAxisEnclosesCrossingDeparture(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(51, 52))
	image := func(p r3.Vec, axis coneaxis.Axis, apex, slope *big.Rat) [3]*big.Float {
		alpha := new(big.Float).SetPrec(512).Quo(new(big.Float).SetPrec(512).SetInt64(1),
			exactCrossing(p, axis, apex, slope))
		var out [3]*big.Float
		for i, c := range []float64{p.X, p.Y, p.Z} {
			out[i] = new(big.Float).SetPrec(512).Mul(alpha, new(big.Float).SetPrec(512).SetFloat64(c))
		}
		return out
	}
	for _, v := range []r3.Vec{{X: 1, Y: 2}, {X: 1.0 / 16, Y: 2.0 / 16}} {
		axis := mustAxis(t, v)
		norm := math.Sqrt(v.Dot(v))
		for _, cone := range [][2]float64{{5, 1}, {12, 0.75}} {
			// The apex lies cone[0] mm along the axis: c = cone[0]·|A|, read
			// as the exact rational of its float.
			apex := proofarith.FloatRat(cone[0] * norm)
			slope := proofarith.FloatRat(cone[1])
			sample, bound := ConstantsOf(axis, apex, slope)
			require.NotNil(t, sample)
			for range 40 {
				// A cell about 8 mm along the axis and 3 to 5 mm from it.
				hat := v.Scale(1 / norm)
				e := r3.Vec{X: -hat.Y, Y: hat.X}
				base := hat.Scale(7 + 2*r.Float64()).Add(e.Scale(3 + 2*r.Float64()))
				var tri [3]r3.Vec
				for j := range tri {
					tri[j] = base.Add(r3.Vec{X: 0.4 * r.Float64(), Y: 0.4 * r.Float64(), Z: 0.4 * r.Float64()})
				}
				capBound, _, _, err := TriangleBound(CellGeometry(tri, axis), bound)
				require.NoError(t, err)
				corners := [3][3]*big.Float{}
				for j, p := range tri {
					corners[j] = image(p, axis, apex, slope)
				}
				const steps = 6
				for i := 0; i <= steps; i++ {
					for j := 0; i+j <= steps; j++ {
						l0, l1 := float64(i)/steps, float64(j)/steps
						l2 := 1 - l0 - l1
						p := tri[0].Scale(l0).Add(tri[1].Scale(l1)).Add(tri[2].Scale(l2))
						got := image(p, axis, apex, slope)
						for k := range 3 {
							chord := new(big.Float).SetPrec(512)
							for m, l := range []float64{l0, l1, l2} {
								chord.Add(chord, new(big.Float).SetPrec(512).Mul(corners[m][k],
									new(big.Float).SetPrec(512).SetFloat64(l)))
							}
							d, _ := new(big.Float).SetPrec(512).Sub(got[k], chord).Float64()
							require.LessOrEqual(t, math.Abs(d), capBound, "cell %v about %v", tri, v)
						}
					}
				}
			}
		}
	}
}
