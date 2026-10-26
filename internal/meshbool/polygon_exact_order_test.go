package meshbool_test

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// exactProjectedSegment uses only big.Rat arithmetic, including its determinant.
func exactProjectedSegment(a, b, p meshbool.Xp2) (bool, bool) {
	dx := new(big.Rat).Sub(b.U, a.U)
	dy := new(big.Rat).Sub(b.V, a.V)
	px := new(big.Rat).Sub(p.U, a.U)
	py := new(big.Rat).Sub(p.V, a.V)
	det := new(big.Rat).Sub(new(big.Rat).Mul(dx, py), new(big.Rat).Mul(dy, px))
	if det.Sign() != 0 {
		return false, false
	}
	between := func(x, lo, hi *big.Rat) bool {
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		return x.Cmp(lo) >= 0 && x.Cmp(hi) <= 0
	}
	on := between(p.U, a.U, b.U) && between(p.V, a.V, b.V)
	atA := p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0
	atB := p.U.Cmp(b.U) == 0 && p.V.Cmp(b.V) == 0
	return on, on && !atA && !atB
}

// exactProjectedPolygon casts a ray by computing each exact intersection,
// independently of the production orientation-based crossing test.
func exactProjectedPolygon(poly []meshbool.Xp2, p meshbool.Xp2) (bool, bool) {
	inside := false
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		if on, _ := exactProjectedSegment(a, b, p); on {
			return false, true
		}
		if (a.V.Cmp(p.V) <= 0) == (b.V.Cmp(p.V) <= 0) {
			continue
		}
		tt := new(big.Rat).Quo(new(big.Rat).Sub(p.V, a.V), new(big.Rat).Sub(b.V, a.V))
		x := new(big.Rat).Add(a.U, new(big.Rat).Mul(tt, new(big.Rat).Sub(b.U, a.U)))
		if x.Cmp(p.U) > 0 {
			inside = !inside
		}
	}
	return inside, false
}

func TestProjectedCoordinateOrderingMatchesExactGeometry(t *testing.T) {
	t.Parallel()
	pow := func(exp int) *big.Rat {
		v := new(big.Int).Lsh(big.NewInt(1), uint(max(exp, -exp)))
		if exp < 0 {
			return new(big.Rat).SetFrac(big.NewInt(1), v)
		}
		return new(big.Rat).SetInt(v)
	}
	for _, scale := range []struct {
		name         string
		origin, step *big.Rat
	}{
		{"rational", big.NewRat(1, 3), big.NewRat(1, 7)},
		{"equal roundings", big.NewRat(1, 1), pow(-80)},
		{"negative equal roundings", big.NewRat(-1, 1), pow(-80)},
		{"rounded zero", new(big.Rat), pow(-1100)},
		{"subnormal", new(big.Rat), pow(-1074)},
		{"overflow", pow(1100), pow(1100)},
	} {
		t.Run(scale.name, func(t *testing.T) {
			coord := func(v int) *big.Rat {
				return new(big.Rat).Add(scale.origin, new(big.Rat).Mul(big.NewRat(int64(v), 2), scale.step))
			}
			for _, kind := range []string{"rational", "homogeneous", "literal"} {
				t.Run(kind, func(t *testing.T) {
					point := func(x, y int) meshbool.Xp2 {
						u, v := coord(x), coord(y)
						switch kind {
						case "rational":
							return meshbool.NewXP2(u, v)
						case "homogeneous":
							p := proof.Xpt{
								X: new(big.Int).Mul(u.Num(), v.Denom()),
								Y: new(big.Int).Mul(v.Num(), u.Denom()),
								Z: new(big.Int), W: new(big.Int).Mul(u.Denom(), v.Denom()),
							}
							return meshbool.NewXP2FromXpt(p, 0, 1)
						default:
							// Literal mirrors have no constructor proof and must be ignored.
							return meshbool.Xp2{U: u, V: v, Fu: math.NaN(), Fv: 1234}
						}
					}
					poly := []meshbool.Xp2{point(0, 0), point(0, 0), point(6, 0), point(6, 6), point(2, 2), point(0, 6)}
					for winding := range 2 {
						for x := -1; x <= 7; x++ {
							for y := -1; y <= 7; y++ {
								p := point(x, y)
								for i, a := range poly {
									b := poly[(i+1)%len(poly)]
									wantOn, wantInterior := exactProjectedSegment(a, b, p)
									gotOn, gotInterior := meshbool.OnSegment2(a, b, p)
									require.Equal(t, wantOn, gotOn, "winding %d edge %d query %d,%d", winding, i, x, y)
									require.Equal(t, wantInterior, gotInterior, "winding %d edge %d query %d,%d", winding, i, x, y)
								}
								wantInside, wantBoundary := exactProjectedPolygon(poly, p)
								gotInside, gotBoundary, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(t.Context()), poly, p)
								require.NoError(t, err)
								require.Equal(t, wantInside, gotInside, "winding %d query %d,%d", winding, x, y)
								require.Equal(t, wantBoundary, gotBoundary, "winding %d query %d,%d", winding, x, y)
							}
						}
						slices.Reverse(poly)
					}
				})
			}
		})
	}
}
