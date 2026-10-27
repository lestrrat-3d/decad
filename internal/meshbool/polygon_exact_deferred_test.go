package meshbool

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestDeferredProjectionsMatchRationalGeometry(t *testing.T) {
	pow := func(exponent int) *big.Rat {
		n := new(big.Int).Lsh(big.NewInt(1), uint(max(exponent, -exponent)))
		if exponent < 0 {
			return new(big.Rat).SetFrac(big.NewInt(1), n)
		}
		return new(big.Rat).SetInt(n)
	}
	for _, scale := range []struct {
		name         string
		origin, step *big.Rat
	}{
		{"rational", big.NewRat(1, 3), big.NewRat(1, 7)},
		{"equal positive rounding", big.NewRat(1, 1), pow(-120)},
		{"equal negative rounding", big.NewRat(-1, 1), pow(-120)},
		{"rounded zero", new(big.Rat), pow(-1100)},
		{"subnormal", new(big.Rat), pow(-1074)},
		{"overflow", pow(1100), pow(1100)},
	} {
		t.Run(scale.name, func(t *testing.T) {
			point := func(x, y int) Xp2 {
				u := new(big.Rat).Add(scale.origin, new(big.Rat).Mul(big.NewRat(int64(x), 1), scale.step))
				v := new(big.Rat).Add(scale.origin, new(big.Rat).Mul(big.NewRat(int64(y), 1), scale.step))
				hu := new(big.Int).Mul(u.Num(), v.Denom())
				hv := new(big.Int).Mul(v.Num(), u.Denom())
				hw := new(big.Int).Mul(u.Denom(), v.Denom())
				return newDeferredXP2(hu, hv, hw, new(big.Rat), new(big.Rat))
			}
			for _, xy := range [][2]int{{-1, -1}, {0, 0}, {1, 1}, {2, 0}, {2, 2}, {3, 3}, {4, 0}, {5, 5}} {
				x, y := xy[0], xy[1]
				a, b, p := point(0, 0), point(4, 0), point(x, y)
				on, interior := OnSegment2(a, b, p)
				require.Equal(t, y == 0 && x >= 0 && x <= 4, on, "point %v", xy)
				require.Equal(t, y == 0 && x > 0 && x < 4, interior, "point %v", xy)
				inside, boundary, err := PointInPoly2(proofbound.NewWorkBudget(t.Context()),
					[]Xp2{point(0, 0), point(4, 0), point(0, 4)}, point(x, y))
				require.NoError(t, err)
				closed := x >= 0 && y >= 0 && x+y <= 4
				require.Equal(t, closed && x > 0 && y > 0 && x+y < 4, inside, "point %v", xy)
				require.Equal(t, closed && (x == 0 || y == 0 || x+y == 4), boundary, "point %v", xy)
				p = point(x, y)
				eager := NewXP2(new(big.Rat).SetFrac(p.Hu, p.Hw), new(big.Rat).SetFrac(p.Hv, p.Hw))
				require.Equal(t, math.Float64bits(eager.Fu), math.Float64bits(p.Fu))
				require.Equal(t, math.Float64bits(eager.Fv), math.Float64bits(p.Fv))
				require.Equal(t, eager.FloatFinite, p.FloatFinite)
				// A rational-only third point forces the mixed exact path.
				eagerA, eagerB := point(0, 0), point(4, 0)
				materializeXP2(eagerA)
				materializeXP2(eagerB)
				want := Cross2x(NewXP2(eagerA.U, eagerA.V), NewXP2(eagerB.U, eagerB.V), eager)
				require.Zero(t, want.Cmp(Cross2x(point(0, 0), point(4, 0), eager)))
				require.Equal(t, want.Sign(), Cross2xSign(point(0, 0), point(4, 0), eager))
				midpoint := deferredMidpointXP2(point(0, 0), point(x, y))
				materializeXP2(midpoint)
				wantU := new(big.Rat).Quo(new(big.Rat).Add(scale.origin, eager.U), big.NewRat(2, 1))
				wantV := new(big.Rat).Quo(new(big.Rat).Add(scale.origin, eager.V), big.NewRat(2, 1))
				require.Zero(t, wantU.Cmp(midpoint.U))
				require.Zero(t, wantV.Cmp(midpoint.V))
			}
			pts := []Xp2{point(0, 0), point(4, 0), point(4, 4), point(0, 4)}
			sign, err := PolyArea2Sign(proofbound.NewWorkBudget(t.Context()), pts)
			require.NoError(t, err)
			require.Equal(t, 1, sign)
			triangles, err := EarClipX(proofbound.NewWorkBudget(t.Context()), pts, []int{0, 1, 2, 3})
			require.NoError(t, err)
			require.Equal(t, [][3]int{{3, 0, 1}, {1, 2, 3}}, triangles)
		})
	}
}
