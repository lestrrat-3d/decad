package loftrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// boreNormalFixture names one bore wall: its section frame, placement,
// plane-local circle center, axial level and radius.
type boreNormalFixture struct {
	name   string
	frame  r3.Frame
	xform  r3.Transform
	u, v   float64
	level  float64
	radius float64
}

func boreNormalFixtures(t *testing.T) []boreNormalFixture {
	t.Helper()
	frame := func(origin, u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		return f
	}
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Radians(0.7))
	require.NoError(t, err)
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	return []boreNormalFixture{
		// Only the center lift rounds: 1e9 + 0.427 has no float.
		{"rounded center", frame(r3.NewVec(1e9, 0, 0), x, y), r3.Identity(), .427, 0, 2, 1},
		// Only the frame map departs from orthonormal; the held axis is
		// exactly the denoted ruling direction.
		{"tilted frame map", frame(r3.Vec{}, r3.NewVec(1, 0, 4), y), r3.Identity(), 0, 0, 2, 1},
		// The placed axis rounds away from the exact image of N, and the
		// long level turns that tilt into the largest departure.
		{"rotated placement", frame(r3.Vec{}, x, r3.NewVec(0, 1, 1)), rotation, 0, 0, 1e6, 1},
	}
}

// TestBoreCylinderNormalBoundCoversTrueDeparture compares the bound with the
// departure the true wall actually has, computed exactly over rationals and
// then in 512-bit floats, at points around both rims.
func TestBoreCylinderNormalBoundCoversTrueDeparture(t *testing.T) {
	for _, tc := range boreNormalFixtures(t) {
		t.Run(tc.name, func(t *testing.T) {
			bound, ok := loftrecord.BoreCylinderNormalBound(tc.frame, tc.xform, tc.u, tc.v, tc.level, tc.radius)
			require.True(t, ok)
			departure := trueBoreDeparture(t, tc)
			require.Positive(t, departure, "the fixture must have a departure for the bound to cover")
			require.LessOrEqual(t, departure, bound)
		})
	}
}

func TestBoreCylinderNormalBoundExactWall(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(3, -2, 5), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	tc := boreNormalFixture{"exact", frame, shift, 0.5, -1.25, 4, 1}
	bound, ok := loftrecord.BoreCylinderNormalBound(tc.frame, tc.xform, tc.u, tc.v, tc.level, tc.radius)
	require.True(t, ok)
	require.Zero(t, bound)
	require.Zero(t, trueBoreDeparture(t, tc))
}

func TestBoreCylinderNormalBoundRefusals(t *testing.T) {
	far, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	// The center rounds by more than half this radius, so the tag can face
	// the wrong way at some wall point.
	_, ok := loftrecord.BoreCylinderNormalBound(far, r3.Identity(), .427, 0, 2, 1e-8)
	require.False(t, ok)
	_, ok = loftrecord.BoreCylinderNormalBound(far, r3.Identity(), .427, 0, 2, 0)
	require.False(t, ok)
	_, ok = loftrecord.BoreCylinderNormalBound(far, r3.Identity(), .427, 0, math.Inf(1), 1)
	require.False(t, ok)
}

// trueBoreDeparture is the largest distance, over sample points of the true
// wall, between the unit normal of the denoted wall and the unit radial of
// the tag the builder publishes.
func trueBoreDeparture(t *testing.T, tc boreNormalFixture) float64 {
	t.Helper()
	l, err := massmoment.PrismRotation(tc.frame, tc.xform)
	require.NoError(t, err)
	basis := tc.xform.Basis()
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	origin := ratVec(tc.frame.Origin())
	translation := ratVec(tc.xform.Translation())
	// c = B·(O + u·U + v·V) + t, with B·U and B·V the first two columns of L.
	var center [3]*big.Rat
	for i := range center {
		sum := new(big.Rat).Set(translation[i])
		for k := range columns {
			sum.Add(sum, new(big.Rat).Mul(rat(vecComponent(columns[k], i)), origin[k]))
		}
		sum.Add(sum, new(big.Rat).Mul(l[i][0], rat(tc.u)))
		sum.Add(sum, new(big.Rat).Mul(l[i][1], rat(tc.v)))
		center[i] = sum
	}
	pp := prismrecord.Payload{Frame: tc.frame, Xform: tc.xform}
	tagOrigin := ratVec(pp.Point(tc.u, tc.v, 0))
	tagAxis := ratVec(pp.Dir(0, 0, 1))
	rho := rat(tc.radius)
	worst := new(big.Float).SetPrec(512)
	// Rational points of the circle: ((1−s²)/(1+s²), 2s/(1+s²)) and (−1, 0).
	var radials [][2]*big.Rat
	for _, s := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), big.NewRat(1, 1), big.NewRat(-1, 2),
		big.NewRat(7, 3), big.NewRat(-5, 2), big.NewRat(2, 9), big.NewRat(-9, 7)} {
		den := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(s, s))
		cx := new(big.Rat).Quo(new(big.Rat).Sub(big.NewRat(1, 1), new(big.Rat).Mul(s, s)), den)
		cy := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), s), den)
		radials = append(radials, [2]*big.Rat{new(big.Rat).Mul(rho, cx), new(big.Rat).Mul(rho, cy)})
	}
	radials = append(radials, [2]*big.Rat{new(big.Rat).Neg(rho), new(big.Rat)})
	for _, q := range radials {
		for _, z := range []*big.Rat{new(big.Rat), rat(tc.level)} {
			local := [3]*big.Rat{q[0], q[1], z}
			var p, rel [3]*big.Rat
			for i := range p {
				p[i] = new(big.Rat).Set(center[i])
				for k := range local {
					p[i].Add(p[i], new(big.Rat).Mul(l[i][k], local[k]))
				}
				rel[i] = new(big.Rat).Sub(p[i], tagOrigin[i])
			}
			share := new(big.Rat).Quo(dot(rel, tagAxis), dot(tagAxis, tagAxis))
			var tagRadial [3]*big.Rat
			for i := range tagRadial {
				tagRadial[i] = new(big.Rat).Sub(rel[i], new(big.Rat).Mul(tagAxis[i], share))
			}
			normal := solveTransposed(t, l, [3]*big.Rat{q[0], q[1], new(big.Rat)})
			gap := unitDistance(tagRadial, normal)
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
	}
	out, _ := worst.Float64()
	return out
}

func rat(f float64) *big.Rat {
	r := proofarith.FloatRat(f)
	if r == nil {
		panic("non-finite fixture value")
	}
	return r
}

func ratVec(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{rat(v.X), rat(v.Y), rat(v.Z)} }

func vecComponent(v r3.Vec, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	}
	return v.Z
}

func dot(a, b [3]*big.Rat) *big.Rat {
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}

// solveTransposed returns x with Lᵀx = b by Cramer's rule.
func solveTransposed(t *testing.T, l [3][3]*big.Rat, b [3]*big.Rat) [3]*big.Rat {
	t.Helper()
	var m [3][3]*big.Rat
	for i := range m {
		for k := range m[i] {
			m[i][k] = l[k][i]
		}
	}
	det := det3(m)
	require.NotZero(t, det.Sign())
	var out [3]*big.Rat
	for c := range out {
		replaced := m
		for i := range replaced {
			row := replaced[i]
			row[c] = b[i]
			replaced[i] = row
		}
		out[c] = new(big.Rat).Quo(det3(replaced), det)
	}
	return out
}

func det3(m [3][3]*big.Rat) *big.Rat {
	minor := func(a, b, c, d *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a, d), new(big.Rat).Mul(b, c))
	}
	sum := new(big.Rat).Mul(m[0][0], minor(m[1][1], m[1][2], m[2][1], m[2][2]))
	sum.Sub(sum, new(big.Rat).Mul(m[0][1], minor(m[1][0], m[1][2], m[2][0], m[2][2])))
	return sum.Add(sum, new(big.Rat).Mul(m[0][2], minor(m[1][0], m[1][1], m[2][0], m[2][1])))
}

// unitDistance is |a/|a| − b/|b|| in 512-bit floats.
func unitDistance(a, b [3]*big.Rat) *big.Float {
	unit := func(v [3]*big.Rat) [3]*big.Float {
		norm := new(big.Float).SetPrec(512).SetRat(dot(v, v))
		norm.Sqrt(norm)
		var out [3]*big.Float
		for i := range out {
			out[i] = new(big.Float).SetPrec(512).SetRat(v[i])
			out[i].Quo(out[i], norm)
		}
		return out
	}
	ua, ub := unit(a), unit(b)
	sum := new(big.Float).SetPrec(512)
	for i := range ua {
		d := new(big.Float).SetPrec(512).Sub(ua[i], ub[i])
		sum.Add(sum, new(big.Float).SetPrec(512).Mul(d, d))
	}
	return sum.Sqrt(sum)
}
