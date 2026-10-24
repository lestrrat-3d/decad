package revolveproof

// These tests read the unexported general-axis brackets directly; the
// exported routes reach them only through a full Sketch tooth whose root cone
// rounds neither its apex nor its slope.

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// roundedSection is a general certificate about A = (1, 3, 0), of length √10,
// whose root cone has slope k* = 1/3 and apex scalar Z_a = 6. Neither rounds
// exactly: the reported apex (float(0.6), float(1.8), 0) lies off the axis line,
// because the two coordinates round by unrelated amounts, and the reported
// slope is float(1/3).
func roundedSection(t *testing.T) facetedrecord.RevolveSectionCertificate {
	t.Helper()
	axis, ok := coneaxis.New(r3.Vec{X: 1, Y: 3})
	require.True(t, ok)
	slope, apex := big.NewRat(1, 3), big.NewRat(6, 1)
	s, _ := slope.Float64()
	x, _ := big.NewRat(6, 10).Float64()
	y, _ := big.NewRat(18, 10).Float64()
	cone := sketch.ConeSection{Apex: r3.Vec{X: x, Y: y}, Axis: axis.Vec(), Slope: s}
	return facetedrecord.RevolveSectionCertificate{Axis: axis,
		General: &facetedrecord.RevolveSectionAxis{Cone: cone, Slope: slope, Apex: apex}}
}

// scaledPoint is the world point with A-scaled axial Z and radial R in the
// XY plane: (Z·A + R·N)/10 with N = (−3, 1, 0).
func scaledPoint(z, r int64) [3]*big.Rat {
	return [3]*big.Rat{big.NewRat(z-3*r, 10), big.NewRat(3*z+r, 10), new(big.Rat)}
}

// exactForms evaluates the reported cone's F and the true root cone's
// |p×A|² − (k*·(p·A − Z_a))² at p, written out from the certificate's data.
func exactForms(section facetedrecord.RevolveSectionCertificate, p [3]*big.Rat) (*big.Rat, *big.Rat) {
	general := section.General
	a := section.Axis.Vec()
	av := [3]*big.Rat{proofarith.FloatRat(a.X), proofarith.FloatRat(a.Y), proofarith.FloatRat(a.Z)}
	o := general.Cone.Apex
	w := [3]*big.Rat{new(big.Rat).Sub(p[0], proofarith.FloatRat(o.X)),
		new(big.Rat).Sub(p[1], proofarith.FloatRat(o.Y)), new(big.Rat).Sub(p[2], proofarith.FloatRat(o.Z))}
	norm2 := func(v [3]*big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Add(new(big.Rat).Mul(v[0], v[0]), new(big.Rat).Mul(v[1], v[1])),
			new(big.Rat).Mul(v[2], v[2]))
	}
	cross := func(v [3]*big.Rat) [3]*big.Rat {
		c := func(i, j int) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(v[i], av[j]), new(big.Rat).Mul(v[j], av[i]))
		}
		return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
	}
	dot := func(v [3]*big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Add(new(big.Rat).Mul(v[0], av[0]), new(big.Rat).Mul(v[1], av[1])),
			new(big.Rat).Mul(v[2], av[2]))
	}
	s := proofarith.FloatRat(general.Cone.Slope)
	h := dot(w)
	coneForm := new(big.Rat).Sub(norm2(cross(w)), new(big.Rat).Mul(new(big.Rat).Mul(s, s), new(big.Rat).Mul(h, h)))
	radius := new(big.Rat).Mul(general.Slope, new(big.Rat).Sub(dot(p), general.Apex))
	sourceForm := new(big.Rat).Sub(norm2(cross(p)), new(big.Rat).Mul(radius, radius))
	return coneForm, sourceForm
}

// The connector from (Z, R) = (21, 2) to (21, 8) crosses the true root cone
// at R = k*·(21 − 6) = 5, t = 1/2 exactly. The reported cone's crossing,
// solved here in 512-bit arithmetic, lies within the proven shift of it.
func TestSeatShiftAboutCoversTrueContact(t *testing.T) {
	section := roundedSection(t)
	q, endQ := scaledPoint(21, 2), scaledPoint(21, 8)
	var d [3]*big.Rat
	for i := range d {
		d[i] = new(big.Rat).Sub(endQ[i], q[i])
	}
	shift, ok := seatShiftAbout(section, q, endQ, d)
	require.True(t, ok)
	require.Less(t, shift, 1e-9)
	// F(t) along the line is quadratic: read it at t = 0, 1/2 and 1.
	at := func(tt *big.Rat) *big.Rat {
		var p [3]*big.Rat
		for i := range p {
			p[i] = new(big.Rat).Add(q[i], new(big.Rat).Mul(tt, d[i]))
		}
		f, _ := exactForms(section, p)
		return f
	}
	_, sourceHalf := exactForms(section, scaledPoint(21, 5))
	require.Zero(t, sourceHalf.Sign(), "the true contact is t = 1/2")
	f0, fh, f1 := at(new(big.Rat)), at(big.NewRat(1, 2)), at(big.NewRat(1, 1))
	a := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Add(f0, f1)),
		new(big.Rat).Mul(big.NewRat(4, 1), fh))
	b := new(big.Rat).Sub(new(big.Rat).Sub(f1, f0), a)
	const prec = 512
	bf := func(r *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(r) }
	disc := new(big.Float).SetPrec(prec).Sub(new(big.Float).Mul(bf(b), bf(b)),
		new(big.Float).Mul(big.NewFloat(4), new(big.Float).Mul(bf(a), bf(f0))))
	require.Positive(t, disc.Sign())
	disc.Sqrt(disc)
	var root *big.Float
	for _, sign := range []float64{-1, 1} {
		r := new(big.Float).SetPrec(prec).Add(new(big.Float).Neg(bf(b)), new(big.Float).Mul(big.NewFloat(sign), disc))
		r.Quo(r, new(big.Float).Mul(big.NewFloat(2), bf(a)))
		if r.Sign() > 0 && r.Cmp(big.NewFloat(1)) < 0 {
			require.Nil(t, root, "one crossing in (0, 1)")
			root = r
		}
	}
	require.NotNil(t, root)
	gap := new(big.Float).SetPrec(prec).Sub(root, big.NewFloat(0.5))
	gap.Abs(gap)
	require.Positive(t, gap.Sign(), "the reported and the true crossing differ")
	require.LessOrEqual(t, gap.Cmp(big.NewFloat(shift)), 0, "shift %g, gap %s", shift, gap.Text('g', 6))
}

// Over a single-point box the interval arithmetic is exact, so the fitted
// bracket must return exactly |F_blank − F_cone| and its exact directional
// derivative, including the part the off-axis reported apex contributes.
func TestFitMeridianDifferenceAboutIsExactOnPoints(t *testing.T) {
	section := roundedSection(t)
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	difference := func(p [3]*big.Rat) *big.Rat {
		cone, source := exactForms(section, p)
		return new(big.Rat).Sub(source, cone)
	}
	for _, zr := range [][2]int64{{13, 1}, {21, 7}, {36, 2}, {19, 3}} {
		p := scaledPoint(zr[0], zr[1])
		u, _ := p[0].Float64()
		v, _ := p[1].Float64()
		require.Zero(t, proofarith.FloatRat(u).Cmp(p[0]))
		require.Zero(t, proofarith.FloatRat(v).Cmp(p[1]))
		for _, tangent := range [][2]float64{{1, 0}, {0, 1}} {
			diff, deriv, _, ok := fitMeridianDifferenceAbout(section, section.ConeSection(), frame,
				[2]float64{u, v}, [2]float64{u, v}, tangent, tangent)
			require.True(t, ok)
			require.Zero(t, new(big.Rat).Abs(difference(p)).Cmp(diff), "difference at %v", zr)
			// The difference is quadratic, so its central difference is its
			// exact derivative.
			var plus, minus [3]*big.Rat
			for i := range p {
				step := new(big.Rat)
				if i < 2 {
					step = proofarith.FloatRat(tangent[i])
				}
				plus[i] = new(big.Rat).Add(p[i], step)
				minus[i] = new(big.Rat).Sub(p[i], step)
			}
			want := new(big.Rat).Quo(new(big.Rat).Sub(difference(plus), difference(minus)), big.NewRat(2, 1))
			require.Zero(t, want.Abs(want).Cmp(deriv), "derivative at %v along %v", zr, tangent)
		}
	}
}
