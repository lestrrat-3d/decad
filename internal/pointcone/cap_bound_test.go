package pointcone

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// triangleBoundReference is TriangleBound with every denominator built as a
// Dyadic power of G before one DyQuoFloatUp, the form TriangleBound's
// chained and fused quotients must match bit for bit.
func triangleBoundReference(geom CellGeom, c *Consts) (float64, float64, float64, error) {
	if c == nil {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone axis has no finite norm bracket", decaderr.ErrUnsupported)
	}
	xMin, rhoMin, reach, h := geom.xMin, geom.rhoMin, geom.reach, geom.h
	if rhoMin <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap reaches the axis", decaderr.ErrUnsupported)
	}
	x, rho, hd := proofarith.MustDyOf(xMin), proofarith.MustDyOf(rhoMin), proofarith.MustDyOf(h)
	g := proofarith.DyAdd(proofarith.DyMul(x, c.kd), proofarith.DyMul(c.kn, rho))
	if g.Sign() <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap has no positive ray denominator", decaderr.ErrUnsupported)
	}
	h2 := proofarith.DyMul(hd, hd)
	g2 := proofarith.DyMul(g, g)
	g3 := proofarith.DyMul(g2, g)
	adG2 := proofarith.DyMul(c.ad, g2)
	alphaBound := proofbound.AbsSumUpper(
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cRec, h2), proofarith.DyMul(c.ad, g3)),
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cRad, h2),
			proofarith.DyMul(proofarith.DyShift(rho, 1), adG2)))
	capBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(reach, alphaBound),
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cProd, h2), adG2))
	alphaMax := proofarith.DyQuoFloatUp(c.cAlpha, proofarith.DyMul(c.ad, g))
	lip := proofbound.AbsSumUpper(alphaMax,
		proofbound.ProductUpper(reach, proofarith.DyQuoFloatUp(c.cProd, adG2)))
	return capBound, alphaBound, lip, nil
}

// TriangleBound is bit-identical to triangleBoundReference, errors included,
// over subdivided and random caps about the canonical and a general axis,
// cells scaled across many binades, cells that reach the axis or lie behind
// the apex, and cones with small, wide, non-dyadic and negative constants.
func TestTriangleBoundMatchesReference(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(41, 43))
	var geoms []CellGeom
	for _, axis := range []coneaxis.Axis{coneaxis.X(), mustAxis(t, r3.Vec{X: 1, Y: 2})} {
		far, caps := capTestFan(t, 3)
		geoms = append(geoms, capGeometry(far, caps, axis, 1)...)
		far, caps = capTestRandom(r, 300)
		geoms = append(geoms, capGeometry(far, caps, axis, 1)...)
	}
	cells := 1500
	if raceDetector {
		cells = 300
	}
	for range cells {
		scale := math.Ldexp(1, r.IntN(40)-20)
		base := r3.Vec{X: (r.Float64()*2 - 1) * 10 * scale, Y: (r.Float64()*2 - 1) * 10 * scale,
			Z: (r.Float64()*2 - 1) * 10 * scale}
		var tri [3]r3.Vec
		for j := range tri {
			tri[j] = base.Add(r3.Vec{X: r.Float64() * scale, Y: r.Float64() * scale, Z: r.Float64() * scale})
		}
		geoms = append(geoms, CellGeometry(tri, coneaxis.X()))
	}
	geoms = append(geoms, CellGeom{xMin: 1, rhoMin: 0, reach: 1, h: 1},
		CellGeom{xMin: -1e6, rhoMin: 1, reach: 1, h: 1})

	pairs := pointConeTestCones(r,
		[2]*big.Rat{big.NewRat(9, 1), big.NewRat(1, 2)},
		[2]*big.Rat{big.NewRat(1, 1000), big.NewRat(1000, 1)},
		[2]*big.Rat{big.NewRat(-7, 3), big.NewRat(5, 11)},
		[2]*big.Rat{proofarith.FloatRat(math.Pi), proofarith.FloatRat(math.E)})
	var cones []*Consts
	for _, p := range pairs {
		cones = append(cones, xConsts(p[0], p[1]))
		_, bound := ConstantsOf(mustAxis(t, r3.Vec{X: 1, Y: 2}), p[0], p[1])
		cones = append(cones, bound)
	}
	cones = append(cones, nil)

	errs := 0
	for ci, c := range cones {
		for k, g := range geoms {
			wantCap, wantAlpha, wantLip, wantErr := triangleBoundReference(g, c)
			gotCap, gotAlpha, gotLip, gotErr := TriangleBound(g, c)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error(), "cone %d cell %d", ci, k)
				errs++
				continue
			}
			require.NoError(t, gotErr, "cone %d cell %d", ci, k)
			require.Equal(t, math.Float64bits(wantCap), math.Float64bits(gotCap), "cone %d cell %d cap", ci, k)
			require.Equal(t, math.Float64bits(wantAlpha), math.Float64bits(gotAlpha), "cone %d cell %d alpha", ci, k)
			require.Equal(t, math.Float64bits(wantLip), math.Float64bits(gotLip), "cone %d cell %d lip", ci, k)
		}
	}
	require.Positive(t, errs)
}
