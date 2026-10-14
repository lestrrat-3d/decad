// Package pointcone holds the point-loft cone crossing map: exact cone
// constants, held crossing samples, far-cap subdivision and contact split, and
// cap and triangle crossing bounds.
package pointcone

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Consts are the cone's apex a = an/ad and slope k = kn/kd as exact
// integers, with the products every triangle bound multiplies in. Every value
// is an integer, so a Dyadic with exponent zero.
type Consts struct {
	ad, kd            proofarith.Dyadic
	cRec, cRad, cProd proofarith.Dyadic // an(kd+kn)²kd, an·kn·kd, an(kd+kn)kd
	cAlpha            proofarith.Dyadic // an·kd
	kn                proofarith.Dyadic
	apexF, slopeF     float64 // apex.Float64(), slope.Float64()
}

// ConstantsOf builds the exact constants of the cone with the given apex and
// slope. Neither argument may be nil.
func ConstantsOf(apex, slope *big.Rat) *Consts {
	dyInt := func(z *big.Int) proofarith.Dyadic {
		d, _ := proofarith.DyOfRat(new(big.Rat).SetInt(z))
		return d
	}
	an, ad := dyInt(new(big.Int).Set(apex.Num())), dyInt(new(big.Int).Set(apex.Denom()))
	kn, kd := dyInt(new(big.Int).Set(slope.Num())), dyInt(new(big.Int).Set(slope.Denom()))
	sum := proofarith.DyAdd(kd, kn)
	ankd := proofarith.DyMul(an, kd)
	apexF, _ := apex.Float64()
	slopeF, _ := slope.Float64()
	return &Consts{
		ad: ad, kd: kd, kn: kn, apexF: apexF, slopeF: slopeF,
		cRec:   proofarith.DyMul(proofarith.DyMul(ankd, sum), sum),
		cRad:   proofarith.DyMul(ankd, kn),
		cProd:  proofarith.DyMul(ankd, sum),
		cAlpha: ankd,
	}
}

// Edge is an unordered pair of far-cap vertex indices; build one with
// OrderedEdge.
type Edge struct{ A, B int }

// Side is a far-cap boundary edge from vertex U to vertex V with the facet
// group of the side wall it lies on.
type Side struct{ U, V, Group int }

// Sample is a held far-cap crossing: the cone parameter Q, its reciprocal
// Alpha, and the error bounds of Q and of the scaled point.
type Sample struct {
	Q, Alpha           float64
	QError, PointError float64
}

// OrderedEdge returns the edge between a and b with the smaller index first.
func OrderedEdge(a, b int) Edge {
	if a > b {
		a, b = b, a
	}
	return Edge{a, b}
}

// The cone's far plane is not represented in the radial trim mesh. Admit
// this arm only when the entire authenticated fan clears that plane.
// AboveFarCap reports whether every vertex of verts, widened by meshBound,
// lies above the exact far plane x = far.
func AboveFarCap(verts []r3.Vec, meshBound float64, far *big.Rat) bool {
	if !proofbound.FiniteVec(r3.Vec{X: meshBound}) || meshBound < 0 {
		return false
	}
	for _, p := range verts {
		if !proofbound.FiniteVec(p) {
			return false
		}
		lo := new(big.Rat).Sub(proofarith.FloatRat(p.X), proofarith.FloatRat(meshBound))
		if lo.Cmp(far) <= 0 {
			return false
		}
	}
	return true
}

// The larger cone's radial crossing lies farther from the point-loft apex.
// Ordered uses a deliberately wide exact-rational enclosure of the source fan
// (verts, widened by meshBound), so nearly coincident or crossing cones fall
// back to mesh Boolean. It reports whether the later cone's crossing lies
// strictly beyond the earlier cone's over the whole far cap.
func Ordered(earlierApex, earlierSlope, laterApex, laterSlope *big.Rat,
	verts []r3.Vec, meshBound float64) bool {
	xMin, xMax := math.Inf(1), math.Inf(-1)
	yMin, yMax := math.Inf(1), math.Inf(-1)
	zMin, zMax := math.Inf(1), math.Inf(-1)
	rhoMax := new(big.Rat)
	for _, p := range verts[1:] {
		xMin, xMax = min(xMin, p.X), max(xMax, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		y, z := proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)
		r2 := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
		r := proofarith.FloatRat(proofbound.RatSqrtUp(r2))
		if r.Cmp(rhoMax) > 0 {
			rhoMax = r
		}
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	rhoMin := proofarith.FloatRat(max(minAbs(yMin, yMax), minAbs(zMin, zMax)))
	// aLater/gLater > aEarlier/gEarlier exactly when this affine
	// expression in x and rho is positive over the whole far cap.
	dA := new(big.Rat).Sub(laterApex, earlierApex)
	dR := new(big.Rat).Sub(new(big.Rat).Mul(laterApex, earlierSlope),
		new(big.Rat).Mul(earlierApex, laterSlope))
	x := proofarith.FloatRat(xMin)
	if dA.Sign() < 0 {
		x = proofarith.FloatRat(xMax)
	}
	rho := rhoMin
	if dR.Sign() < 0 {
		rho = rhoMax
	}
	margin := new(big.Rat).Add(new(big.Rat).Mul(dA, x), new(big.Rat).Mul(dR, rho))
	move := new(big.Rat).Add(new(big.Rat).Abs(dA), new(big.Rat).Abs(dR))
	move.Mul(move, proofarith.FloatRat(meshBound))
	return margin.Cmp(move) > 0
}

func ratF(v float64) *big.Rat       { return proofarith.FloatRat(v) }
func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

// ToeHeelOrdered reports whether the heel cone's radius exceeds the toe cone's
// over the whole of verts widened by meshBound. Both slopes must be positive
// and the toe apex strictly below the heel apex.
func ToeHeelOrdered(toeApex, toeSlope, heelApex, heelSlope *big.Rat,
	verts []r3.Vec, meshBound float64) bool {
	if len(verts) == 0 || proofbound.IsNonFinite(meshBound) || meshBound < 0 ||
		toeApex.Cmp(heelApex) >= 0 || toeSlope.Sign() <= 0 || heelSlope.Sign() <= 0 {
		return false
	}
	minX, maxX := verts[0].X, verts[0].X
	for _, v := range verts {
		if !proofbound.FiniteVec(v) {
			return false
		}
		minX, maxX = min(minX, v.X), max(maxX, v.X)
	}
	lo := ratSub(ratF(minX), ratF(meshBound))
	hi := ratAdd(ratF(maxX), ratF(meshBound))
	// The heel radius exceeds the toe radius when this expression is
	// positive. It is affine in X, so both interval endpoints suffice.
	margin := func(x *big.Rat) *big.Rat {
		return ratSub(ratMul(toeSlope, ratSub(heelApex, x)),
			ratMul(heelSlope, ratSub(toeApex, x)))
	}
	return margin(lo).Sign() > 0 && margin(hi).Sign() > 0
}
