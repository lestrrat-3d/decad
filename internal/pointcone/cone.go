// Package pointcone holds the point-loft cone crossing map: exact cone
// constants, held crossing samples, far-cap subdivision and contact split, and
// cap and triangle crossing bounds.
//
// Every cone is about an axis A through the world origin
// (agent-docs/revolve-axis-design.md §3). Its apex and far plane are A-scaled
// scalars c = P·A, and its slope k is homogeneous. For the canonical axis
// A = +X these are the millimetre X coordinates, and every function keeps its
// +X body unchanged behind coneaxis.Axis.Canonical. For any other axis,
// crossings, orderings and signs use the exact A-scaled values s(p) = p·A and
// σ(p) = |p×A|, and bound formulas convert to millimetres on the side that
// widens them (§3.4).
package pointcone

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
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

// ConstantsOf builds the two constant sets of the cone about axis with apex
// scalar apex and slope (agent-docs/revolve-axis-design.md §3.4). The first is
// the sample set, built from the A-scaled apex c, which FactorDyadic reads. The
// second is the bound set, which TriangleBound reads against millimetre cell
// geometry. It is built from a↑, a float no smaller than c/|A|. For a canonical
// axis both are the one set built from apex. For a general axis the bound set
// is nil when |A| has no finite bracket, and TriangleBound then refuses.
// Neither rational may be nil.
func ConstantsOf(axis coneaxis.Axis, apex, slope *big.Rat) (*Consts, *Consts) {
	sample := constantsOf(apex, slope)
	if axis.Canonical() {
		return sample, sample
	}
	aUp, ok := axis.ScaledUp(apex)
	if !ok {
		return sample, nil
	}
	return sample, constantsOf(proofarith.FloatRat(aUp), slope)
}

// constantsOf builds the exact constants of the cone with the given apex and
// slope.
func constantsOf(apex, slope *big.Rat) *Consts {
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
// lies above the exact far plane s(p) = far about axis. For a general axis the
// millimetre widening moves s by at most meshBound·|A|↑ (§5.4).
func AboveFarCap(verts []r3.Vec, meshBound float64, far *big.Rat, axis coneaxis.Axis) bool {
	if !proofbound.FiniteVec(r3.Vec{X: meshBound}) || meshBound < 0 {
		return false
	}
	if !axis.Canonical() {
		return aboveFarCapAxis(verts, meshBound, far, axis)
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

func aboveFarCapAxis(verts []r3.Vec, meshBound float64, far *big.Rat, axis coneaxis.Axis) bool {
	widen, ok := axisWidening(meshBound, axis)
	if !ok {
		return false
	}
	for _, p := range verts {
		if !proofbound.FiniteVec(p) {
			return false
		}
		lo := new(big.Rat).Sub(axis.Axial(p).Rat(), widen)
		if lo.Cmp(far) <= 0 {
			return false
		}
	}
	return true
}

// axisWidening is meshBound·|A|↑, the most a millimetre displacement of
// meshBound moves the A-scaled axial or radial reading of a point.
func axisWidening(meshBound float64, axis coneaxis.Axis) (*big.Rat, bool) {
	lo, hi := axis.NormBracket()
	if !(lo > 0) || proofbound.IsNonFinite(hi) {
		return nil, false
	}
	return new(big.Rat).Mul(proofarith.FloatRat(meshBound), proofarith.FloatRat(hi)), true
}

// The larger cone's radial crossing lies farther from the point-loft apex.
// Ordered uses a deliberately wide exact-rational enclosure of the source fan
// (verts, widened by meshBound), so nearly coincident or crossing cones fall
// back to mesh Boolean. It reports whether the later cone's crossing lies
// strictly beyond the earlier cone's over the whole far cap. Both cones are
// about axis.
func Ordered(earlierApex, earlierSlope, laterApex, laterSlope *big.Rat,
	verts []r3.Vec, meshBound float64, axis coneaxis.Axis) bool {
	if !axis.Canonical() {
		return orderedAxis(earlierApex, earlierSlope, laterApex, laterSlope, verts, meshBound, axis)
	}
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

// orderedAxis is Ordered about a general axis. The margin dA·s + dR·σ is
// exact in A-scaled readings, the radial minimum comes from the exact
// orthogonal pair of §3.5, and the mesh-bound move takes meshBound·|A|↑
// (§5.4).
func orderedAxis(earlierApex, earlierSlope, laterApex, laterSlope *big.Rat,
	verts []r3.Vec, meshBound float64, axis coneaxis.Axis) bool {
	if len(verts) < 2 || proofbound.IsNonFinite(meshBound) || meshBound < 0 {
		return false
	}
	widen, ok := axisWidening(meshBound, axis)
	if !ok {
		return false
	}
	var sMin, sMax *big.Rat
	rhoMax := new(big.Rat)
	for _, p := range verts[1:] {
		if !proofbound.FiniteVec(p) {
			return false
		}
		s := axis.Axial(p).Rat()
		if sMin == nil || s.Cmp(sMin) < 0 {
			sMin = s
		}
		if sMax == nil || s.Cmp(sMax) > 0 {
			sMax = s
		}
		r := proofbound.RatSqrtUp(axis.RadialSq(p).Rat())
		if proofbound.IsNonFinite(r) {
			return false
		}
		if rf := proofarith.FloatRat(r); rf.Cmp(rhoMax) > 0 {
			rhoMax = rf
		}
	}
	rhoLo, ok := axis.RadialLowerOf(verts[1:])
	if !ok {
		return false
	}
	rhoMin := proofarith.FloatRat(rhoLo)
	dA := new(big.Rat).Sub(laterApex, earlierApex)
	dR := new(big.Rat).Sub(new(big.Rat).Mul(laterApex, earlierSlope),
		new(big.Rat).Mul(earlierApex, laterSlope))
	s := sMin
	if dA.Sign() < 0 {
		s = sMax
	}
	rho := rhoMin
	if dR.Sign() < 0 {
		rho = rhoMax
	}
	margin := new(big.Rat).Add(new(big.Rat).Mul(dA, s), new(big.Rat).Mul(dR, rho))
	move := new(big.Rat).Add(new(big.Rat).Abs(dA), new(big.Rat).Abs(dR))
	move.Mul(move, widen)
	return margin.Cmp(move) > 0
}

func ratF(v float64) *big.Rat       { return proofarith.FloatRat(v) }
func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

// ToeHeelOrdered reports whether the heel cone's radius exceeds the toe cone's
// over the whole of verts widened by meshBound. Both slopes must be positive
// and the toe apex strictly below the heel apex. Both cones are about axis;
// for a general axis the axial interval is the exact range of s(p), widened by
// meshBound·|A|↑ (§5.4).
func ToeHeelOrdered(toeApex, toeSlope, heelApex, heelSlope *big.Rat,
	verts []r3.Vec, meshBound float64, axis coneaxis.Axis) bool {
	if len(verts) == 0 || proofbound.IsNonFinite(meshBound) || meshBound < 0 ||
		toeApex.Cmp(heelApex) >= 0 || toeSlope.Sign() <= 0 || heelSlope.Sign() <= 0 {
		return false
	}
	var lo, hi *big.Rat
	if axis.Canonical() {
		minX, maxX := verts[0].X, verts[0].X
		for _, v := range verts {
			if !proofbound.FiniteVec(v) {
				return false
			}
			minX, maxX = min(minX, v.X), max(maxX, v.X)
		}
		lo = ratSub(ratF(minX), ratF(meshBound))
		hi = ratAdd(ratF(maxX), ratF(meshBound))
	} else {
		var ok bool
		if lo, hi, ok = axialRange(verts, meshBound, axis); !ok {
			return false
		}
	}
	// The heel radius exceeds the toe radius when this expression is
	// positive. It is affine in X, so both interval endpoints suffice.
	margin := func(x *big.Rat) *big.Rat {
		return ratSub(ratMul(toeSlope, ratSub(heelApex, x)),
			ratMul(heelSlope, ratSub(toeApex, x)))
	}
	return margin(lo).Sign() > 0 && margin(hi).Sign() > 0
}

// axialRange is the exact range of s(p) over verts, widened on each side by
// meshBound·|A|↑.
func axialRange(verts []r3.Vec, meshBound float64, axis coneaxis.Axis) (*big.Rat, *big.Rat, bool) {
	widen, ok := axisWidening(meshBound, axis)
	if !ok {
		return nil, nil, false
	}
	var lo, hi *big.Rat
	for _, v := range verts {
		if !proofbound.FiniteVec(v) {
			return nil, nil, false
		}
		s := axis.Axial(v).Rat()
		if lo == nil || s.Cmp(lo) < 0 {
			lo = s
		}
		if hi == nil || s.Cmp(hi) > 0 {
			hi = s
		}
	}
	return ratSub(lo, widen), ratAdd(hi, widen), true
}
