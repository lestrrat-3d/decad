package decad

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueRectangleClip intersects the exact held rectangular prism with the
// exact affine plane spanned by the requested held U and V vectors. It returns
// a rounded plane-local polygon and a displacement covering every matched
// point of its exact source boundary in both local and world coordinates.
func obliqueRectangleClip(pp prismPayload, plane r3.Frame) ([]Point2, float64, bool) {
	u0, u1, v0, v1, ok := momentinput.RectangleBounds(pp.Profile)
	if !ok || pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	base := proofarith.DyVec(pp.Frame.Origin())
	axis := [3]proofarith.DyV3{
		proofarith.DyVec(pp.Frame.U()), proofarith.DyVec(pp.Frame.V()), proofarith.DyVec(pp.Frame.N()),
	}
	limits := [3][2]float64{{u0, u1}, {v0, v1}, {pp.Z0, pp.Z1}}
	requestedOrigin := proofarith.DyVec(plane.Origin())
	normal := proofarith.DvCross(proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()))
	if proofarith.DvIsZero(normal) {
		return nil, 0, false
	}
	var corners [8]proofarith.DyV3
	var side [8]proofarith.Dyadic
	for corner := range corners {
		point := base
		for coordinate := range 3 {
			choice := (corner >> coordinate) & 1
			point = proofarith.DvAdd(point, proofarith.DvScale(axis[coordinate],
				proofarith.DyOfFinite(limits[coordinate][choice])))
		}
		corners[corner] = proofarith.DvTransform(pp.Xform, point)
		side[corner] = proofarith.DvDot(proofarith.DvSub(corners[corner], requestedOrigin), normal)
		// Contact at a box vertex needs a separate topology proof.
		if side[corner].IsZero() {
			return nil, 0, false
		}
	}
	var exact []sectionRatPoint
	for a := range corners {
		for coordinate := range 3 {
			b := a ^ (1 << coordinate)
			if b < a || side[a].Sign() == side[b].Sign() {
				continue
			}
			t := sectionRatQuo(side[a].Rat(), proofarith.DySubScalar(side[a], side[b]).Rat())
			var world [3]*big.Rat
			for component := range 3 {
				world[component] = sectionRatAdd(corners[a][component].Rat(),
					sectionRatMul(t, sectionRatSub(corners[b][component].Rat(), corners[a][component].Rat())))
			}
			point, ok := sectionPlaneCoordinates(world, plane)
			if !ok {
				return nil, 0, false
			}
			duplicate := false
			for _, other := range exact {
				if point.u.Cmp(other.u) == 0 && point.v.Cmp(other.v) == 0 {
					duplicate = true
					break
				}
			}
			if !duplicate {
				exact = append(exact, point)
			}
		}
	}
	if len(exact) < 3 || len(exact) > 6 {
		return nil, 0, false
	}
	sectionSortRatPolygon(exact)
	proxy := make([]Point2, len(exact))
	delta := float64(0)
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	for i, point := range exact {
		u, _ := point.u.Float64()
		v, _ := point.v.Float64()
		if math.IsNaN(u) || math.IsInf(u, 0) || math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, 0, false
		}
		proxy[i] = Point2{U: u, V: v}
		uError := proofarith.RationalFloatError(point.u, u)
		vError := proofarith.RationalFloatError(point.v, v)
		local := proofbound.AbsSumUpper(uError, vError)
		world := proofbound.AbsSumUpper(proofbound.ProductUpper(uError, uNorm),
			proofbound.ProductUpper(vError, vNorm))
		delta = max(delta, local, world)
	}
	if !sectionMatchedConvex(exact, proxy) || math.IsInf(delta, 0) || math.IsNaN(delta) {
		return nil, 0, false
	}
	delta = proofbound.UpRound(delta)
	// A loop whose coordinate error is comparable to a source edge has no
	// useful local feature scale for later reconstruction and verification.
	for i, point := range exact {
		next := exact[(i+1)%len(exact)]
		uSpan := new(big.Rat).Abs(sectionRatSub(next.u, point.u))
		vSpan := new(big.Rat).Abs(sectionRatSub(next.v, point.v))
		edgeL1 := sectionRatAdd(uSpan, vSpan)
		if edgeL1.Sign() <= 0 || sectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}

type sectionRatPoint struct {
	u, v *big.Rat
}

func sectionRatAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func sectionRatSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func sectionRatMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
func sectionRatQuo(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }

func sectionPlaneCoordinates(world [3]*big.Rat, plane r3.Frame) (sectionRatPoint, bool) {
	origin := proofarith.DyVec(plane.Origin())
	uAxis := proofarith.DyVec(plane.U())
	vAxis := proofarith.DyVec(plane.V())
	for a := range 3 {
		for b := a + 1; b < 3; b++ {
			det := sectionRatSub(sectionRatMul(uAxis[a].Rat(), vAxis[b].Rat()),
				sectionRatMul(uAxis[b].Rat(), vAxis[a].Rat()))
			if det.Sign() == 0 {
				continue
			}
			da := sectionRatSub(world[a], origin[a].Rat())
			db := sectionRatSub(world[b], origin[b].Rat())
			u := sectionRatQuo(sectionRatSub(sectionRatMul(da, vAxis[b].Rat()),
				sectionRatMul(db, vAxis[a].Rat())), det)
			v := sectionRatQuo(sectionRatSub(sectionRatMul(uAxis[a].Rat(), db),
				sectionRatMul(uAxis[b].Rat(), da)), det)
			for component := range 3 {
				lifted := sectionRatAdd(origin[component].Rat(),
					sectionRatAdd(sectionRatMul(u, uAxis[component].Rat()), sectionRatMul(v, vAxis[component].Rat())))
				if lifted.Cmp(world[component]) != 0 {
					return sectionRatPoint{}, false
				}
			}
			return sectionRatPoint{u: u, v: v}, true
		}
	}
	return sectionRatPoint{}, false
}

func sectionRatCross(a, b sectionRatPoint) *big.Rat {
	return sectionRatSub(sectionRatMul(a.u, b.v), sectionRatMul(a.v, b.u))
}

func sectionRatVector(a, b sectionRatPoint) sectionRatPoint {
	return sectionRatPoint{sectionRatSub(b.u, a.u), sectionRatSub(b.v, a.v)}
}

func sectionSortRatPolygon(points []sectionRatPoint) {
	center := sectionRatPoint{new(big.Rat), new(big.Rat)}
	for _, point := range points {
		center.u.Add(center.u, point.u)
		center.v.Add(center.v, point.v)
	}
	count := new(big.Rat).SetInt64(int64(len(points)))
	center.u.Quo(center.u, count)
	center.v.Quo(center.v, count)
	half := func(p sectionRatPoint) int {
		if p.v.Sign() > 0 || (p.v.Sign() == 0 && p.u.Sign() >= 0) {
			return 0
		}
		return 1
	}
	sort.Slice(points, func(i, j int) bool {
		a, b := sectionRatVector(center, points[i]), sectionRatVector(center, points[j])
		if half(a) != half(b) {
			return half(a) < half(b)
		}
		return sectionRatCross(a, b).Sign() > 0
	})
}

func sectionMatchedConvex(source []sectionRatPoint, rounded []Point2) bool {
	if len(source) != len(rounded) {
		return false
	}
	proxy := make([]sectionRatPoint, len(rounded))
	for i, point := range rounded {
		proxy[i] = sectionRatPoint{proofarith.FloatRat(point.U), proofarith.FloatRat(point.V)}
	}
	for i := range source {
		next := (i + 1) % len(source)
		a0, a1 := sectionRatVector(source[i], source[next]), sectionRatVector(proxy[i], proxy[next])
		for j := range source {
			if j == i || j == next {
				continue
			}
			b0, b1 := sectionRatVector(source[i], source[j]), sectionRatVector(proxy[i], proxy[j])
			mixed := sectionRatAdd(sectionRatCross(a0, b1), sectionRatCross(a1, b0))
			if sectionRatCross(a0, b0).Sign() <= 0 || sectionRatCross(a1, b1).Sign() <= 0 || mixed.Sign() <= 0 {
				return false
			}
		}
	}
	return true
}
