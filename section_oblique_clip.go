package decad

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/r3"
)

// obliqueRectangleClip intersects the exact held rectangular prism with the
// exact affine plane spanned by the requested held U and V vectors. It returns
// a rounded plane-local polygon and a displacement covering every matched
// point of its exact source boundary in both local and world coordinates.
func obliqueRectangleClip(pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
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
	var exact []sectionclip.SectionRatPoint
	for a := range corners {
		for coordinate := range 3 {
			b := a ^ (1 << coordinate)
			if b < a || side[a].Sign() == side[b].Sign() {
				continue
			}
			t := sectionclip.SectionRatQuo(side[a].Rat(), proofarith.DySubScalar(side[a], side[b]).Rat())
			var world [3]*big.Rat
			for component := range 3 {
				world[component] = sectionclip.SectionRatAdd(corners[a][component].Rat(),
					sectionclip.SectionRatMul(t, sectionclip.SectionRatSub(corners[b][component].Rat(), corners[a][component].Rat())))
			}
			point, ok := sectionPlaneCoordinates(world, plane)
			if !ok {
				return nil, 0, false
			}
			duplicate := false
			for _, other := range exact {
				if point.U.Cmp(other.U) == 0 && point.V.Cmp(other.V) == 0 {
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
	proxy := make([]planePoint, len(exact))
	delta := float64(0)
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	for i, point := range exact {
		u, _ := point.U.Float64()
		v, _ := point.V.Float64()
		if math.IsNaN(u) || math.IsInf(u, 0) || math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, 0, false
		}
		proxy[i] = planePoint{U: u, V: v}
		uError := proofarith.RationalFloatError(point.U, u)
		vError := proofarith.RationalFloatError(point.V, v)
		local := proofbound.AbsSumUpper(uError, vError)
		world := proofbound.AbsSumUpper(proofbound.ProductUpper(uError, uNorm),
			proofbound.ProductUpper(vError, vNorm))
		delta = max(delta, local, world)
	}
	if !sectionclip.SectionMatchedConvex(exact, proxy) || math.IsInf(delta, 0) || math.IsNaN(delta) {
		return nil, 0, false
	}
	delta = proofbound.UpRound(delta)
	// A loop whose coordinate error is comparable to a source edge has no
	// useful local feature scale for later reconstruction and verification.
	for i, point := range exact {
		next := exact[(i+1)%len(exact)]
		uSpan := new(big.Rat).Abs(sectionclip.SectionRatSub(next.U, point.U))
		vSpan := new(big.Rat).Abs(sectionclip.SectionRatSub(next.V, point.V))
		edgeL1 := sectionclip.SectionRatAdd(uSpan, vSpan)
		if edgeL1.Sign() <= 0 || sectionclip.SectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}

func sectionPlaneCoordinates(world [3]*big.Rat, plane r3.Frame) (sectionclip.SectionRatPoint, bool) {
	origin := proofarith.DyVec(plane.Origin())
	uAxis := proofarith.DyVec(plane.U())
	vAxis := proofarith.DyVec(plane.V())
	for a := range 3 {
		for b := a + 1; b < 3; b++ {
			det := sectionclip.SectionRatSub(sectionclip.SectionRatMul(uAxis[a].Rat(), vAxis[b].Rat()),
				sectionclip.SectionRatMul(uAxis[b].Rat(), vAxis[a].Rat()))
			if det.Sign() == 0 {
				continue
			}
			da := sectionclip.SectionRatSub(world[a], origin[a].Rat())
			db := sectionclip.SectionRatSub(world[b], origin[b].Rat())
			u := sectionclip.SectionRatQuo(sectionclip.SectionRatSub(sectionclip.SectionRatMul(da, vAxis[b].Rat()),
				sectionclip.SectionRatMul(db, vAxis[a].Rat())), det)
			v := sectionclip.SectionRatQuo(sectionclip.SectionRatSub(sectionclip.SectionRatMul(uAxis[a].Rat(), db),
				sectionclip.SectionRatMul(uAxis[b].Rat(), da)), det)
			for component := range 3 {
				lifted := sectionclip.SectionRatAdd(origin[component].Rat(),
					sectionclip.SectionRatAdd(sectionclip.SectionRatMul(u, uAxis[component].Rat()), sectionclip.SectionRatMul(v, vAxis[component].Rat())))
				if lifted.Cmp(world[component]) != 0 {
					return sectionclip.SectionRatPoint{}, false
				}
			}
			return sectionclip.SectionRatPoint{U: u, V: v}, true
		}
	}
	return sectionclip.SectionRatPoint{}, false
}

func sectionSortRatPolygon(points []sectionclip.SectionRatPoint) {
	center := sectionclip.SectionRatPoint{U: new(big.Rat), V: new(big.Rat)}
	for _, point := range points {
		center.U.Add(center.U, point.U)
		center.V.Add(center.V, point.V)
	}
	count := new(big.Rat).SetInt64(int64(len(points)))
	center.U.Quo(center.U, count)
	center.V.Quo(center.V, count)
	half := func(p sectionclip.SectionRatPoint) int {
		if p.V.Sign() > 0 || (p.V.Sign() == 0 && p.U.Sign() >= 0) {
			return 0
		}
		return 1
	}
	sort.Slice(points, func(i, j int) bool {
		a, b := sectionclip.SectionRatVector(center, points[i]), sectionclip.SectionRatVector(center, points[j])
		if half(a) != half(b) {
			return half(a) < half(b)
		}
		return sectionclip.SectionRatCross(a, b).Sign() > 0
	})
}
