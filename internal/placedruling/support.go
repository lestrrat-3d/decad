package placedruling

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// Plane is the exact planar face selected as the cylinder's support.
type Plane struct {
	Normal        proofarith.DyV3
	Axis, Origin  int
	Offset, Alpha proofarith.Dyadic
	Heights       [2]proofarith.Dyadic
	Face          planar.SupportFace
	Local         bool
	Clearance     *big.Rat
}

var alphaLimit = proofarith.MustDyOf(1.0 / 4)

type triedPlane struct{ normal, origin proofarith.DyV3 }

// Support picks the plane a placed cylinder rests on: among S's face
// planes whose outward normal is a signed axis, with |α| <= 1/4 and S's
// triangles on it belonging to one face, the one whose larger |H±| is least,
// the first in S's triangle order on a tie. A plane with an S vertex strictly
// in front of it is face-local (§10.6): it is admitted only when the column
// test holds at f = 0 over the coordinate box of the staged corners, and it
// records that box's lateral clearance. points are S's staged vertices.
func Support(c Cylinder, solid *planar.PlanarSolid, points []proofarith.DyV3,
	poll func() error) (Plane, bool, error) {
	var best Plane
	var bestScore proofarith.Dyadic
	found := false
	var tried []triedPlane
	for _, tri := range solid.Tris {
		if err := poll(); err != nil {
			return Plane{}, false, err
		}
		origin := points[tri[0]]
		n := proofarith.DvCross(proofarith.DvSub(points[tri[1]], origin),
			proofarith.DvSub(points[tri[2]], origin))
		axis, unit, ok := unitAxis(n)
		if !ok || planeTried(tried, unit, origin) {
			continue
		}
		tried = append(tried, triedPlane{normal: unit, origin: origin})
		alpha := proofarith.DvDot(unit, c.Columns[c.Axis])
		if proofarith.DyCmp(proofarith.DyAbs(alpha), alphaLimit) > 0 {
			continue
		}
		plane := Plane{Normal: unit, Axis: axis, Origin: tri[0], Offset: origin[axis], Alpha: alpha}
		score := proofarith.DyZero()
		for i, center := range c.Centers {
			plane.Heights[i] = proofarith.DySubScalar(proofarith.DvDot(unit, proofarith.DvSub(center, origin)), c.Radius)
			score = maxDy(score, proofarith.DyAbs(plane.Heights[i]))
		}
		if found && proofarith.DyCmp(score, bestScore) >= 0 {
			continue
		}
		face, ok := planar.BuildSupportFace(solid, points, unit, origin)
		if !ok {
			continue
		}
		plane.Face = face
		for _, v := range points {
			if proofarith.DvDot(unit, proofarith.DvSub(v, origin)).Sign() > 0 {
				plane.Local = true
				break
			}
		}
		if plane.Local {
			clearance, apart, err := column(c, solid, points, unit, origin, poll)
			if err != nil {
				return Plane{}, false, err
			}
			if !apart {
				continue
			}
			plane.Clearance = clearance
		}
		best, bestScore, found = plane, score, true
	}
	return best, found, nil
}

func planeTried(tried []triedPlane, n, a proofarith.DyV3) bool {
	for _, p := range tried {
		if proofarith.DvIsZero(proofarith.DvCross(n, p.normal)) &&
			proofarith.DvDot(n, p.normal).Sign() > 0 &&
			proofarith.DvDot(p.normal, proofarith.DvSub(a, p.origin)).Sign() == 0 {
			return true
		}
	}
	return false
}

func unitAxis(n proofarith.DyV3) (int, proofarith.DyV3, bool) {
	axis := -1
	for k := range 3 {
		if n[k].Sign() == 0 {
			continue
		}
		if axis >= 0 {
			return 0, proofarith.DyV3{}, false
		}
		axis = k
	}
	if axis < 0 {
		return 0, proofarith.DyV3{}, false
	}
	var unit proofarith.DyV3
	unit[axis] = proofarith.DyInt(int64(n[axis].Sign()))
	return axis, unit, true
}

// column is §10.6's column test at f = 0 for a placed cylinder: every
// S triangle with a vertex strictly in front of the plane through origin
// must project apart from the coordinate box of the staged corners, whose
// hull holds the cylinder. clearance is that box's lateral clearance, nil for
// an unbounded one (planar.PlanarColumnClear).
func column(c Cylinder, solid *planar.PlanarSolid, points []proofarith.DyV3, unit, origin proofarith.DyV3,
	poll func() error) (*big.Rat, bool, error) {
	var lo, hi [3]*big.Rat
	for i, corner := range c.StagedCorners() {
		for k := range 3 {
			v := corner[k].Rat()
			if i == 0 || v.Cmp(lo[k]) < 0 {
				lo[k] = v
			}
			if i == 0 || v.Cmp(hi[k]) > 0 {
				hi[k] = v
			}
		}
	}
	staged := planar.PlanarSolid{Verts: points, Tris: solid.Tris}
	return planar.PlanarColumnClear(&staged, unit, origin, lo, hi, poll)
}

func maxDy(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}
