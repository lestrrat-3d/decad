package box

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// SourceOrientedBox records the exact source prism under its placement and
// query pose. The returned placed edges identify its original planar faces.
func SourceOrientedBox(profile momentinput.Profile, frame r3.Frame, z0, z1 float64,
	placement, pose r3.Transform) (OrientedBox, [3]proofarith.DyV3, bool) {
	if !RectangularProfile(profile) || !CardinalBasis(frame.U(), frame.V(), frame.N()) ||
		!pose.IsValid() || !proofbound.FiniteVec(pose.Translation()) ||
		!proofbound.FiniteVec(frame.Origin()) || !finiteSourceValues(z0, z1) {
		return OrientedBox{}, [3]proofarith.DyV3{}, false
	}
	var umin, umax, vmin, vmax float64
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return OrientedBox{}, [3]proofarith.DyV3{}, false
		}
		point := line.Start
		if i == 0 {
			umin, umax, vmin, vmax = point.U, point.U, point.V, point.V
		}
		umin, umax = math.Min(umin, point.U), math.Max(umax, point.U)
		vmin, vmax = math.Min(vmin, point.V), math.Max(vmax, point.V)
	}
	if !finiteSourceValues(umin, umax, vmin, vmax) || umin >= umax || vmin >= vmax || z0 >= z1 {
		return OrientedBox{}, [3]proofarith.DyV3{}, false
	}
	values := [3][2]proofarith.Dyadic{{proofarith.MustDyOf(umin), proofarith.MustDyOf(umax)},
		{proofarith.MustDyOf(vmin), proofarith.MustDyOf(vmax)}, {proofarith.MustDyOf(z0), proofarith.MustDyOf(z1)}}
	basis := [3]proofarith.DyV3{proofarith.DyVec(frame.U()), proofarith.DyVec(frame.V()), proofarith.DyVec(frame.N())}
	origin := proofarith.DyVec(frame.Origin())
	var box OrientedBox
	for index := range box.Corner {
		point := origin
		for axis := range 3 {
			point = proofarith.DvAdd(point, proofarith.DvScale(basis[axis], values[axis][(index>>axis)&1]))
		}
		box.Corner[index] = proofarith.DvTransform(pose, proofarith.DvTransform(placement, point))
	}
	box.Edge = [3]proofarith.DyV3{proofarith.DvSub(box.Corner[1], box.Corner[0]),
		proofarith.DvSub(box.Corner[2], box.Corner[0]), proofarith.DvSub(box.Corner[4], box.Corner[0])}
	for axis := range 3 {
		if proofarith.DvIsZero(box.Edge[axis]) {
			return OrientedBox{}, [3]proofarith.DyV3{}, false
		}
	}
	var placedEdge [3]proofarith.DyV3
	for axis := range 3 {
		placedEdge[axis] = proofarith.DvTransform(placement,
			proofarith.DvScale(basis[axis], proofarith.DySubScalar(values[axis][1], values[axis][0])))
		placedEdge[axis] = proofarith.DvSub(placedEdge[axis], proofarith.DyVec(placement.Translation()))
	}
	return box, placedEdge, true
}

func finiteSourceValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
