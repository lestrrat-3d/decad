// Package linkagebound computes exact projection bounds for linkage point readings.
package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Reading holds one body's enclosed points and their joint velocities.
type Reading struct {
	Pos    []motionbound.IvVec
	Vel    [][]motionbound.IvVec
	Pad    *big.Rat
	PrismK int
}

// StaticPoints is the reading of points no joint moves, with no velocity.
func StaticPoints(points []motionbound.RatVec) Reading {
	out := Reading{Pos: make([]motionbound.IvVec, len(points)), Vel: make([][]motionbound.IvVec, len(points))}
	for c, x := range points {
		out.Pos[c] = motionbound.PointVec(x)
	}
	return out
}

// ApplyIdeal maps an enclosed point through an ideal pose,
// x ↦ rot·(x − pivot) + pivot + shift.
func ApplyIdeal(p motionbound.IdealPose, x motionbound.IvVec) motionbound.IvVec {
	return motionbound.IvVecAdd(motionbound.IvVecAdd(p.Rot.Apply(motionbound.IvVecSub(x, p.Pivot)), p.Pivot), p.Shift)
}

// IvCross is the cross product a × b over rational intervals.
func IvCross(a, b motionbound.IvVec) motionbound.IvVec {
	term := func(i, j int) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a[i], b[j]), proofbound.IntervalMul(a[j], b[i]))
	}
	return motionbound.IvVec{term(1, 2), term(2, 0), term(0, 1)}
}

// ReadPoints maps any static point reading
// through the relative pose and given its velocity under each joint.
func ReadPoints(frames []motionbound.MotionFrame, params []motionbound.MotionParam, path []int, revolute []bool, below int, out Reading) Reading {
	type jointAt struct {
		revolute    bool
		unit, pivot motionbound.IvVec
	}
	joints := make([]jointAt, 0, len(path)-below)
	var pose *motionbound.IdealPose
	for _, i := range path[below:] {
		f := frames[i]
		var unit motionbound.IvVec
		for d := range 3 {
			unit[d] = proofbound.IntervalScale(f.Unit, f.Axis[d])
		}
		pivot := motionbound.PointVec(f.Center)
		ideal := f.At(params[i])
		if pose != nil {
			unit = pose.Rot.Apply(unit)
			pivot = ApplyIdeal(*pose, pivot)
			ideal = ideal.Then(*pose)
		}
		joints = append(joints, jointAt{revolute: revolute[i], unit: unit, pivot: pivot})
		pose = &ideal
	}
	for c := range out.Pos {
		if pose != nil {
			out.Pos[c] = ApplyIdeal(*pose, out.Pos[c])
		}
		out.Vel[c] = make([]motionbound.IvVec, len(joints))
		for n, j := range joints {
			if !j.revolute {
				out.Vel[c][n] = j.unit
				continue
			}
			out.Vel[c][n] = IvCross(j.unit, motionbound.IvVecSub(out.Pos[c], j.pivot))
		}
	}
	return out
}

// RoundOut widens an enclosure to the floats around it, read back as exact
// rationals; ok is false when an end overflows a float.
func RoundOut(iv proofbound.RatInterval) (proofbound.RatInterval, bool) {
	lo := proofarith.FloatRat(proofbound.RatFloatDown(iv.Lo))
	hi := proofarith.FloatRat(proofbound.RatFloatUp(iv.Hi))
	if lo == nil || hi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalOwned(lo, hi), true
}

// RoundCorners rounds a corner reading outward (Bounds); ok is false
// when a value overflows a float.
func RoundCorners(r Reading) (Bounds, bool) {
	out := Bounds{
		Lo: make([][3]*big.Rat, len(r.Pos)), Hi: make([][3]*big.Rat, len(r.Pos)),
		Vel: make([][][3]proofbound.RatInterval, len(r.Pos)), Pad: r.Pad, PrismK: r.PrismK,
	}
	for c := range r.Pos {
		for d := range 3 {
			iv, ok := RoundOut(r.Pos[c][d])
			if !ok {
				return Bounds{}, false
			}
			out.Lo[c][d], out.Hi[c][d] = iv.Lo, iv.Hi
		}
		out.Vel[c] = make([][3]proofbound.RatInterval, len(r.Vel[c]))
		for n, v := range r.Vel[c] {
			for d := range 3 {
				iv, ok := RoundOut(v[d])
				if !ok {
					return Bounds{}, false
				}
				out.Vel[c][n][d] = iv
			}
		}
	}
	return out, true
}
