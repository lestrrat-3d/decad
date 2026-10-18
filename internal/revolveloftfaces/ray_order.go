package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RayOrderRay is the private numeric snapshot of one sorted shell ray.
type RayOrderRay struct {
	Y, Z      *big.Rat
	SeamIndex int
	CapCount  int
	CapID     int
}

// RayOrderSeam carries one seam point and its source movement bound.
type RayOrderSeam struct {
	Point     r3.Vec
	LoftBound float64
}

// RayBox encloses one source ray in the radial plane.
type RayBox struct {
	Y, Z loftface.Interval
}

// CapRayOrderBounded requires each cap endpoint's source box to stay strictly
// between its neighboring rays, including a neighboring seam's movement.
func CapRayOrderBounded(rays []RayOrderRay, seam []RayOrderSeam, capBoxes []RayBox) bool {
	if len(capBoxes) == 0 {
		return true
	}
	if len(rays) < 4 {
		return false
	}
	boxes := make([]RayBox, len(rays))
	capRay := make([]int, len(capBoxes))
	for i := range capRay {
		capRay[i] = -1
	}
	for index, ray := range rays {
		if ray.Y == nil || ray.Z == nil || ray.CapCount > 1 {
			return false
		}
		box := RayBox{
			Y: loftface.Interval{Lo: ray.Y, Hi: ray.Y},
			Z: loftface.Interval{Lo: ray.Z, Hi: ray.Z},
		}
		if ray.SeamIndex >= 0 {
			if ray.SeamIndex >= len(seam) || ray.CapCount != 0 {
				return false
			}
			point := seam[ray.SeamIndex]
			if proofbound.IsNonFinite(point.LoftBound) || point.LoftBound < 0 {
				return false
			}
			bound := lfF(point.LoftBound)
			y, z := lfF(point.Point.Y), lfF(point.Point.Z)
			box.Y = loftface.Interval{Lo: lfSub(y, bound), Hi: lfAdd(y, bound)}
			box.Z = loftface.Interval{Lo: lfSub(z, bound), Hi: lfAdd(z, bound)}
		}
		if ray.CapCount == 1 {
			id := ray.CapID
			if id < 0 || id >= len(capBoxes) || capRay[id] >= 0 || ray.SeamIndex >= 0 {
				return false
			}
			capRay[id] = index
			box = capBoxes[id]
		}
		if !RayBoxValid(box) {
			return false
		}
		boxes[index] = box
	}
	for _, index := range capRay {
		if index < 0 {
			return false
		}
		prev := (index - 1 + len(rays)) % len(rays)
		next := (index + 1) % len(rays)
		if prev < index && !RayBefore(rays[prev].Y, rays[prev].Z, rays[index].Y, rays[index].Z) ||
			next > index && !RayBefore(rays[index].Y, rays[index].Z, rays[next].Y, rays[next].Z) ||
			RayCrossRange(boxes[prev], boxes[index]).Lo.Sign() <= 0 ||
			RayCrossRange(boxes[index], boxes[next]).Lo.Sign() <= 0 {
			return false
		}
	}
	return true
}

// RayBefore compares two exact radial directions in cyclic order.
func RayBefore(ay, az, by, bz *big.Rat) bool {
	upper := func(y, z *big.Rat) bool {
		return z.Sign() > 0 || z.Sign() == 0 && y.Sign() >= 0
	}
	if upper(ay, az) != upper(by, bz) {
		return upper(ay, az)
	}
	return lfSub(lfMul(ay, bz), lfMul(az, by)).Sign() > 0
}

// RayBoxValid checks both exact radial intervals.
func RayBoxValid(box RayBox) bool {
	return box.Y.Lo != nil && box.Y.Hi != nil && box.Z.Lo != nil && box.Z.Hi != nil &&
		box.Y.Lo.Cmp(box.Y.Hi) <= 0 && box.Z.Lo.Cmp(box.Z.Hi) <= 0
}

// RayCrossRange bounds the cross product of two radial source boxes.
func RayCrossRange(a, b RayBox) loftface.Interval {
	return loftface.IntervalSub(rayProductRange(a.Y, b.Z), rayProductRange(a.Z, b.Y))
}

func rayProductRange(a, b loftface.Interval) loftface.Interval {
	products := [4]*big.Rat{
		lfMul(a.Lo, b.Lo), lfMul(a.Lo, b.Hi),
		lfMul(a.Hi, b.Lo), lfMul(a.Hi, b.Hi),
	}
	out := loftface.Interval{Lo: products[0], Hi: products[0]}
	for _, product := range products[1:] {
		if product.Cmp(out.Lo) < 0 {
			out.Lo = product
		}
		if product.Cmp(out.Hi) > 0 {
			out.Hi = product
		}
	}
	return out
}
