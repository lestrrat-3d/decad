package revolveloftfaces

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ShellRayCap is a certified cap endpoint: its held point, its blank-side
// bound and its radial source box.
type ShellRayCap struct {
	Point      r3.Vec
	BlankBound float64
	Box        RayBox
}

// ShellAnchors holds the authenticated source stations and cap endpoints that
// one blank shell places on its angular grid. Seams holds one station run per
// whole-arc source contact; a cap endpoint's ID is its index in Caps.
type ShellAnchors struct {
	Seams [][]ArcSeamVertex
	Caps  []ShellRayCap
}

// ShellRayRef names one station of one source run.
type ShellRayRef struct{ Run, Station int }

// ShellRay is one transient exact direction with its source anchor IDs.
type ShellRay struct {
	Y, Z      *big.Rat
	SeamIndex int
	SeamRefs  []ShellRayRef
	CapIDs    []int
}

// ShellTriangleCell labels one blank-shell triangle by its Revolve section,
// adjacent ring band, angular ray interval and triangle half.
type ShellTriangleCell struct {
	Section, Band, Ray, Half int
}

// ShellRays builds one angular grid for all source station runs and cap endpoints.
// Coincident stations from different runs keep separate source IDs.
func ShellRays(n int, anchors ShellAnchors) []ShellRay {
	rays := make([]ShellRay, 0, 4*n+len(anchors.Caps))
	for quadrant := range 4 {
		for i := range n {
			q := big.NewRat(int64(i), int64(n))
			q2 := lfMul(q, q)
			denominator := lfAdd(lfOne(), q2)
			c := lfDiv(lfSub(lfOne(), q2), denominator)
			s := lfDiv(lfMul(big.NewRat(2, 1), q), denominator)
			ray := ShellRay{SeamIndex: -1}
			switch quadrant {
			case 0:
				ray.Y, ray.Z = c, s
			case 1:
				ray.Y, ray.Z = lfSub(lfZero(), s), c
			case 2:
				ray.Y, ray.Z = lfSub(lfZero(), c), lfSub(lfZero(), s)
			case 3:
				ray.Y, ray.Z = s, lfSub(lfZero(), c)
			}
			rays = append(rays, ray)
		}
	}
	for i, point := range anchors.Caps {
		if point.Point.Y == 0 && point.Point.Z == 0 {
			return nil
		}
		rays = append(rays, ShellRay{
			Y: lfF(point.Point.Y), Z: lfF(point.Point.Z), SeamIndex: -1,
			CapIDs: []int{i}})
	}
	sort.Slice(rays, func(i, j int) bool {
		return RayBefore(rays[i].Y, rays[i].Z, rays[j].Y, rays[j].Z)
	})
	unique := rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !sameShellRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		// A cap endpoint needs its own ray. A collision with a grid or
		// another cap ray would erase its certified angular interval.
		if len(ray.CapIDs) != 0 || len(unique[len(unique)-1].CapIDs) != 0 {
			return nil
		}
	}
	rays = unique
	for run, seam := range anchors.Seams {
		for station, point := range seam {
			rays = append(rays, ShellRay{Y: point.Exact[1], Z: point.Exact[2],
				SeamIndex: -1, SeamRefs: []ShellRayRef{{Run: run, Station: station}}})
		}
	}
	sort.Slice(rays, func(i, j int) bool {
		return RayBefore(rays[i].Y, rays[i].Z, rays[j].Y, rays[j].Z)
	})
	unique = rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !sameShellRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		previous := &unique[len(unique)-1]
		if len(ray.CapIDs) != 0 || len(previous.CapIDs) != 0 {
			return nil
		}
		for _, ref := range ray.SeamRefs {
			for _, existing := range previous.SeamRefs {
				if existing.Run == ref.Run {
					return nil
				}
			}
			previous.SeamRefs = append(previous.SeamRefs, ref)
		}
	}
	return unique
}

func sameShellRay(a, b ShellRay) bool {
	return lfSub(lfMul(a.Y, b.Z), lfMul(a.Z, b.Y)).Sign() == 0 &&
		lfAdd(lfMul(a.Y, b.Y), lfMul(a.Z, b.Z)).Sign() > 0
}

// ShellRayOrderBounded checks the angular order of every source anchor. With
// one run it also sets each ray's SeamIndex.
func ShellRayOrderBounded(rays []ShellRay, anchors ShellAnchors) bool {
	if len(anchors.Seams) == 1 {
		for i := range rays {
			if len(rays[i].SeamRefs) == 1 {
				rays[i].SeamIndex = rays[i].SeamRefs[0].Station
			}
		}
		if len(anchors.Caps) == 0 {
			return true
		}
		orderRays := make([]RayOrderRay, len(rays))
		for i, ray := range rays {
			orderRays[i] = RayOrderRay{
				Y: ray.Y, Z: ray.Z, SeamIndex: ray.SeamIndex, CapCount: len(ray.CapIDs),
			}
			if len(ray.CapIDs) != 0 {
				orderRays[i].CapID = ray.CapIDs[0]
			}
		}
		orderSeam := make([]RayOrderSeam, len(anchors.Seams[0]))
		for i, point := range anchors.Seams[0] {
			orderSeam[i] = RayOrderSeam{Point: point.Point, LoftBound: point.LoftBound}
		}
		capBoxes := make([]RayBox, len(anchors.Caps))
		for i, point := range anchors.Caps {
			capBoxes[i] = point.Box
		}
		return CapRayOrderBounded(orderRays, orderSeam, capBoxes)
	}
	return allShellRayOrderBounded(rays, anchors)
}

// Every source anchor must stay in the same cyclic angular order as the
// shared ring grid. A cap endpoint and a seam station cannot share a ray.
func allShellRayOrderBounded(rays []ShellRay, anchors ShellAnchors) bool {
	if len(rays) < 4 {
		return false
	}
	boxes := make([][]RayBox, len(rays))
	seenCaps := make([]bool, len(anchors.Caps))
	seenStations := make([][]bool, len(anchors.Seams))
	for run := range seenStations {
		seenStations[run] = make([]bool, len(anchors.Seams[run]))
	}
	for index, ray := range rays {
		if ray.Y == nil || ray.Z == nil || len(ray.CapIDs) > 1 ||
			len(ray.CapIDs) > 0 && len(ray.SeamRefs) > 0 {
			return false
		}
		if len(ray.CapIDs) == 0 && len(ray.SeamRefs) == 0 {
			boxes[index] = []RayBox{{
				Y: loftface.Interval{Lo: ray.Y, Hi: ray.Y},
				Z: loftface.Interval{Lo: ray.Z, Hi: ray.Z}}}
		}
		for _, id := range ray.CapIDs {
			if id < 0 || id >= len(anchors.Caps) || seenCaps[id] {
				return false
			}
			seenCaps[id] = true
			boxes[index] = append(boxes[index], anchors.Caps[id].Box)
		}
		for _, ref := range ray.SeamRefs {
			if ref.Run < 0 || ref.Run >= len(anchors.Seams) ||
				ref.Station < 0 || ref.Station >= len(anchors.Seams[ref.Run]) ||
				seenStations[ref.Run][ref.Station] {
				return false
			}
			seenStations[ref.Run][ref.Station] = true
			point := anchors.Seams[ref.Run][ref.Station]
			if proofbound.IsNonFinite(point.LoftBound) || point.LoftBound < 0 {
				return false
			}
			bound := lfF(point.LoftBound)
			y, z := lfF(point.Point.Y), lfF(point.Point.Z)
			boxes[index] = append(boxes[index], RayBox{
				Y: loftface.Interval{Lo: lfSub(y, bound), Hi: lfAdd(y, bound)},
				Z: loftface.Interval{Lo: lfSub(z, bound), Hi: lfAdd(z, bound)}})
		}
		for _, box := range boxes[index] {
			if !RayBoxValid(box) {
				return false
			}
		}
	}
	for _, seen := range seenCaps {
		if !seen {
			return false
		}
	}
	for _, run := range seenStations {
		for _, seen := range run {
			if !seen {
				return false
			}
		}
	}
	for index := range rays {
		next := (index + 1) % len(rays)
		for _, first := range boxes[index] {
			for _, second := range boxes[next] {
				if RayCrossRange(first, second).Lo.Sign() <= 0 {
					return false
				}
			}
		}
	}
	return true
}

// MeridianRayPoint holds the point at exact X on a ray at the given radius
// from the Revolve axis. Its bound charges the rounded ray length and the
// held point's rounding.
func MeridianRayPoint(ray [2]*big.Rat, x, radius *big.Rat) (r3.Vec, float64, bool) {
	if ray[0] == nil || ray[1] == nil || x == nil || radius == nil || radius.Sign() <= 0 {
		return r3.Vec{}, 0, false
	}
	r2 := lfAdd(lfMul(ray[0], ray[0]), lfMul(ray[1], ray[1]))
	stored, _ := r2.Float64()
	if proofbound.IsNonFinite(stored) || stored <= 0 {
		return r3.Vec{}, 0, false
	}
	denominator := math.Sqrt(stored)
	if proofbound.IsNonFinite(denominator) || denominator <= 0 {
		return r3.Vec{}, 0, false
	}
	scale := lfDiv(radius, lfF(denominator))
	exact := loftface.RatVec{x, lfMul(ray[0], scale), lfMul(ray[1], scale)}
	point, round, ok := HeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	actual := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	residual := proofbound.RatFloatUp(lfDiv(
		lfAbs(lfSub(actual, lfMul(radius, radius))), radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}
