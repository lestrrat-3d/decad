package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// axisBoxPrism admits a prism only when its recorded section is one complete
// rectangle, its frame preserves the world axes, and it has no placement.
// The body's exact Bounds then give the six actual planes of the solid.
func axisBoxPrism(b *Body) bool {
	if !b.solid || b.kind != BodySolid || b.bounds.Exactness != Exact || b.bounds.Bound.Base() != 0 {
		return false
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || pp.surfaceResult || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 {
		return false
	}
	if !cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) {
		return false
	}
	basis := pp.xform.Basis()
	if basis.EX != (r3.Vec{X: 1}) || basis.EY != (r3.Vec{Y: 1}) ||
		basis.EZ != (r3.Vec{Z: 1}) || pp.xform.Translation() != (r3.Vec{}) {
		return false
	}
	return rectangularProfile(pp.profile)
}

func cardinalBasis(u, v, n r3.Vec) bool {
	axis := func(p r3.Vec) int {
		switch {
		case math.Abs(p.X) == 1 && p.Y == 0 && p.Z == 0:
			return 1
		case p.X == 0 && math.Abs(p.Y) == 1 && p.Z == 0:
			return 2
		case p.X == 0 && p.Y == 0 && math.Abs(p.Z) == 1:
			return 3
		default:
			return 0
		}
	}
	a, b, c := axis(u), axis(v), axis(n)
	return a != 0 && b != 0 && c != 0 && a != b && a != c && b != c
}

func rectangularProfile(profile ProfileRecord) bool {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return false
	}
	var corners [4]Point2
	var ends [4]Point2
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(LineSeg)
		if !ok {
			return false
		}
		switch {
		case line.TStart == 0 && line.TEnd == 1:
			corners[i], ends[i] = line.Start, line.End
		case line.TStart == 1 && line.TEnd == 0:
			corners[i], ends[i] = line.End, line.Start
		default:
			return false
		}
		if corners[i].U == ends[i].U && corners[i].V == ends[i].V {
			return false
		}
		if corners[i].U != ends[i].U && corners[i].V != ends[i].V {
			return false
		}
	}
	minU, maxU := corners[0].U, corners[0].U
	minV, maxV := corners[0].V, corners[0].V
	for _, p := range corners[1:] {
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
	}
	if minU == maxU || minV == maxV {
		return false
	}
	var seen [4]bool
	for i, p := range corners {
		if ends[i] != corners[(i+1)%4] {
			return false
		}
		if (p.U != minU && p.U != maxU) || (p.V != minV && p.V != maxV) {
			return false
		}
		corner := 0
		if p.U == maxU {
			corner += 1
		}
		if p.V == maxV {
			corner += 2
		}
		if seen[corner] {
			return false
		}
		seen[corner] = true
	}
	return true
}

// clearanceAxisBoxes gives the closed-form gap of two certified rectangular
// solids. A positive box gap also excludes nesting. Contact and near-contact
// keep the general kernel's existing certificate and tolerance decisions.
func clearanceAxisBoxes(a, b *Body) (pairResult, bool) {
	if !axisBoxPrism(a) || !axisBoxPrism(b) {
		return pairResult{}, false
	}
	axisGap := func(amin, amax, bmin, bmax float64) boundedScalar {
		if amax < bmin {
			return boundedSub(exactScalar(bmin), exactScalar(amax))
		}
		if bmax < amin {
			return boundedSub(exactScalar(amin), exactScalar(bmax))
		}
		return exactScalar(0)
	}
	ab, bb := a.bounds, b.bounds
	dx := axisGap(ab.Min.X, ab.Max.X, bb.Min.X, bb.Max.X)
	dy := axisGap(ab.Min.Y, ab.Max.Y, bb.Min.Y, bb.Max.Y)
	dz := axisGap(ab.Min.Z, ab.Max.Z, bb.Min.Z, bb.Max.Z)
	squared := boundedAdd(boundedAdd(boundedMul(dx, dx), boundedMul(dy, dy)), boundedMul(dz, dz))
	gap := boundedSqrt(squared)
	lo, hi := boundedEnds(gap)
	lo = math.Max(0, lo)
	scale := max(1, math.Abs(ab.Min.X), math.Abs(ab.Min.Y), math.Abs(ab.Min.Z),
		math.Abs(ab.Max.X), math.Abs(ab.Max.Y), math.Abs(ab.Max.Z),
		math.Abs(bb.Min.X), math.Abs(bb.Min.Y), math.Abs(bb.Min.Z),
		math.Abs(bb.Max.X), math.Abs(bb.Max.Y), math.Abs(bb.Max.Z))
	if isNonFinite(hi) || lo <= 1e-9*scale {
		return pairResult{}, false
	}
	// The farthest distance within or between two boxes occurs at corners.
	// Use downward endpoints so the reference diameter never overstates it.
	distanceLower := func(x, y, z boundedScalar) float64 {
		sum := boundedAdd(boundedAdd(boundedMul(x, x), boundedMul(y, y)), boundedMul(z, z))
		lower, _ := boundedEnds(boundedSqrt(sum))
		return math.Max(0, lower)
	}
	span := func(lo, hi float64) boundedScalar {
		return boundedSub(exactScalar(hi), exactScalar(lo))
	}
	far := func(amin, amax, bmin, bmax float64) boundedScalar {
		left := span(bmin, amax)
		right := span(amin, bmax)
		if left.value >= right.value {
			return left
		}
		return right
	}
	diam := max(
		distanceLower(span(ab.Min.X, ab.Max.X), span(ab.Min.Y, ab.Max.Y), span(ab.Min.Z, ab.Max.Z)),
		distanceLower(span(bb.Min.X, bb.Max.X), span(bb.Min.Y, bb.Max.Y), span(bb.Min.Z, bb.Max.Z)),
		distanceLower(
			far(ab.Min.X, ab.Max.X, bb.Min.X, bb.Max.X),
			far(ab.Min.Y, ab.Max.Y, bb.Min.Y, bb.Max.Y),
			far(ab.Min.Z, ab.Max.Z, bb.Min.Z, bb.Max.Z),
		),
	)
	if isNonFinite(diam) {
		return pairResult{}, false
	}
	return pairResult{verdict: pairDisjoint, lo: lo, hi: hi, exact: gap.bound == 0, diam: diam}, true
}
