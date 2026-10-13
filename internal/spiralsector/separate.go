package spiralsector

import (
	"context"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Separate proves source-solid or held-shell separation from
// certified mesh displacements. A solid lies inside the convex hull of its
// boundary, so separation along one YZ direction proves disjointness.
func Separate(ctx context.Context, shells []Shell, moves []r3.Transform, held bool) bool {
	if len(shells) != len(moves) {
		return false
	}
	maxHeld := 0.0
	for _, part := range shells {
		if proofbound.IsNonFinite(part.MeshBound) || part.MeshBound < 0 {
			return false
		}
		if held {
			maxHeld = max(maxHeld, part.MeshBound)
		}
	}
	// Each angular offset gets at most one exact source-slice certificate.
	// Float rotations at different stations are compared to that reference
	// through the exact recorded matrices, never assumed to compose exactly.
	referenceClearance := make(map[int]float64)
	for i := range shells {
		for j := i + 1; j < len(shells); j++ {
			if err := ctx.Err(); err != nil {
				return false
			}
			pi, pj := shells[i], shells[j]
			if proofbound.IsNonFinite(pi.MeshBound) ||
				proofbound.IsNonFinite(pj.MeshBound) || pi.MeshBound < 0 || pj.MeshBound < 0 {
				return false
			}
			heldAllowance := 0.0
			if held {
				heldAllowance = proofbound.AbsSumUpper(pi.MeshBound, pj.MeshBound)
			}
			angleOffset := j - i
			if gap := referenceClearance[angleOffset]; gap > 0 &&
				transferSeparation(shells[0], moves, i, j,
					angleOffset, gap, heldAllowance) {
				continue
			}
			ci := moves[i].ApplyDir(r3.Vec{Y: 1})
			cj := moves[j].ApplyDir(r3.Vec{Y: 1})
			ny, nz := ratSub(ratF(cj.Y), ratF(ci.Y)), ratSub(ratF(cj.Z), ratF(ci.Z))
			if ny.Sign() == 0 && nz.Sign() == 0 {
				return false
			}
			norm := proofbound.RatSqrtUp(ratAdd(ratMul(ny, ny), ratMul(nz, nz)))
			if proofbound.IsNonFinite(norm) || norm <= 0 {
				return false
			}
			loI, hiI, okI := projection(shells[i], ny, nz, norm)
			loJ, hiJ, okJ := projection(shells[j], ny, nz, norm)
			if !okI || !okJ {
				return false
			}
			if hiI.Cmp(loJ) < 0 || hiJ.Cmp(loI) < 0 {
				continue
			}
			if len(shells) > 2 {
				gap := referenceClearance[angleOffset]
				if gap == 0 {
					axialExtra, valid := sourceAxialDrift(shells[0], moves, angleOffset)
					if !valid {
						return false
					}
					// A held target point may be displaced in X from its source.
					// Include that bound before transferring this offset's slice gap.
					if held {
						axialExtra = proofbound.AbsSumUpper(axialExtra, maxHeld)
					}
					var ok bool
					gap, ok = slabsClearance(ctx, shells[0], shells[angleOffset], axialExtra)
					if !ok || gap <= 0 {
						return false
					}
					referenceClearance[angleOffset] = gap
				}
				if transferSeparation(shells[0], moves, i, j,
					angleOffset, gap, heldAllowance) {
					continue
				}
			}
			if _, ok := slabsClearance(ctx, shells[i], shells[j], 0); !ok {
				return false
			}
		}
	}
	return true
}

// A target pair at one X maps back to two reference points at most this far
// from that X. The first displacement compares Mi with its exact-X version;
// the second compares Mj with that version of Mi composed with Mk.
func sourceAxialDrift(p Shell, moves []r3.Transform, referenceIndex int) (float64, bool) {
	if len(p.Verts) == 0 || referenceIndex <= 0 || referenceIndex >= len(moves) ||
		proofbound.IsNonFinite(p.MeshBound) || p.MeshBound < 0 {
		return 0, false
	}
	xReach := 0.0
	for _, v := range p.Verts {
		if !proofbound.FiniteVec(v) {
			return 0, false
		}
		xReach = max(xReach, math.Abs(v.X))
	}
	xReach = proofbound.AbsSumUpper(xReach, p.MeshBound)
	firstScale := ratF(moves[referenceIndex].Basis().EX.X)
	defect := new(big.Rat)
	for _, move := range moves {
		scale := ratF(move.Basis().EX.X)
		for _, reference := range []*big.Rat{ratOne(), firstScale} {
			gap := ratAbs(ratSub(scale, reference))
			if gap.Cmp(defect) > 0 {
				defect = gap
			}
		}
	}
	bound := proofbound.ProductUpper(proofbound.RatFloatUp(defect), xReach)
	return bound, !proofbound.IsNonFinite(bound)
}

// A positive source-slice clearance for T and Mk(T) transfers to Mi(T) and
// Mj(T) when Mi*Mk differs from Mj by less than that clearance after Mi's
// length defect and both target held-shell bounds. The canonical left map
// leaves X unchanged; enlarged reference slices charge each actual X scale
// and each target held shell's possible axial displacement.
func transferSeparation(p Shell, moves []r3.Transform,
	i, j, referenceIndex int, clearance, heldAllowance float64) bool {
	if len(moves) < 2 || i == j || i < 0 || j < 0 ||
		referenceIndex <= 0 || referenceIndex >= len(moves) ||
		i >= len(moves) || j >= len(moves) || clearance <= 0 ||
		proofbound.IsNonFinite(clearance) || proofbound.IsNonFinite(heldAllowance) || heldAllowance < 0 {
		return false
	}
	if len(p.Verts) == 0 || proofbound.IsNonFinite(p.MeshBound) {
		return false
	}
	maxAbs := 0.0
	for _, v := range p.Verts {
		maxAbs = max(maxAbs, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	radius := proofbound.AbsSumUpper(proofbound.Radius3D(maxAbs), p.MeshBound)
	mi, errI := massmoment.PlacementRotation(moves[i])
	m1, err1 := massmoment.PlacementRotation(moves[referenceIndex])
	mj, errJ := massmoment.PlacementRotation(moves[j])
	if errI != nil || err1 != nil || errJ != nil {
		return false
	}
	// Replace only the left copy's one-ULP X scale. This exact-X map carries
	// the reference's X slabs unchanged; sourceAxialDrift pays for the
	// difference to the actual placed copy in the enlarged source slices.
	mi[0][0] = ratOne()
	charge, err := massmoment.MapChargeOf(mi)
	if err != nil || charge.Stretch >= 1 {
		return false
	}
	deviation := new(big.Rat)
	for row := range 3 {
		for column := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, ratMul(mi[row][k], m1[k][column]))
			}
			deviation.Add(deviation, ratAbs(ratSub(entry, mj[row][column])))
		}
	}
	allowance := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.RatFloatUp(deviation), radius), heldAllowance)
	return ratSub(ratMul(ratSub(ratOne(), ratF(charge.Stretch)), ratF(clearance)), ratF(allowance)).Sign() > 0
}

// A spiral tooth changes angle across its axial span. Whole-body angular
// projections can overlap even when every corresponding X slice is separate.
// Every true boundary point at X in one slab is within meshBound of a held
// triangle whose X range intersects that slab enlarged by meshBound. The
// projected extrema of those triangles, enlarged by meshBound again in the
// tested YZ direction, enclose that true slice. A bounded solid's occupied
// slice lies inside the convex hull of its boundary slice.
func slabsClearance(ctx context.Context, pa, pb Shell, axialExtra float64) (float64, bool) {
	if len(pa.Tris) == 0 || len(pb.Tris) == 0 ||
		proofbound.IsNonFinite(axialExtra) || axialExtra < 0 {
		return 0, false
	}
	clearance := math.Inf(1)
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, part := range []Shell{pa, pb} {
		if proofbound.IsNonFinite(part.MeshBound) || part.MeshBound < 0 {
			return 0, false
		}
		reach := part.MeshBound
		if axialExtra > 0 {
			reach = proofbound.AbsSumUpper(reach, axialExtra)
		}
		for _, v := range part.Verts {
			if !proofbound.FiniteVec(v) {
				return 0, false
			}
			lo = min(lo, math.Nextafter(v.X-reach, math.Inf(-1)))
			hi = max(hi, math.Nextafter(v.X+reach, math.Inf(1)))
		}
	}
	if !(lo < hi) || proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return 0, false
	}
	const slabs = 64
	levels := make([]float64, slabs+1)
	levels[0], levels[slabs] = lo, hi
	for i := 1; i < slabs; i++ {
		levels[i] = lo + (hi-lo)*float64(i)/slabs
		if !(levels[i-1] < levels[i]) {
			return 0, false
		}
	}
	if !(levels[slabs-1] < levels[slabs]) {
		return 0, false
	}
	for i := range slabs {
		if err := ctx.Err(); err != nil {
			return 0, false
		}
		fa, ca, okA := slabFacets(pa, levels[i], levels[i+1], axialExtra)
		fb, cb, okB := slabFacets(pb, levels[i], levels[i+1], axialExtra)
		if len(fa) == 0 || len(fb) == 0 {
			continue
		}
		if !okA || !okB {
			return 0, false
		}
		direction := cb.Sub(ca)
		ny, nz := ratF(direction.Y), ratF(direction.Z)
		if ny.Sign() == 0 && nz.Sign() == 0 {
			return 0, false
		}
		norm := proofbound.RatSqrtUp(ratAdd(ratMul(ny, ny), ratMul(nz, nz)))
		if proofbound.IsNonFinite(norm) || norm <= 0 {
			return 0, false
		}
		loA, hiA := slabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		loB, hiB := slabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		separate := loA == nil || loB == nil || hiA.Cmp(loB) < 0 || hiB.Cmp(loA) < 0
		if !separate {
			for _, axis := range slabHullAxes(pa, fa, pb, fb) {
				if err := ctx.Err(); err != nil {
					return 0, false
				}
				ny, nz := ratF(axis.Y), ratF(axis.Z)
				norm := proofbound.RatSqrtUp(ratAdd(ratMul(ny, ny), ratMul(nz, nz)))
				if proofbound.IsNonFinite(norm) || norm <= 0 {
					continue
				}
				loA, hiA = slabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
				loB, hiB = slabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
				if loA == nil || loB == nil || hiA.Cmp(loB) < 0 || hiB.Cmp(loA) < 0 {
					separate = true
					break
				}
			}
		}
		if !separate {
			return 0, false
		}
		if loA != nil && loB != nil {
			var gap *big.Rat
			if hiA.Cmp(loB) < 0 {
				gap = ratSub(loB, hiA)
			} else if hiB.Cmp(loA) < 0 {
				gap = ratSub(loA, hiB)
			}
			if gap != nil {
				lower := math.Nextafter(proofbound.RatFloatDown(gap)/norm, math.Inf(-1))
				clearance = min(clearance, max(0, lower))
			}
		}
	}
	return clearance, true
}

// The hull is only an axis generator. Every accepted axis is checked again
// using exact-rational projections of the clipped triangles and local bounds.
func slabHullAxes(a Shell, fa []int, b Shell, fb []int) []r3.Vec {
	type point struct{ y, z float64 }
	points := func(p Shell, facets []int) []point {
		out := make([]point, 0, 3*len(facets))
		for _, i := range facets {
			for _, vi := range p.Tris[i] {
				v := p.Verts[vi]
				out = append(out, point{v.Y, v.Z})
			}
		}
		return out
	}
	hull := func(p []point) []point {
		sort.Slice(p, func(i, j int) bool {
			if p[i].y != p[j].y {
				return p[i].y < p[j].y
			}
			return p[i].z < p[j].z
		})
		cross := func(a, b, c point) float64 {
			return (b.y-a.y)*(c.z-a.z) - (b.z-a.z)*(c.y-a.y)
		}
		out := make([]point, 0, 2*len(p))
		for _, v := range p {
			for len(out) >= 2 && cross(out[len(out)-2], out[len(out)-1], v) <= 0 {
				out = out[:len(out)-1]
			}
			out = append(out, v)
		}
		lower := len(out)
		for i := len(p) - 2; i >= 0; i-- {
			v := p[i]
			for len(out) > lower && cross(out[len(out)-2], out[len(out)-1], v) <= 0 {
				out = out[:len(out)-1]
			}
			out = append(out, v)
		}
		if len(out) > 1 {
			out = out[:len(out)-1]
		}
		return out
	}
	axes := make([]r3.Vec, 0, 64)
	for _, data := range []struct {
		p Shell
		f []int
	}{{a, fa}, {b, fb}} {
		h := hull(points(data.p, data.f))
		for i, v := range h {
			next := h[(i+1)%len(h)]
			axis := r3.Vec{Y: -(next.z - v.z), Z: next.y - v.y}
			if proofbound.FiniteVec(axis) && (axis.Y != 0 || axis.Z != 0) {
				axes = append(axes, axis)
			}
		}
	}
	return axes
}

func slabFacets(p Shell, lo, hi, axialExtra float64) ([]int, r3.Vec, bool) {
	reach := p.MeshBound
	if axialExtra > 0 {
		reach = proofbound.AbsSumUpper(reach, axialExtra)
	}
	searchLo := math.Nextafter(lo-reach, math.Inf(-1))
	searchHi := math.Nextafter(hi+reach, math.Inf(1))
	selected := make([]int, 0, len(p.Tris)/8)
	center := r3.Vec{}
	for i, tri := range p.Tris {
		a, b, c := p.Verts[tri[0]], p.Verts[tri[1]], p.Verts[tri[2]]
		if max(a.X, b.X, c.X) < searchLo || min(a.X, b.X, c.X) > searchHi {
			continue
		}
		selected = append(selected, i)
		center = center.Add(a).Add(b).Add(c)
	}
	if len(selected) == 0 {
		return nil, r3.Vec{}, false
	}
	center = center.Scale(1 / float64(3*len(selected)))
	return selected, center, proofbound.FiniteVec(center)
}

func slabProjection(p Shell, facets []int, slabLo, slabHi float64, ny, nz *big.Rat,
	norm, axialExtra float64) (*big.Rat, *big.Rat) {
	var lo, hi *big.Rat
	if len(p.VertexBound) != len(p.Verts) {
		return nil, nil
	}
	include := func(value, charge *big.Rat) {
		lower, upper := ratSub(value, charge), ratAdd(value, charge)
		if lo == nil || lower.Cmp(lo) < 0 {
			lo = lower
		}
		if hi == nil || upper.Cmp(hi) > 0 {
			hi = upper
		}
	}
	for _, i := range facets {
		tri := p.Tris[i]
		beta := max(p.VertexBound[tri[0]], p.VertexBound[tri[1]], p.VertexBound[tri[2]])
		if axialExtra > 0 {
			beta = proofbound.AbsSumUpper(beta, axialExtra)
		}
		charge := ratF(proofbound.ProductUpper(norm, beta))
		xLo := ratF(math.Nextafter(slabLo-beta, math.Inf(-1)))
		xHi := ratF(math.Nextafter(slabHi+beta, math.Inf(1)))
		var x, value [3]*big.Rat
		for k, vi := range tri {
			v := p.Verts[vi]
			x[k] = ratF(v.X)
			value[k] = ratAdd(ratMul(ny, ratF(v.Y)), ratMul(nz, ratF(v.Z)))
			if x[k].Cmp(xLo) >= 0 && x[k].Cmp(xHi) <= 0 {
				include(value[k], charge)
			}
		}
		for k := range 3 {
			next := (k + 1) % 3
			if x[k].Cmp(x[next]) == 0 {
				continue
			}
			for _, limit := range [...]*big.Rat{xLo, xHi} {
				if (x[k].Cmp(limit) < 0 && x[next].Cmp(limit) > 0) ||
					(x[k].Cmp(limit) > 0 && x[next].Cmp(limit) < 0) {
					t := new(big.Rat).Quo(ratSub(limit, x[k]), ratSub(x[next], x[k]))
					include(ratAdd(value[k], ratMul(t, ratSub(value[next], value[k]))), charge)
				}
			}
		}
	}
	return lo, hi
}

func projection(p Shell, ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	if len(p.Verts) == 0 || proofbound.IsNonFinite(p.MeshBound) || p.MeshBound < 0 {
		return nil, nil, false
	}
	var lo, hi *big.Rat
	for _, v := range p.Verts {
		if !proofbound.FiniteVec(v) {
			return nil, nil, false
		}
		value := ratAdd(ratMul(ny, ratF(v.Y)), ratMul(nz, ratF(v.Z)))
		if lo == nil || value.Cmp(lo) < 0 {
			lo = value
		}
		if hi == nil || value.Cmp(hi) > 0 {
			hi = value
		}
	}
	chargeBound := proofbound.ProductUpper(norm, p.MeshBound)
	if proofbound.IsNonFinite(chargeBound) {
		return nil, nil, false
	}
	charge := ratF(chargeBound)
	return ratSub(lo, charge), ratAdd(hi, charge), true
}
