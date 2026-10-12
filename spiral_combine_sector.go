package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralCombineInputs authenticates every tool as a copy of the first trim.
// Distinct sector projections prove the complete source solids disjoint before
// their meshes enter one Boolean operand.
func spiralCombineInputs(ctx context.Context, blank *Body, tools []*Body) (*spiralCombineSource, error) {
	if len(tools) < 2 || len(tools) > 31 {
		return nil, fmt.Errorf("%w: this spiral-tooth Union class accepts two to 31 tools", ErrUnsupported)
	}
	first, ok, err := certifySpiralFirstJoin(ctx, blank, tools[0])
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", ErrUnsupported)
	}
	moves := make([]r3.Transform, len(tools))
	moves[0] = r3.Identity()
	for i, tool := range tools[1:] {
		p, ok := tool.payload.(facetedPayload)
		if !ok || p.spiralTooth == nil || p.spiralTooth.source != first.tooth ||
			p.spiralTooth.motion != p.xform || !spiralNearAxisRotation(p.xform) {
			return nil, fmt.Errorf("%w: tool %d is not a rigid copy of the first spiral tooth", ErrUnsupported, i+2)
		}
		moves[i+1] = p.xform
	}
	if !spiralToolSectorsSeparate(ctx, tools, moves, false) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, fmt.Errorf("%w: spiral tooth sectors are not proven disjoint", ErrUnsupported)
	}
	heldParts, err := spiralPlacedHeldTools(ctx, tools, moves)
	if err != nil {
		return nil, err
	}
	if !spiralToolSectorsSeparate(ctx, heldParts, moves, true) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, fmt.Errorf("%w: spiral held tooth shells are not proven disjoint", ErrUnsupported)
	}
	return &spiralCombineSource{first: first, tools: tools, moves: moves}, nil
}

// spiralPlacedHeldTools bounds each copy's held shell against the exact
// recorded placement of the first held shell. The intrinsic trim-to-mesh
// displacement belongs to the source certificate and is not charged here.
func spiralPlacedHeldTools(ctx context.Context, tools []*Body, moves []r3.Transform) ([]*Body, error) {
	if len(tools) < 2 || len(tools) != len(moves) || moves[0] != r3.Identity() {
		return nil, fmt.Errorf("%w: spiral held copies lack their source placement", ErrUnsupported)
	}
	first, ok := tools[0].payload.(facetedPayload)
	if !ok || len(first.verts) == 0 || len(first.tris) == 0 || first.loftFacesTrim == nil {
		return nil, fmt.Errorf("%w: spiral held source is incomplete", ErrUnsupported)
	}
	parts := make([]*Body, 0, len(tools))
	parts = append(parts, &Body{payload: facetedPayload{
		verts: first.verts, tris: first.tris,
		vertexBound: make([]float64, len(first.verts)),
	}})
	for i := 1; i < len(tools); i++ {
		placed, ok := tools[i].payload.(facetedPayload)
		if !ok || placed.spiralTooth == nil || placed.spiralTooth.source != first.loftFacesTrim ||
			placed.spiralTooth.motion != moves[i] || placed.xform != moves[i] ||
			len(placed.verts) != len(first.verts) || len(placed.tris) != len(first.tris) {
			return nil, fmt.Errorf("%w: spiral held copy %d lost source correspondence", ErrUnsupported, i+1)
		}
		for j, tri := range placed.tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if tri != first.tris[j] {
				return nil, fmt.Errorf("%w: spiral held copy %d changed source facets", ErrUnsupported, i+1)
			}
		}
		placementBound := make([]float64, len(placed.verts))
		for j, v := range placed.verts {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			exact := meshbool.ExactRigidImage(moves[i].Basis(), moves[i].Translation(), first.verts[j])
			placementBound[j] = proofbound.Radius3D(
				proofbound.RatFloatUp(meshbool.CoordDistance(exact, v)))
			if proofbound.IsNonFinite(placementBound[j]) {
				return nil, fmt.Errorf("%w: spiral held copy %d has no finite placement bound", ErrUnsupported, i+1)
			}
		}
		bound := facetproof.FacetBoundMax(placed.tris, placementBound)
		if proofbound.IsNonFinite(bound) {
			return nil, fmt.Errorf("%w: spiral held copy %d has no finite shell bound", ErrUnsupported, i+1)
		}
		parts = append(parts, &Body{payload: facetedPayload{
			verts: placed.verts, tris: placed.tris,
			vertexBound: placementBound, meshBound: bound,
		}})
	}
	return parts, nil
}

// spiralToolSectorsSeparate proves source-solid or held-shell separation from
// certified mesh displacements. A solid lies inside the convex hull of its
// boundary, so separation along one YZ direction proves disjointness.
func spiralToolSectorsSeparate(ctx context.Context, tools []*Body, moves []r3.Transform, held bool) bool {
	if len(tools) != len(moves) {
		return false
	}
	maxHeld := 0.0
	for _, tool := range tools {
		part, ok := tool.payload.(facetedPayload)
		if !ok || proofbound.IsNonFinite(part.meshBound) || part.meshBound < 0 {
			return false
		}
		if held {
			maxHeld = max(maxHeld, part.meshBound)
		}
	}
	// Each angular offset gets at most one exact source-slice certificate.
	// Float rotations at different stations are compared to that reference
	// through the exact recorded matrices, never assumed to compose exactly.
	referenceClearance := make(map[int]float64)
	for i := range tools {
		for j := i + 1; j < len(tools); j++ {
			if err := ctx.Err(); err != nil {
				return false
			}
			pi, okI := tools[i].payload.(facetedPayload)
			pj, okJ := tools[j].payload.(facetedPayload)
			if !okI || !okJ || proofbound.IsNonFinite(pi.meshBound) ||
				proofbound.IsNonFinite(pj.meshBound) || pi.meshBound < 0 || pj.meshBound < 0 {
				return false
			}
			heldAllowance := 0.0
			if held {
				heldAllowance = proofbound.AbsSumUpper(pi.meshBound, pj.meshBound)
			}
			angleOffset := j - i
			if gap := referenceClearance[angleOffset]; gap > 0 &&
				spiralTransferSectorSeparation(tools[0], moves, i, j,
					angleOffset, gap, heldAllowance) {
				continue
			}
			ci := moves[i].ApplyDir(r3.Vec{Y: 1})
			cj := moves[j].ApplyDir(r3.Vec{Y: 1})
			ny, nz := lfSub(lfF(cj.Y), lfF(ci.Y)), lfSub(lfF(cj.Z), lfF(ci.Z))
			if ny.Sign() == 0 && nz.Sign() == 0 {
				return false
			}
			norm := proofbound.RatSqrtUp(lfAdd(lfMul(ny, ny), lfMul(nz, nz)))
			if proofbound.IsNonFinite(norm) || norm <= 0 {
				return false
			}
			loI, hiI, okI := facetedToolProjection(tools[i], ny, nz, norm)
			loJ, hiJ, okJ := facetedToolProjection(tools[j], ny, nz, norm)
			if !okI || !okJ {
				return false
			}
			if hiI.Cmp(loJ) < 0 || hiJ.Cmp(loI) < 0 {
				continue
			}
			if len(tools) > 2 {
				gap := referenceClearance[angleOffset]
				if gap == 0 {
					axialExtra, valid := spiralSourceAxialDrift(tools[0], moves, angleOffset)
					if !valid {
						return false
					}
					// A held target point may be displaced in X from its source.
					// Include that bound before transferring this offset's slice gap.
					if held {
						axialExtra = proofbound.AbsSumUpper(axialExtra, maxHeld)
					}
					var ok bool
					gap, ok = spiralToolSlabsClearance(ctx, tools[0], tools[angleOffset], axialExtra)
					if !ok || gap <= 0 {
						return false
					}
					referenceClearance[angleOffset] = gap
				}
				if spiralTransferSectorSeparation(tools[0], moves, i, j,
					angleOffset, gap, heldAllowance) {
					continue
				}
			}
			if _, ok := spiralToolSlabsClearance(ctx, tools[i], tools[j], 0); !ok {
				return false
			}
		}
	}
	return true
}

// A target pair at one X maps back to two reference points at most this far
// from that X. The first displacement compares Mi with its exact-X version;
// the second compares Mj with that version of Mi composed with Mk.
func spiralSourceAxialDrift(first *Body, moves []r3.Transform, referenceIndex int) (float64, bool) {
	p, ok := first.payload.(facetedPayload)
	if !ok || len(p.verts) == 0 || referenceIndex <= 0 || referenceIndex >= len(moves) ||
		proofbound.IsNonFinite(p.meshBound) || p.meshBound < 0 {
		return 0, false
	}
	xReach := 0.0
	for _, v := range p.verts {
		if !proofbound.FiniteVec(v) {
			return 0, false
		}
		xReach = max(xReach, math.Abs(v.X))
	}
	xReach = proofbound.AbsSumUpper(xReach, p.meshBound)
	firstScale := lfF(moves[referenceIndex].Basis().EX.X)
	defect := new(big.Rat)
	for _, move := range moves {
		scale := lfF(move.Basis().EX.X)
		for _, reference := range []*big.Rat{lfOne(), firstScale} {
			gap := lfAbs(lfSub(scale, reference))
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
func spiralTransferSectorSeparation(first *Body, moves []r3.Transform,
	i, j, referenceIndex int, clearance, heldAllowance float64) bool {
	if first == nil || len(moves) < 2 || i == j || i < 0 || j < 0 ||
		referenceIndex <= 0 || referenceIndex >= len(moves) ||
		i >= len(moves) || j >= len(moves) || clearance <= 0 ||
		proofbound.IsNonFinite(clearance) || proofbound.IsNonFinite(heldAllowance) || heldAllowance < 0 {
		return false
	}
	p, ok := first.payload.(facetedPayload)
	if !ok || len(p.verts) == 0 || proofbound.IsNonFinite(p.meshBound) {
		return false
	}
	maxAbs := 0.0
	for _, v := range p.verts {
		maxAbs = max(maxAbs, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	radius := proofbound.AbsSumUpper(proofbound.Radius3D(maxAbs), p.meshBound)
	mi, errI := massmoment.PlacementRotation(moves[i])
	m1, err1 := massmoment.PlacementRotation(moves[referenceIndex])
	mj, errJ := massmoment.PlacementRotation(moves[j])
	if errI != nil || err1 != nil || errJ != nil {
		return false
	}
	// Replace only the left copy's one-ULP X scale. This exact-X map carries
	// the reference's X slabs unchanged; spiralSourceAxialDrift pays for the
	// difference to the actual placed copy in the enlarged source slices.
	mi[0][0] = lfOne()
	charge, err := massmoment.MapChargeOf(mi)
	if err != nil || charge.Stretch >= 1 {
		return false
	}
	deviation := new(big.Rat)
	for row := range 3 {
		for column := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				entry.Add(entry, lfMul(mi[row][k], m1[k][column]))
			}
			deviation.Add(deviation, lfAbs(lfSub(entry, mj[row][column])))
		}
	}
	allowance := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.RatFloatUp(deviation), radius), heldAllowance)
	return lfSub(lfMul(lfSub(lfOne(), lfF(charge.Stretch)), lfF(clearance)), lfF(allowance)).Sign() > 0
}

// A spiral tooth changes angle across its axial span. Whole-body angular
// projections can overlap even when every corresponding X slice is separate.
// Every true boundary point at X in one slab is within meshBound of a held
// triangle whose X range intersects that slab enlarged by meshBound. The
// projected extrema of those triangles, enlarged by meshBound again in the
// tested YZ direction, enclose that true slice. A bounded solid's occupied
// slice lies inside the convex hull of its boundary slice.
func spiralToolSlabsClearance(ctx context.Context, a, b *Body, axialExtra float64) (float64, bool) {
	pa, okA := a.payload.(facetedPayload)
	pb, okB := b.payload.(facetedPayload)
	if !okA || !okB || len(pa.tris) == 0 || len(pb.tris) == 0 ||
		proofbound.IsNonFinite(axialExtra) || axialExtra < 0 {
		return 0, false
	}
	clearance := math.Inf(1)
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, part := range []facetedPayload{pa, pb} {
		if proofbound.IsNonFinite(part.meshBound) || part.meshBound < 0 {
			return 0, false
		}
		reach := part.meshBound
		if axialExtra > 0 {
			reach = proofbound.AbsSumUpper(reach, axialExtra)
		}
		for _, v := range part.verts {
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
		fa, ca, okA := spiralSlabFacets(pa, levels[i], levels[i+1], axialExtra)
		fb, cb, okB := spiralSlabFacets(pb, levels[i], levels[i+1], axialExtra)
		if len(fa) == 0 || len(fb) == 0 {
			continue
		}
		if !okA || !okB {
			return 0, false
		}
		direction := cb.Sub(ca)
		ny, nz := lfF(direction.Y), lfF(direction.Z)
		if ny.Sign() == 0 && nz.Sign() == 0 {
			return 0, false
		}
		norm := proofbound.RatSqrtUp(lfAdd(lfMul(ny, ny), lfMul(nz, nz)))
		if proofbound.IsNonFinite(norm) || norm <= 0 {
			return 0, false
		}
		loA, hiA := spiralSlabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		loB, hiB := spiralSlabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		separate := loA == nil || loB == nil || hiA.Cmp(loB) < 0 || hiB.Cmp(loA) < 0
		if !separate {
			for _, axis := range spiralSlabHullAxes(pa, fa, pb, fb) {
				if err := ctx.Err(); err != nil {
					return 0, false
				}
				ny, nz := lfF(axis.Y), lfF(axis.Z)
				norm := proofbound.RatSqrtUp(lfAdd(lfMul(ny, ny), lfMul(nz, nz)))
				if proofbound.IsNonFinite(norm) || norm <= 0 {
					continue
				}
				loA, hiA = spiralSlabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
				loB, hiB = spiralSlabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
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
				gap = lfSub(loB, hiA)
			} else if hiB.Cmp(loA) < 0 {
				gap = lfSub(loA, hiB)
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
func spiralSlabHullAxes(a facetedPayload, fa []int, b facetedPayload, fb []int) []r3.Vec {
	type point struct{ y, z float64 }
	points := func(p facetedPayload, facets []int) []point {
		out := make([]point, 0, 3*len(facets))
		for _, i := range facets {
			for _, vi := range p.tris[i] {
				v := p.verts[vi]
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
		p facetedPayload
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

func spiralSlabFacets(p facetedPayload, lo, hi, axialExtra float64) ([]int, r3.Vec, bool) {
	reach := p.meshBound
	if axialExtra > 0 {
		reach = proofbound.AbsSumUpper(reach, axialExtra)
	}
	searchLo := math.Nextafter(lo-reach, math.Inf(-1))
	searchHi := math.Nextafter(hi+reach, math.Inf(1))
	selected := make([]int, 0, len(p.tris)/8)
	center := r3.Vec{}
	for i, tri := range p.tris {
		a, b, c := p.verts[tri[0]], p.verts[tri[1]], p.verts[tri[2]]
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

func spiralSlabProjection(p facetedPayload, facets []int, slabLo, slabHi float64, ny, nz *big.Rat,
	norm, axialExtra float64) (*big.Rat, *big.Rat) {
	var lo, hi *big.Rat
	if len(p.vertexBound) != len(p.verts) {
		return nil, nil
	}
	include := func(value, charge *big.Rat) {
		lower, upper := lfSub(value, charge), lfAdd(value, charge)
		if lo == nil || lower.Cmp(lo) < 0 {
			lo = lower
		}
		if hi == nil || upper.Cmp(hi) > 0 {
			hi = upper
		}
	}
	for _, i := range facets {
		tri := p.tris[i]
		beta := max(p.vertexBound[tri[0]], p.vertexBound[tri[1]], p.vertexBound[tri[2]])
		if axialExtra > 0 {
			beta = proofbound.AbsSumUpper(beta, axialExtra)
		}
		charge := lfF(proofbound.ProductUpper(norm, beta))
		xLo := lfF(math.Nextafter(slabLo-beta, math.Inf(-1)))
		xHi := lfF(math.Nextafter(slabHi+beta, math.Inf(1)))
		var x, value [3]*big.Rat
		for k, vi := range tri {
			v := p.verts[vi]
			x[k] = lfF(v.X)
			value[k] = lfAdd(lfMul(ny, lfF(v.Y)), lfMul(nz, lfF(v.Z)))
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
					t := new(big.Rat).Quo(lfSub(limit, x[k]), lfSub(x[next], x[k]))
					include(lfAdd(value[k], lfMul(t, lfSub(value[next], value[k]))), charge)
				}
			}
		}
	}
	return lo, hi
}

func facetedToolProjection(body *Body, ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	p, ok := body.payload.(facetedPayload)
	if !ok || len(p.verts) == 0 || proofbound.IsNonFinite(p.meshBound) || p.meshBound < 0 {
		return nil, nil, false
	}
	var lo, hi *big.Rat
	for _, v := range p.verts {
		if !proofbound.FiniteVec(v) {
			return nil, nil, false
		}
		value := lfAdd(lfMul(ny, lfF(v.Y)), lfMul(nz, lfF(v.Z)))
		if lo == nil || value.Cmp(lo) < 0 {
			lo = value
		}
		if hi == nil || value.Cmp(hi) > 0 {
			hi = value
		}
	}
	chargeBound := proofbound.ProductUpper(norm, p.meshBound)
	if proofbound.IsNonFinite(chargeBound) {
		return nil, nil, false
	}
	charge := lfF(chargeBound)
	return lfSub(lo, charge), lfAdd(hi, charge), true
}
