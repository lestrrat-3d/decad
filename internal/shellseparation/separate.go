package shellseparation

import (
	"context"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Separate proves pairwise source-solid or held-shell separation. Axis selects
// the longitudinal Cartesian coordinate. Linked must be true only after the
// caller verifies that every shell is an image of the first under its recorded
// move; it permits a charged reference-gap transfer for repeated copies.
//
// maps holds each copy's true linear map where it differs from the float move.
// A nil maps, or a nil entry, means PlacementRotation of that move. A non-nil
// entry is an exactly orthogonal rational map the copy's true solid is the image
// under. The caller must pass linked = false unless every such copy's true
// translation is also exactly zero, because transfer composes no translation.
// Transfer through canonicalized axes composes float moves, so any exact entry
// switches it off there and the direct slab checks decide.
func Separate(ctx context.Context, shells []Shell, moves []r3.Transform, maps [][3][3]*big.Rat,
	kind Kind, axis Axis, linked bool) bool {
	if ctx == nil || len(shells) < 2 || len(shells) != len(moves) ||
		(maps != nil && len(maps) != len(moves)) {
		return false
	}
	for _, m := range maps {
		if !validMap(m) {
			return false
		}
	}
	if kind != SourceShell && kind != HeldCopyShell && kind != ProxyShell {
		return false
	}
	for _, part := range shells {
		if !validShell(part) {
			return false
		}
	}
	for _, move := range moves {
		if !move.IsValid() {
			return false
		}
	}
	if axis != AxisX {
		for _, m := range maps {
			if m[0][0] != nil {
				linked = false
			}
		}
		var ok bool
		shells, moves, ok = canonicalize(shells, moves, axis)
		if !ok {
			return false
		}
	}
	return separateCanonical(ctx, shells, moves, maps, kind != SourceShell,
		linked && transferableMoves(moves, maps))
}

// validMap accepts the unset map, or a map whose nine entries are all set.
func validMap(m [3][3]*big.Rat) bool {
	set := 0
	for _, row := range m {
		for _, entry := range row {
			if entry != nil {
				set++
			}
		}
	}
	return set == 0 || set == 9
}

// moveMap is copy i's true linear map: the exact entry of maps when set, else
// the exact reading of the float move's basis.
func moveMap(moves []r3.Transform, maps [][3][3]*big.Rat, i int) ([3][3]*big.Rat, error) {
	if maps != nil && maps[i][0][0] != nil {
		return maps[i], nil
	}
	return massmoment.PlacementRotation(moves[i])
}

func validShell(p Shell) bool {
	if len(p.Verts) == 0 || len(p.Tris) == 0 || len(p.VertexBound) != len(p.Verts) ||
		proofbound.IsNonFinite(p.MeshBound) || p.MeshBound < 0 {
		return false
	}
	for i, v := range p.Verts {
		beta := p.VertexBound[i]
		if !proofbound.FiniteVec(v) || proofbound.IsNonFinite(beta) || beta < 0 || beta > p.MeshBound {
			return false
		}
	}
	for _, tri := range p.Tris {
		for _, index := range tri {
			if index < 0 || index >= len(p.Verts) {
				return false
			}
		}
	}
	return true
}

func canonicalize(shells []Shell, moves []r3.Transform, axis Axis) ([]Shell, []r3.Transform, bool) {
	var u, v r3.Vec
	switch axis {
	case AxisY:
		u, v = r3.Vec{Y: 1}, r3.Vec{Z: 1}
	case AxisZ:
		u, v = r3.Vec{Z: 1}, r3.Vec{X: 1}
	default:
		return nil, nil, false
	}
	frame, err := r3.NewFrame(r3.Vec{}, u, v)
	if err != nil {
		return nil, nil, false
	}
	toWorld, err := r3.FromFrame(frame)
	if err != nil {
		return nil, nil, false
	}
	toLocal, err := toWorld.Inverse()
	if err != nil {
		return nil, nil, false
	}
	localShells := make([]Shell, len(shells))
	localMoves := make([]r3.Transform, len(moves))
	for i, part := range shells {
		localShells[i] = part
		localShells[i].Verts = make([]r3.Vec, len(part.Verts))
		for j, world := range part.Verts {
			localShells[i].Verts[j] = frame.ToLocal(world)
		}
		placed, err := toWorld.Then(moves[i])
		if err != nil {
			return nil, nil, false
		}
		localMoves[i], err = placed.Then(toLocal)
		if err != nil {
			return nil, nil, false
		}
	}
	return localShells, localMoves, true
}

func transferableMoves(moves []r3.Transform, maps [][3][3]*big.Rat) bool {
	if len(moves) < 2 || moves[0] != r3.Identity() {
		return false
	}
	for i, move := range moves[1:] {
		if move.Translation() != (r3.Vec{}) || move.IsReflection() {
			return false
		}
		if maps != nil && maps[i+1][0][0] != nil {
			if !exactXRotation(maps[i+1]) {
				return false
			}
			continue
		}
		basis := move.Basis()
		if basis.EX.X < math.Nextafter(1, 0) ||
			basis.EX.X > math.Nextafter(1, math.Inf(1)) ||
			basis.EX.Y != 0 || basis.EX.Z != 0 || basis.EY.X != 0 || basis.EZ.X != 0 ||
			basis.EY.Y != basis.EZ.Z || basis.EY.Z != -basis.EZ.Y {
			return false
		}
	}
	return true
}

// exactXRotation reports whether m has the structure of a rotation about X:
// no X cross terms, equal diagonal and opposite off-diagonal Y/Z terms, and an
// X scale of exactly 1. A rotation about X fixes X, so any other X scale is
// malformed.
func exactXRotation(m [3][3]*big.Rat) bool {
	for _, entry := range []*big.Rat{m[0][1], m[0][2], m[1][0], m[2][0]} {
		if entry.Sign() != 0 {
			return false
		}
	}
	return proof.CmpRat(m[1][1], m[2][2]) == 0 &&
		proof.CmpRat(m[1][2], new(big.Rat).Neg(m[2][1])) == 0 &&
		proof.CmpRat(m[0][0], ratOne()) == 0
}

func separateCanonical(ctx context.Context, shells []Shell, moves []r3.Transform,
	maps [][3][3]*big.Rat, held, transfer bool) bool {
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
			if gap := referenceClearance[angleOffset]; transfer && gap > 0 &&
				transferSeparation(shells[0], moves, maps, i, j,
					angleOffset, gap, heldAllowance) {
				continue
			}
			ci := moves[i].ApplyDir(r3.Vec{Y: 1})
			cj := moves[j].ApplyDir(r3.Vec{Y: 1})
			ny, nz := ratSub(ratF(cj.Y), ratF(ci.Y)), ratSub(ratF(cj.Z), ratF(ci.Z))
			if ny.Sign() == 0 && nz.Sign() == 0 {
				// Identical orientations may still have a measured placement gap.
				// The direction is a candidate only; exact projection proves it.
				first, second := shells[i].Verts[0], shells[j].Verts[0]
				ny, nz = ratSub(ratF(second.Y), ratF(first.Y)),
					ratSub(ratF(second.Z), ratF(first.Z))
				if ny.Sign() == 0 && nz.Sign() == 0 {
					ny = ratOne()
				}
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
			if proof.CmpRat(hiI, loJ) < 0 || proof.CmpRat(hiJ, loI) < 0 {
				continue
			}
			if transfer && len(shells) > 2 {
				gap := referenceClearance[angleOffset]
				if gap == 0 {
					axialExtra, valid := sourceAxialDrift(shells[0], moves, maps, angleOffset)
					if valid {
						// A held target point may be displaced in X from its source.
						// Include that bound before transferring this offset's slice gap.
						if held {
							axialExtra = proofbound.AbsSumUpper(axialExtra, maxHeld)
						}
						var referenceOK bool
						gap, referenceOK = slabsClearance(ctx, shells[0], shells[angleOffset], axialExtra)
						if referenceOK && gap > 0 && !proofbound.IsNonFinite(gap) {
							referenceClearance[angleOffset] = gap
						}
					}
				}
				if gap > 0 && !proofbound.IsNonFinite(gap) && transferSeparation(shells[0], moves, maps, i, j,
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
func sourceAxialDrift(p Shell, moves []r3.Transform, maps [][3][3]*big.Rat,
	referenceIndex int) (float64, bool) {
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
	xScale := func(i int) (*big.Rat, bool) {
		m, err := moveMap(moves, maps, i)
		return m[0][0], err == nil
	}
	firstScale, ok := xScale(referenceIndex)
	if !ok {
		return 0, false
	}
	defect := new(big.Rat)
	for i := range moves {
		scale, ok := xScale(i)
		if !ok {
			return 0, false
		}
		for _, reference := range []*big.Rat{ratOne(), firstScale} {
			gap := ratAbs(ratSub(scale, reference))
			if proof.CmpRat(gap, defect) > 0 {
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
func transferSeparation(p Shell, moves []r3.Transform, maps [][3][3]*big.Rat,
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
	mi, errI := moveMap(moves, maps, i)
	m1, err1 := moveMap(moves, maps, referenceIndex)
	mj, errJ := moveMap(moves, maps, j)
	if errI != nil || err1 != nil || errJ != nil {
		return false
	}
	// Replace only the left copy's one-ULP X scale. This exact-X map carries
	// the reference's X slabs unchanged; sourceAxialDrift pays for the
	// difference to the actual placed copy in the enlarged source slices. An
	// exact rotation already has X scale 1, so mi is its own map. mi is an array
	// value, so the replacement never touches the caller's rationals.
	if mi[0][0].Cmp(ratOne()) != 0 {
		mi[0][0] = ratOne()
	}
	charge, err := massmoment.MapChargeOf(mi)
	if err != nil || charge.Stretch >= 1 {
		return false
	}
	deviation := new(big.Rat)
	for row := range 3 {
		for column := range 3 {
			entry := new(big.Rat)
			for k := range 3 {
				proof.AddRat(entry, entry, ratMul(mi[row][k], m1[k][column]))
			}
			proof.AddRat(deviation, deviation, ratAbs(ratSub(entry, mj[row][column])))
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
		if proofbound.IsNonFinite(reach) {
			return 0, false
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
	// A triangle's X range depends on the shell alone, so it is read once here
	// instead of once per slab.
	xa, xb := triangleXRanges(pa), triangleXRanges(pb)
	for i := range slabs {
		if err := ctx.Err(); err != nil {
			return 0, false
		}
		fa, ca, okA := slabFacets(pa, xa, levels[i], levels[i+1], axialExtra)
		fb, cb, okB := slabFacets(pb, xb, levels[i], levels[i+1], axialExtra)
		if !okA || !okB {
			return 0, false
		}
		if len(fa) == 0 || len(fb) == 0 {
			continue
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
		loA, hiA, validA := slabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		loB, hiB, validB := slabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
		if !validA || !validB {
			return 0, false
		}
		separate := loA == nil || loB == nil || proof.CmpRat(hiA, loB) < 0 || proof.CmpRat(hiB, loA) < 0
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
				loA, hiA, validA = slabProjection(pa, fa, levels[i], levels[i+1], ny, nz, norm, axialExtra)
				loB, hiB, validB = slabProjection(pb, fb, levels[i], levels[i+1], ny, nz, norm, axialExtra)
				if !validA || !validB {
					return 0, false
				}
				if loA == nil || loB == nil || proof.CmpRat(hiA, loB) < 0 || proof.CmpRat(hiB, loA) < 0 {
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
			if proof.CmpRat(hiA, loB) < 0 {
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
	if math.IsInf(clearance, 1) {
		// No slab held material from both shells, so there is no finite
		// transverse gap to transfer to another pair.
		return 0, true
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

// xRanges holds each triangle's smallest and largest vertex X, by triangle.
type xRanges struct{ lo, hi []float64 }

func triangleXRanges(p Shell) xRanges {
	r := xRanges{lo: make([]float64, len(p.Tris)), hi: make([]float64, len(p.Tris))}
	for i, tri := range p.Tris {
		a, b, c := p.Verts[tri[0]].X, p.Verts[tri[1]].X, p.Verts[tri[2]].X
		r.lo[i], r.hi[i] = min(a, b, c), max(a, b, c)
	}
	return r
}

// slabFacets selects, in ascending triangle order, the triangles whose X range
// meets the slab enlarged by the shell's reach. x holds p's triangle X ranges
// from triangleXRanges; only selected triangles have their vertices loaded.
func slabFacets(p Shell, x xRanges, lo, hi, axialExtra float64) ([]int, r3.Vec, bool) {
	reach := p.MeshBound
	if axialExtra > 0 {
		reach = proofbound.AbsSumUpper(reach, axialExtra)
	}
	searchLo := math.Nextafter(lo-reach, math.Inf(-1))
	searchHi := math.Nextafter(hi+reach, math.Inf(1))
	if proofbound.IsNonFinite(reach) || proofbound.IsNonFinite(searchLo) ||
		proofbound.IsNonFinite(searchHi) {
		return nil, r3.Vec{}, false
	}
	selected := make([]int, 0, len(p.Tris)/8)
	center := r3.Vec{}
	for i, top := range x.hi {
		if top < searchLo || x.lo[i] > searchHi {
			continue
		}
		tri := p.Tris[i]
		selected = append(selected, i)
		center = center.Add(p.Verts[tri[0]]).Add(p.Verts[tri[1]]).Add(p.Verts[tri[2]])
	}
	if len(selected) == 0 {
		return nil, r3.Vec{}, true
	}
	center = center.Scale(1 / float64(3*len(selected)))
	return selected, center, proofbound.FiniteVec(center)
}

// slabCandidates visits every candidate extreme point of the clipped facets:
// each vertex inside the enlarged slab and each strict crossing of a facet
// edge with an enlarged slab limit. Each test compares exact floats, so it is
// the same test as the exact rational comparison. visit gets the candidate's
// position in facets, the vertex or edge start k, the limit (-1 for a vertex,
// 0 for the lower limit, 1 for the upper), the exact float charge, and an
// outward enclosure of the candidate's value. It returns false for an
// invalid bound, as slabProjection always has.
func slabCandidates(p Shell, facets []int, slabLo, slabHi float64, dir direction,
	norm, axialExtra float64, visit func(f, k, limit int, charge, lo, hi float64)) bool {
	for f, i := range facets {
		tri := p.Tris[i]
		beta := max(p.VertexBound[tri[0]], p.VertexBound[tri[1]], p.VertexBound[tri[2]])
		if axialExtra > 0 {
			beta = proofbound.AbsSumUpper(beta, axialExtra)
		}
		chargeBound := proofbound.ProductUpper(norm, beta)
		lower := math.Nextafter(slabLo-beta, math.Inf(-1))
		upper := math.Nextafter(slabHi+beta, math.Inf(1))
		if proofbound.IsNonFinite(beta) || beta < 0 ||
			proofbound.IsNonFinite(chargeBound) ||
			proofbound.IsNonFinite(lower) || proofbound.IsNonFinite(upper) {
			return false
		}
		var x, vLo, vHi [3]float64
		for k, vi := range tri {
			v := p.Verts[vi]
			x[k] = v.X
			vLo[k], vHi[k] = dir.value(v.Y, v.Z)
			if x[k] >= lower && x[k] <= upper {
				visit(f, k, -1, chargeBound, vLo[k], vHi[k])
			}
		}
		for k := range 3 {
			next := (k + 1) % 3
			if x[k] == x[next] {
				continue
			}
			for li, limit := range [...]float64{lower, upper} {
				if (x[k] < limit && x[next] > limit) || (x[k] > limit && x[next] < limit) {
					lo, hi := crossingEnclosure(x[k], x[next], limit, vLo[k], vHi[k], vLo[next], vHi[next])
					visit(f, k, li, chargeBound, lo, hi)
				}
			}
		}
	}
	return true
}

func slabProjection(p Shell, facets []int, slabLo, slabHi float64, ny, nz *big.Rat,
	norm, axialExtra float64) (*big.Rat, *big.Rat, bool) {
	if len(p.VertexBound) != len(p.Verts) {
		return nil, nil, false
	}
	dir := newDirection(ny, nz)
	// The smallest float upper end of any candidate's lower value, and the
	// greatest float lower end of any candidate's upper value.
	lowCut, highCut := math.Inf(1), math.Inf(-1)
	found := false
	if !slabCandidates(p, facets, slabLo, slabHi, dir, norm, axialExtra,
		func(_, _, _ int, charge, lo, hi float64) {
			found = true
			lowCut = min(lowCut, fSubUp(hi, charge))
			highCut = max(highCut, fAddDown(lo, charge))
		}) {
		return nil, nil, false
	}
	if !found {
		return nil, nil, true
	}
	var lo, hi *big.Rat
	slabCandidates(p, facets, slabLo, slabHi, dir, norm, axialExtra,
		func(f, k, limit int, charge, vlo, vhi float64) {
			needLo := !(fSubDown(vlo, charge) > lowCut)
			needHi := !(fAddUp(vhi, charge) < highCut)
			if !needLo && !needHi {
				return
			}
			value := slabCandidateExact(p, facets[f], k, limit, slabLo, slabHi, dir, axialExtra)
			c := ratF(charge)
			if needLo {
				if lower := ratSub(value, c); lo == nil || proof.CmpRat(lower, lo) < 0 {
					lo = lower
				}
			}
			if needHi {
				if upper := ratAdd(value, c); hi == nil || proof.CmpRat(upper, hi) > 0 {
					hi = upper
				}
			}
		})
	return lo, hi, true
}

// slabCandidateExact is the exact value of one candidate from slabCandidates,
// by the same expressions slabProjection has always used.
func slabCandidateExact(p Shell, i, k, limit int, slabLo, slabHi float64, dir direction,
	axialExtra float64) *big.Rat {
	tri := p.Tris[i]
	vk := p.Verts[tri[k]]
	valueK := dir.exact(vk.Y, vk.Z)
	if limit < 0 {
		return valueK
	}
	beta := max(p.VertexBound[tri[0]], p.VertexBound[tri[1]], p.VertexBound[tri[2]])
	if axialExtra > 0 {
		beta = proofbound.AbsSumUpper(beta, axialExtra)
	}
	edge := math.Nextafter(slabLo-beta, math.Inf(-1))
	if limit == 1 {
		edge = math.Nextafter(slabHi+beta, math.Inf(1))
	}
	vn := p.Verts[tri[(k+1)%3]]
	valueN := dir.exact(vn.Y, vn.Z)
	xk, xn, l := ratF(vk.X), ratF(vn.X), ratF(edge)
	t := proof.QuoRat(new(big.Rat), ratSub(l, xk), ratSub(xn, xk))
	return ratAdd(valueK, ratMul(t, ratSub(valueN, valueK)))
}

func projection(p Shell, ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	if len(p.Verts) == 0 || proofbound.IsNonFinite(p.MeshBound) || p.MeshBound < 0 {
		return nil, nil, false
	}
	// Every vertex carries the same charge, so the float pass filters the
	// projected values themselves (see extremum.go).
	dir := newDirection(ny, nz)
	lowCut, highCut := math.Inf(1), math.Inf(-1)
	for _, v := range p.Verts {
		if !proofbound.FiniteVec(v) {
			return nil, nil, false
		}
		vlo, vhi := dir.value(v.Y, v.Z)
		lowCut, highCut = min(lowCut, vhi), max(highCut, vlo)
	}
	var lo, hi *big.Rat
	for _, v := range p.Verts {
		vlo, vhi := dir.value(v.Y, v.Z)
		needLo, needHi := !(vlo > lowCut), !(vhi < highCut)
		if !needLo && !needHi {
			continue
		}
		value := dir.exact(v.Y, v.Z)
		if needLo && (lo == nil || proof.CmpRat(value, lo) < 0) {
			lo = value
		}
		if needHi && (hi == nil || proof.CmpRat(value, hi) > 0) {
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
