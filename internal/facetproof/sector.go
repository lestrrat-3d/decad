package facetproof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SectorSource is one closed revolve sector for SectorSourcesSeparate: its
// held boundary vertices, the payload's certified mesh bound, and that bound
// charged for the sector's recorded placement.
type SectorSource struct {
	Verts     []r3.Vec
	MeshBound float64
	Bound     float64
}

func ratF(v float64) *big.Rat { return proof.FloatRat(v) }

func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

// SectorSourcesSeparate reports whether every point in each closed source
// solid lies within its boundary coordinate ranges. A disjoint coordinate or YZ
// projection after the certified mesh bounds are charged proves whole-source
// separation, including the interior.
func SectorSourcesSeparate(sectors []SectorSource) bool {
	return sectorSourcesSeparate(sectors, func(a, b SectorSource, aLo, aHi, bLo, bHi r3.Vec) bool {
		return sectorPairProjectionSeparate(a.Verts, b.Verts, a.Bound, b.Bound, aLo, aHi, bLo, bHi)
	})
}

// SectorSourcesSeparateAbout is SectorSourcesSeparate for sectors placed about
// the axis vector axis, a finite nonzero vector of any length. A pair whose
// boxes overlap is tested along the exact box-centre difference minus its
// axis component, n = (A·A)·d − (d·A)·A, in place of the YZ direction, which
// is weak off the X axis (agent-docs/revolve-axis-design.md §5.8). The
// direction only selects the test: exact projections of every held vertex,
// widened by |n|↑ times each bound, supply the proof.
func SectorSourcesSeparateAbout(sectors []SectorSource, axis r3.Vec) bool {
	a, ok := exactVec(axis)
	if !ok {
		return false
	}
	return sectorSourcesSeparate(sectors, func(p, q SectorSource, pLo, pHi, qLo, qHi r3.Vec) bool {
		return sectorPairSeparateAbout(p, q, pLo, pHi, qLo, qHi, a)
	})
}

func sectorSourcesSeparate(sectors []SectorSource,
	pairSeparate func(a, b SectorSource, aLo, aHi, bLo, bHi r3.Vec) bool) bool {
	type box struct{ lo, hi r3.Vec }
	boxes := make([]box, len(sectors))
	for i, sector := range sectors {
		if len(sector.Verts) == 0 || sector.MeshBound < 0 ||
			proofbound.IsNonFinite(sector.MeshBound) {
			return false
		}
		bound := sector.Bound
		if proofbound.IsNonFinite(bound) {
			return false
		}
		lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
		for _, p := range sector.Verts {
			if !proofbound.FiniteVec(p) {
				return false
			}
			lo.X, lo.Y, lo.Z = min(lo.X, p.X), min(lo.Y, p.Y), min(lo.Z, p.Z)
			hi.X, hi.Y, hi.Z = max(hi.X, p.X), max(hi.Y, p.Y), max(hi.Z, p.Z)
		}
		boxes[i] = box{lo: r3.Vec{
			X: math.Nextafter(lo.X-bound, math.Inf(-1)),
			Y: math.Nextafter(lo.Y-bound, math.Inf(-1)),
			Z: math.Nextafter(lo.Z-bound, math.Inf(-1)),
		}, hi: r3.Vec{
			X: math.Nextafter(hi.X+bound, math.Inf(1)),
			Y: math.Nextafter(hi.Y+bound, math.Inf(1)),
			Z: math.Nextafter(hi.Z+bound, math.Inf(1)),
		}}
		if !proofbound.FiniteVec(boxes[i].lo) || !proofbound.FiniteVec(boxes[i].hi) {
			return false
		}
	}
	for i := range boxes {
		for j := i + 1; j < len(boxes); j++ {
			a, b := boxes[i], boxes[j]
			if a.hi.X < b.lo.X || b.hi.X < a.lo.X ||
				a.hi.Y < b.lo.Y || b.hi.Y < a.lo.Y ||
				a.hi.Z < b.lo.Z || b.hi.Z < a.lo.Z {
				continue
			}
			if !pairSeparate(sectors[i], sectors[j], a.lo, a.hi, b.lo, b.hi) {
				return false
			}
		}
	}
	return true
}

// exactVec reads v's components as rationals, reporting false for a
// non-finite or zero vector.
func exactVec(v r3.Vec) ([3]*big.Rat, bool) {
	if !proofbound.FiniteVec(v) || v == (r3.Vec{}) {
		return [3]*big.Rat{}, false
	}
	return [3]*big.Rat{ratF(v.X), ratF(v.Y), ratF(v.Z)}, true
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	return ratAdd(ratAdd(ratMul(a[0], b[0]), ratMul(a[1], b[1])), ratMul(a[2], b[2]))
}

// sectorPairSeparateAbout separates two sources along the box-centre
// difference d (doubled, which keeps it exact) with its component along the
// exact axis a removed: n = (a·a)·d − (d·a)·a, so n·a = 0 exactly.
func sectorPairSeparateAbout(a, b SectorSource, aLo, aHi, bLo, bHi r3.Vec, axis [3]*big.Rat) bool {
	var d [3]*big.Rat
	for i, pair := range [3][4]float64{
		{aLo.X, aHi.X, bLo.X, bHi.X}, {aLo.Y, aHi.Y, bLo.Y, bHi.Y}, {aLo.Z, aHi.Z, bLo.Z, bHi.Z},
	} {
		d[i] = ratSub(ratAdd(ratF(pair[0]), ratF(pair[1])), ratAdd(ratF(pair[2]), ratF(pair[3])))
	}
	aa, da := ratDot(axis, axis), ratDot(d, axis)
	var n [3]*big.Rat
	for i := range n {
		n[i] = ratSub(ratMul(aa, d[i]), ratMul(da, axis[i]))
	}
	normSq := ratDot(n, n)
	if normSq.Sign() == 0 {
		return false
	}
	norm := proofbound.RatSqrtUp(normSq)
	if proofbound.IsNonFinite(norm) || norm <= 0 {
		return false
	}
	aMin, aMax, ok := sectorProjectionAlong(a.Verts, a.Bound, n, norm)
	if !ok {
		return false
	}
	bMin, bMax, ok := sectorProjectionAlong(b.Verts, b.Bound, n, norm)
	return ok && (aMax.Cmp(bMin) < 0 || bMax.Cmp(aMin) < 0)
}

// sectorProjectionAlong returns the exact range of n·p over verts, widened on
// each side by norm·bound rounded up, where norm ≥ |n|.
func sectorProjectionAlong(verts []r3.Vec, bound float64, n [3]*big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	var lo, hi *big.Rat
	for _, vertex := range verts {
		value := ratDot(n, [3]*big.Rat{ratF(vertex.X), ratF(vertex.Y), ratF(vertex.Z)})
		if lo == nil || value.Cmp(lo) < 0 {
			lo = value
		}
		if hi == nil || value.Cmp(hi) > 0 {
			hi = value
		}
	}
	charge := proofbound.ProductUpper(norm, bound)
	if lo == nil || proofbound.IsNonFinite(charge) {
		return nil, nil, false
	}
	return ratSub(lo, ratF(charge)), ratAdd(hi, ratF(charge)), true
}

// The difference between the held box centers chooses a test direction; it
// does not certify anything by itself. Exact rational projections of all held
// vertices, enlarged by the spatial and placement bounds, supply the proof.
func sectorPairProjectionSeparate(a, b []r3.Vec, aBound, bBound float64,
	aLo, aHi, bLo, bHi r3.Vec) bool {
	if sectorPairProjectionSeparateFloat(a, b, aBound, bBound, aLo, aHi, bLo, bHi) {
		return true
	}
	ny := ratSub(ratAdd(ratF(aLo.Y), ratF(aHi.Y)), ratAdd(ratF(bLo.Y), ratF(bHi.Y)))
	nz := ratSub(ratAdd(ratF(aLo.Z), ratF(aHi.Z)), ratAdd(ratF(bLo.Z), ratF(bHi.Z)))
	if ny.Sign() == 0 && nz.Sign() == 0 {
		return false
	}
	norm := proofbound.RatSqrtUp(ratAdd(ratMul(ny, ny), ratMul(nz, nz)))
	if proofbound.IsNonFinite(norm) || norm <= 0 {
		return false
	}
	aMin, aMax, ok := sectorProjection(a, aBound, ny, nz, norm)
	if !ok {
		return false
	}
	bMin, bMax, ok := sectorProjection(b, bBound, ny, nz, norm)
	return ok && (aMax.Cmp(bMin) < 0 || bMax.Cmp(aMin) < 0)
}

// sectorPairProjectionSeparateFloat is a fast sufficient proof before the
// exact projection check. It uses the float box-center direction as any
// separating direction, then encloses each dot product by 1e-12 of its
// absolute product sum, which exceeds the accumulated normal rounding error.
// The 1e-300 term covers subnormal rounding. The exact proof remains
// responsible whenever the intervals overlap or any calculation is non-finite.
func sectorPairProjectionSeparateFloat(a, b []r3.Vec, aBound, bBound float64,
	aLo, aHi, bLo, bHi r3.Vec) bool {
	ny, nz, norm, ok := sectorProjectionAxisFloat(aLo, aHi, bLo, bHi)
	if !ok {
		return false
	}
	aMin, aMax, ok := sectorProjectionFloat(a, ny, nz, norm, aBound)
	if !ok {
		return false
	}
	bMin, bMax, ok := sectorProjectionFloat(b, ny, nz, norm, bBound)
	if !ok {
		return false
	}
	return aMax < bMin || bMax < aMin
}

func sectorProjectionAxisFloat(aLo, aHi, bLo, bHi r3.Vec) (float64, float64, float64, bool) {
	centerY := aLo.Y*0.5 + aHi.Y*0.5 - (bLo.Y*0.5 + bHi.Y*0.5)
	centerZ := aLo.Z*0.5 + aHi.Z*0.5 - (bLo.Z*0.5 + bHi.Z*0.5)
	scale := max(math.Abs(centerY), math.Abs(centerZ))
	if proofbound.IsNonFinite(scale) || scale == 0 {
		return 0, 0, 0, false
	}
	ny, nz := centerY/scale, centerZ/scale
	norm := math.Nextafter(math.Abs(ny)+math.Abs(nz), math.Inf(1))
	if proofbound.IsNonFinite(norm) || norm <= 0 {
		return 0, 0, 0, false
	}
	return ny, nz, norm, true
}

func sectorProjectionFloat(verts []r3.Vec, ny, nz, norm, bound float64) (float64, float64, bool) {
	if bound < 0 {
		return 0, 0, false
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, vertex := range verts {
		py, pz := ny*vertex.Y, nz*vertex.Z
		dot, magnitude := py+pz, math.Abs(py)+math.Abs(pz)
		if proofbound.IsNonFinite(dot) || proofbound.IsNonFinite(magnitude) {
			return 0, 0, false
		}
		err := math.Nextafter(1e-12*magnitude+1e-300, math.Inf(1))
		lower := math.Nextafter(dot-err, math.Inf(-1))
		upper := math.Nextafter(dot+err, math.Inf(1))
		if proofbound.IsNonFinite(lower) || proofbound.IsNonFinite(upper) {
			return 0, 0, false
		}
		lo, hi = min(lo, lower), max(hi, upper)
	}
	charge := math.Nextafter(norm*bound, math.Inf(1))
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || proofbound.IsNonFinite(charge) {
		return 0, 0, false
	}
	lo = math.Nextafter(lo-charge, math.Inf(-1))
	hi = math.Nextafter(hi+charge, math.Inf(1))
	return lo, hi, !proofbound.IsNonFinite(lo) && !proofbound.IsNonFinite(hi)
}

func sectorProjection(verts []r3.Vec, bound float64,
	ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	var lo, hi *big.Rat
	for _, vertex := range verts {
		value := ratAdd(ratMul(ny, ratF(vertex.Y)), ratMul(nz, ratF(vertex.Z)))
		if lo == nil || value.Cmp(lo) < 0 {
			lo = value
		}
		if hi == nil || value.Cmp(hi) > 0 {
			hi = value
		}
	}
	charge := proofbound.ProductUpper(norm, bound)
	if lo == nil || proofbound.IsNonFinite(charge) {
		return nil, nil, false
	}
	return ratSub(lo, ratF(charge)), ratAdd(hi, ratF(charge)), true
}
