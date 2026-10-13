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
			if !sectorPairProjectionSeparate(sectors[i].Verts, sectors[j].Verts,
				sectors[i].Bound, sectors[j].Bound, a.lo, a.hi, b.lo, b.hi) {
				return false
			}
		}
	}
	return true
}

// The difference between the held box centers chooses a test direction; it
// does not certify anything by itself. Exact rational projections of all held
// vertices, enlarged by the spatial and placement bounds, supply the proof.
func sectorPairProjectionSeparate(a, b []r3.Vec, aBound, bBound float64,
	aLo, aHi, bLo, bHi r3.Vec) bool {
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
