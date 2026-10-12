package decad

import (
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// revolveAxisRotation admits only a proper rigid rotation around the
// canonical +X axis used by this Revolve source class. Its held basis error
// is charged by massmoment.MapChargeOf before the placed mesh is used.
func revolveAxisRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EX == (r3.Vec{X: 1}) && b.EY.X == 0 && b.EZ.X == 0 &&
		b.EY.Y == b.EZ.Z && b.EY.Z == -b.EZ.Y
}

func revolvePlacementCharge(t r3.Transform) (massmoment.MapCharge, error) {
	if t == r3.Identity() {
		return massmoment.MapCharge{}, nil
	}
	if !revolveAxisRotation(t) {
		return massmoment.MapCharge{}, fmt.Errorf("%w: sector placement is not an axis rotation", ErrUnsupported)
	}
	rotation, err := massmoment.PlacementRotation(t)
	if err != nil {
		return massmoment.MapCharge{}, err
	}
	return massmoment.MapChargeOf(rotation)
}

func revolvePlacedBound(bound float64, charge massmoment.MapCharge) float64 {
	return proofbound.AbsSumUpper(bound,
		proofbound.ProductUpper(charge.Stretch, bound))
}

// Every point in each closed source solid lies within its boundary coordinate
// ranges. A disjoint coordinate or YZ projection after the certified mesh
// bounds are charged proves whole-source separation, including the interior.
func revolveSectorSourcesSeparate(sectors []facetedPayload) bool {
	type box struct{ lo, hi r3.Vec }
	boxes := make([]box, len(sectors))
	bounds := make([]float64, len(sectors))
	for i, sector := range sectors {
		if len(sector.verts) == 0 || sector.meshBound < 0 ||
			proofbound.IsNonFinite(sector.meshBound) {
			return false
		}
		charge := massmoment.MapCharge{}
		if i != 0 {
			if sector.revolveLoftSector == nil ||
				sector.xform != sector.revolveLoftSector.motion {
				return false
			}
			var err error
			charge, err = revolvePlacementCharge(sector.revolveLoftSector.motion)
			if err != nil {
				return false
			}
		}
		bound := revolvePlacedBound(sector.meshBound, charge)
		if proofbound.IsNonFinite(bound) {
			return false
		}
		bounds[i] = bound
		lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
		for _, p := range sector.verts {
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
			if !revolveSectorPairProjectionSeparate(sectors[i], sectors[j],
				bounds[i], bounds[j], a.lo, a.hi, b.lo, b.hi) {
				return false
			}
		}
	}
	return true
}

// The difference between the held box centers chooses a test direction; it
// does not certify anything by itself. Exact rational projections of all held
// vertices, enlarged by the spatial and placement bounds, supply the proof.
func revolveSectorPairProjectionSeparate(a, b facetedPayload, aBound, bBound float64,
	aLo, aHi, bLo, bHi r3.Vec) bool {
	ny := lfSub(lfAdd(lfF(aLo.Y), lfF(aHi.Y)), lfAdd(lfF(bLo.Y), lfF(bHi.Y)))
	nz := lfSub(lfAdd(lfF(aLo.Z), lfF(aHi.Z)), lfAdd(lfF(bLo.Z), lfF(bHi.Z)))
	if ny.Sign() == 0 && nz.Sign() == 0 {
		return false
	}
	norm := proofbound.RatSqrtUp(lfAdd(lfMul(ny, ny), lfMul(nz, nz)))
	if proofbound.IsNonFinite(norm) || norm <= 0 {
		return false
	}
	aMin, aMax, ok := revolveSectorProjection(a, aBound, ny, nz, norm)
	if !ok {
		return false
	}
	bMin, bMax, ok := revolveSectorProjection(b, bBound, ny, nz, norm)
	return ok && (aMax.Cmp(bMin) < 0 || bMax.Cmp(aMin) < 0)
}

func revolveSectorProjection(sector facetedPayload, bound float64,
	ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	var lo, hi *big.Rat
	for _, vertex := range sector.verts {
		value := lfAdd(lfMul(ny, lfF(vertex.Y)), lfMul(nz, lfF(vertex.Z)))
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
	return lfSub(lo, lfF(charge)), lfAdd(hi, lfF(charge)), true
}

func revolveAngleAt(p r3.Vec, start float64) float64 {
	angle := math.Atan2(p.Z, p.Y)
	for angle < start {
		angle += 2 * math.Pi
	}
	for angle >= start+2*math.Pi {
		angle -= 2 * math.Pi
	}
	return angle
}

type revolveSectorInterval struct{ lo, hi float64 }

// The largest unused gap supplies a seam outside every certified root
// sector. The later spatial-error test proves these held intervals disjoint.
func revolveSectorSeam(path []revolveSectorNode, motions []r3.Transform) (float64, error) {
	if len(path) < 2 || len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", ErrUnsupported)
	}
	intervals := make([]revolveSectorInterval, len(motions))
	for i, motion := range motions {
		a := math.Atan2(motion.Apply(path[0].point).Z, motion.Apply(path[0].point).Y)
		b := math.Atan2(motion.Apply(path[len(path)-1].point).Z,
			motion.Apply(path[len(path)-1].point).Y)
		if a < 0 {
			a += 2 * math.Pi
		}
		if b < 0 {
			b += 2 * math.Pi
		}
		if a > b {
			a, b = b, a
		}
		if b-a >= math.Pi {
			a, b = b, a+2*math.Pi
		}
		intervals[i] = revolveSectorInterval{lo: a, hi: b}
	}
	sort.Slice(intervals, func(i, j int) bool { return intervals[i].lo < intervals[j].lo })
	best, seam := 0.0, 0.0
	for i := range intervals {
		j := (i + 1) % len(intervals)
		end := intervals[j].lo
		if j == 0 {
			end += 2 * math.Pi
		}
		gap := end - intervals[i].hi
		if gap > best {
			best, seam = gap, intervals[i].hi+gap/2
		}
	}
	if best <= 0 {
		return 0, fmt.Errorf("%w: sector intervals cover the full root", ErrUnsupported)
	}
	if seam >= 2*math.Pi {
		seam -= 2 * math.Pi
	}
	return seam, nil
}

func revolveSectorAngularGap(a, b [2]float64) float64 {
	if b[0] < a[0] {
		a, b = b, a
	}
	if a[1] >= b[0] {
		return math.Inf(-1)
	}
	direct := new(big.Rat).Sub(proofarith.FloatRat(b[0]), proofarith.FloatRat(a[1]))
	wrap := new(big.Rat).Add(proofbound.TwoPiInterval().Lo, proofarith.FloatRat(a[0]))
	wrap.Sub(wrap, proofarith.FloatRat(b[1]))
	return min(proofbound.RatFloatDown(direct), proofbound.RatFloatDown(wrap))
}
