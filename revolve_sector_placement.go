package decad

import (
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetproof"
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

// revolveSectorSourcesSeparate checks each later sector's recorded axis
// placement, charges its mesh bound for that placement, and asks
// facetproof.SectorSourcesSeparate for whole-source separation.
func revolveSectorSourcesSeparate(sectors []facetedPayload) bool {
	sources := make([]facetproof.SectorSource, len(sectors))
	for i, sector := range sectors {
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
		sources[i] = facetproof.SectorSource{Verts: sector.verts, MeshBound: sector.meshBound,
			Bound: revolvePlacedBound(sector.meshBound, charge)}
	}
	return facetproof.SectorSourcesSeparate(sources)
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
