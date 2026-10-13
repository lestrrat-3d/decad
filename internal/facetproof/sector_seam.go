package facetproof

import (
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// AngleAt returns the angle of p around the +X axis in [start, start+2π).
func AngleAt(p r3.Vec, start float64) float64 {
	angle := math.Atan2(p.Z, p.Y)
	for angle < start {
		angle += 2 * math.Pi
	}
	for angle >= start+2*math.Pi {
		angle -= 2 * math.Pi
	}
	return angle
}

type sectorInterval struct{ lo, hi float64 }

// SectorSeam returns a seam angle for the sector loop whose first and last
// root contacts are first and last, placed by each of motions.
//
// The largest unused gap supplies a seam outside every certified root
// sector. The later spatial-error test proves these held intervals disjoint.
func SectorSeam(first, last r3.Vec, motions []r3.Transform) (float64, error) {
	if len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", decaderr.ErrUnsupported)
	}
	intervals := make([]sectorInterval, len(motions))
	for i, motion := range motions {
		a := math.Atan2(motion.Apply(first).Z, motion.Apply(first).Y)
		b := math.Atan2(motion.Apply(last).Z,
			motion.Apply(last).Y)
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
		intervals[i] = sectorInterval{lo: a, hi: b}
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
		return 0, fmt.Errorf("%w: sector intervals cover the full root", decaderr.ErrUnsupported)
	}
	if seam >= 2*math.Pi {
		seam -= 2 * math.Pi
	}
	return seam, nil
}

// SectorAngularGap returns the rounded-down angular gap between two sector
// angle ranges a and b, or -Inf when they overlap.
func SectorAngularGap(a, b [2]float64) float64 {
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
