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

// AngleBasis measures the angle of a point about a sector join's axis. The
// zero value is the +X convention, atan2(Z, Y). A General basis reads
// atan2(p·E1, p·E0) in the blank's own held frame, so its branch cut lies
// along −E0 (agent-docs/revolve-axis-design.md §3.7). The held frame departs
// from an exact frame about the axis; the sector plan charges that departure
// (§5.6), so the angles here are construction quantities only.
type AngleBasis struct {
	General bool
	E0, E1  r3.Vec
}

// Angle returns the angle of p about the basis's axis in [−π, π].
func (b AngleBasis) Angle(p r3.Vec) float64 {
	if !b.General {
		return math.Atan2(p.Z, p.Y)
	}
	return math.Atan2(p.Dot(b.E1), p.Dot(b.E0))
}

// AngleAt returns the angle of p about the basis's axis in
// [start, start+2π).
func (b AngleBasis) AngleAt(p r3.Vec, start float64) float64 {
	return wrapAngle(b.Angle(p), start)
}

// AngleAt returns the angle of p around the +X axis in [start, start+2π).
func AngleAt(p r3.Vec, start float64) float64 {
	return AngleBasis{}.AngleAt(p, start)
}

func wrapAngle(angle, start float64) float64 {
	for angle < start {
		angle += 2 * math.Pi
	}
	for angle >= start+2*math.Pi {
		angle -= 2 * math.Pi
	}
	return angle
}

type sectorInterval struct{ lo, hi float64 }

// SectorSeam returns a seam angle about the +X axis for the sector loop whose
// first and last root contacts are first and last, placed by each of motions.
func SectorSeam(first, last r3.Vec, motions []r3.Transform) (float64, error) {
	return AngleBasis{}.SectorSeam(first, last, motions)
}

// SectorSeam returns a seam angle about the basis's axis for the sector loop
// whose first and last root contacts are first and last, placed by each of
// motions.
//
// The largest unused gap supplies a seam outside every certified root
// sector. The later spatial-error test proves these held intervals disjoint.
func (basis AngleBasis) SectorSeam(first, last r3.Vec, motions []r3.Transform) (float64, error) {
	if len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", decaderr.ErrUnsupported)
	}
	intervals := make([]sectorInterval, len(motions))
	for i, motion := range motions {
		a := basis.Angle(motion.Apply(first))
		b := basis.Angle(motion.Apply(last))
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
