package revolvesampling

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// MeridianMin is the meridian chord minimum: three for a closed circular
// generator, two when both ends lie on the axis, and one otherwise. An
// axis-to-axis circular walk needs an off-axis interior station to sweep a face.
func MeridianMin(w survey2d.SegmentWalk) int {
	if w.Closed {
		return 3
	}
	if w.StartV == 0 && w.EndV == 0 {
		return 2
	}
	return 1
}

// RequireAxisIncidence checks the recorded junctions before sampling. At each
// axis point, a manifold pole has exactly one off-axis walk end and one
// on-axis line end from the same loop. Two junctions at one axis point,
// or two walks of the same kind meeting there, refuse the profile.
// Interior chord stations cannot add incidence because ArcStation refuses
// a station on or across the axis.
func RequireAxisIncidence(resolved []revolveaxis.ResolvedWalks, junctions [][]revolvemesh.RevMeridian) error {
	seen := map[float64]struct{}{}
	for li, js := range junctions {
		n := len(js)
		for k, s := range js {
			if !s.OnAxis {
				continue
			}
			if _, dup := seen[s.Z]; dup {
				return fmt.Errorf(`%w: two recorded boundary junctions meet the revolve axis at the same point, so the swept solid pinches there`, decaderr.ErrDegenerate)
			}
			seen[s.Z] = struct{}{}
			incoming := resolved[li].Kinds[(k+n-1)%n]
			outgoing := resolved[li].Kinds[k]
			if (incoming == revolveaxis.WallAxis) == (outgoing == revolveaxis.WallAxis) {
				return fmt.Errorf(`%w: the recorded boundary meets the revolve axis at a junction with %s, which sweeps no manifold pole`, decaderr.ErrDegenerate, axisIncidenceReason(incoming == revolveaxis.WallAxis))
			}
		}
	}
	return nil
}

func axisIncidenceReason(bothAxis bool) string {
	if bothAxis {
		return "two on-axis segments"
	}
	return "two off-axis segments"
}

// OffAxisWalk returns the first circular generator whose meridian polyline
// contains no off-axis sample. Such a generator sweeps no face at the current
// chord count, so the caller can refine that walk.
func OffAxisWalk(resolved []revolveaxis.ResolvedWalks, samples [][]revolvemesh.RevMeridian) (int, int, bool) {
	for li, r := range resolved {
		offAxis := map[int]bool{}
		for _, s := range samples[li] {
			if !s.OnAxis {
				offAxis[s.Walk] = true
			}
		}
		for k, w := range r.Walks {
			if w.IsCircular() && r.Kinds[k] != revolveaxis.WallAxis && !offAxis[k] {
				return li, k, true
			}
		}
	}
	return 0, 0, false
}

// SectionPoints flattens the meridian polylines in ring allocation order.
// The third result holds each sample's outgoing chord sagitta for the section
// clearance proof.
func SectionPoints(samples [][]revolvemesh.RevMeridian) ([]sectionrecord.Point2, [][]int, [][]float64) {
	var pts []sectionrecord.Point2
	var loopIdx [][]int
	var loopSag [][]float64
	for _, loop := range samples {
		base := len(pts)
		idx := make([]int, len(loop))
		sag := make([]float64, len(loop))
		for k, s := range loop {
			pts = append(pts, sectionrecord.Point2{U: s.Z, V: s.Rho})
			idx[k] = base + k
			sag[k] = s.Sag
		}
		loopIdx = append(loopIdx, idx)
		loopSag = append(loopSag, sag)
	}
	return pts, loopIdx, loopSag
}

// CapSegmentArea sums the absolute circular-segment area between each partial
// cap's arc and its chords. Holes cannot cancel an outer boundary's area slack.
func CapSegmentArea(resolved []revolveaxis.ResolvedWalks, counts [][]int) float64 {
	total := 0.0
	for li, r := range resolved {
		for k, w := range r.Walks {
			if !w.IsCircular() {
				continue
			}
			total = proofbound.AbsSumUpper(total,
				revolvemesh.ChordSegmentArea(w.Radius, math.Abs(w.Th1-w.Th0), counts[li][k]))
		}
	}
	return total
}
