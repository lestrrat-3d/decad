package revolvemesh

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SectorBoreMaxStations caps the angular stations a centered cylinder Cut of
// a Revolve/sector Union accepts.
const SectorBoreMaxStations = 2_304

// SectorCapFan returns an angular cell only for a complete, source-
// labeled axis fan triangle on the retained outer ring.
func SectorCapFan(t [3]int, src, axis, group int,
	positions map[int]int, n int) (int, bool, error) {
	if t[0] != axis && t[1] != axis && t[2] != axis {
		return 0, false, nil
	}
	other := [2]int{}
	k := 0
	for _, v := range t {
		if v == axis {
			continue
		}
		if k >= 2 {
			return 0, false, fmt.Errorf("%w: cap fan repeats its axis", decaderr.ErrUnsupported)
		}
		other[k] = v
		k++
	}
	if k != 2 || src != group {
		return 0, false, fmt.Errorf("%w: cap axis belongs to another face", decaderr.ErrUnsupported)
	}
	a, aOK := positions[other[0]]
	b, bOK := positions[other[1]]
	if !aOK || !bOK {
		return 0, false, fmt.Errorf("%w: cap fan misses its outer ring", decaderr.ErrUnsupported)
	}
	if (a+1)%n == b {
		return a, true, nil
	}
	if (b+1)%n == a {
		return b, true, nil
	}
	return 0, false, fmt.Errorf("%w: cap fan crosses angular stations", decaderr.ErrUnsupported)
}

// SectorRingMaxStep rechecks the retained Union station order and
// encloses the wrap interval using the upper side of the 2π certificate.
func SectorRingMaxStep(angles []float64) (float64, bool) {
	if len(angles) < 16 || len(angles) > SectorBoreMaxStations {
		return 0, false
	}
	maxStep := 0.0
	for i, angle := range angles {
		if proofbound.IsNonFinite(angle) {
			return 0, false
		}
		j := (i + 1) % len(angles)
		var step *big.Rat
		if j == 0 {
			step = new(big.Rat).Add(proofbound.TwoPiInterval().Hi,
				new(big.Rat).Sub(proofarith.FloatRat(angles[0]), proofarith.FloatRat(angle)))
		} else {
			step = new(big.Rat).Sub(proofarith.FloatRat(angles[j]), proofarith.FloatRat(angle))
		}
		upper := proofbound.RatFloatUp(step)
		if step.Sign() <= 0 || proofbound.IsNonFinite(upper) || upper > math.Pi/2 {
			return 0, false
		}
		maxStep = max(maxStep, upper)
	}
	return maxStep, true
}

// SectorBorePatches proposes projected graphs from the completed
// result. The crossing audit independently proves each proposal and audits
// every pair outside a proven patch, including all cross-patch pairs. Every
// patch projects onto the world YZ plane, the plane normal to +X.
func SectorBorePatches(src []int, oldCount, stations int,
	band bool, lowGroup, highGroup, bandGroup, wallGroup int) []loftmesh.ProjectedPatch {
	return SectorBorePatchesOnto(src, oldCount, stations, band, lowGroup, highGroup, bandGroup, wallGroup,
		[2]int{1, 2})
}

// SectorBorePatchesOnto is SectorBorePatches projected onto the coordinate
// plane axes names. A bore about a general axis passes the plane that drops
// the axis's largest component (agent-docs/revolve-axis-design.md §5.8, §9).
func SectorBorePatchesOnto(src []int, oldCount, stations int,
	band bool, lowGroup, highGroup, bandGroup, wallGroup int, axes [2]int) []loftmesh.ProjectedPatch {
	groups := make(map[int][]int)
	for i := range oldCount {
		group := src[i]
		if group == lowGroup || group == highGroup ||
			group == bandGroup || group == wallGroup {
			continue
		}
		groups[group] = append(groups[group], i)
	}
	keys := make([]int, 0, len(groups))
	for group := range groups {
		keys = append(keys, group)
	}
	slices.Sort(keys)
	patches := make([]loftmesh.ProjectedPatch, 0, len(keys)+4)
	for _, group := range keys {
		if len(groups[group]) < 2 {
			continue
		}
		patches = append(patches, loftmesh.ProjectedPatch{
			Triangles: groups[group], Axes: axes,
		})
	}
	// A complete annulus has two boundary loops. Each angular half is a
	// single-boundary patch that the existing projected proof can check.
	for capIndex := range 2 {
		if band && capIndex == 0 {
			continue
		}
		for half := range 2 {
			start, end := half*stations/2, (half+1)*stations/2
			indices := make([]int, 0, 2*(end-start))
			for cell := start; cell < end; cell++ {
				base := oldCount + 6*cell + 2*capIndex
				indices = append(indices, base, base+1)
			}
			patches = append(patches, loftmesh.ProjectedPatch{
				Triangles: indices, Axes: axes,
			})
		}
	}
	return patches
}
