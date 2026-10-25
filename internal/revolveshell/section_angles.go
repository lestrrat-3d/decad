package revolveshell

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RevolveSectionAngles chooses a full-turn plan from the source section's
// largest radius and the caller's circular-chord allowance. The seam lies in
// the unused angular complement of the certified contact sector.
func RevolveSectionAngles(section facetedrecord.RevolveSectionCertificate, sector [2]float64,
	maxSag float64) ([]float64, [2]float64, error) {
	var empty [2]float64
	if len(section.Spans) < 4 || maxSag <= 0 || proofbound.IsNonFinite(maxSag) ||
		!proofbound.FiniteVec(r3.Vec{X: sector[0], Y: sector[1]}) ||
		sector[0] >= sector[1] || sector[1]-sector[0] >= math.Pi {
		return nil, empty, fmt.Errorf("%w: section has no bounded angular plan", decaderr.ErrUnsupported)
	}
	radius := 0.0
	if section.General != nil {
		// A general section reads each radius about the axis, R/|A|↓
		// (agent-docs/revolve-axis-design.md §5.7). Every span end is the
		// next span's start.
		stations, ok := sectionStations(section)
		if !ok {
			return nil, empty, fmt.Errorf("%w: section has no finite radius", decaderr.ErrUnsupported)
		}
		for _, station := range stations {
			radius = max(radius, station.radius)
		}
	} else {
		for _, span := range section.Spans {
			radius = max(radius, span.Start.V, span.End.V)
		}
	}
	if radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, empty, fmt.Errorf("%w: section has no finite radius", decaderr.ErrUnsupported)
	}
	maxStep := math.Sqrt(8 * maxSag / radius)
	if maxStep <= 0 || proofbound.IsNonFinite(maxStep) {
		return nil, empty, fmt.Errorf("%w: section needs unbounded angular refinement", decaderr.ErrUnsupported)
	}
	need := math.Ceil(2 * math.Pi / maxStep)
	if need > 8192 {
		return nil, empty, fmt.Errorf("%w: section exceeds the bounded angular plan", decaderr.ErrUnsupported)
	}
	stations := 16
	for float64(stations) < need {
		stations *= 2
	}
	// All station values are held samples. BuildRevolvedSectionRing later
	// encloses their exact trigonometric values and the largest held gap.
	start := math.Pi + (sector[0]+sector[1])/2
	angles := make([]float64, 0, stations+2)
	for i := range stations {
		angles = append(angles, start+2*math.Pi*float64(i)/float64(stations))
	}
	shifted := [2]float64{sector[0] + 2*math.Pi, sector[1] + 2*math.Pi}
	angles = append(angles, shifted[:]...)
	sort.Float64s(angles)
	unique := angles[:0]
	for _, angle := range angles {
		if len(unique) == 0 || angle != unique[len(unique)-1] {
			unique = append(unique, angle)
		}
	}
	if shifted[0] <= unique[0] || shifted[1] >= unique[len(unique)-1]+
		(2*math.Pi/float64(stations)) {
		return nil, empty, fmt.Errorf("%w: contact crosses the angular plan seam", decaderr.ErrUnsupported)
	}
	return unique, shifted, nil
}
