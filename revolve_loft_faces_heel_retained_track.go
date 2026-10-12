package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/sketch"
)

// A retained heel track keeps each raw source enclosure even if two rounded
// stations merge. Wall strips need the live source owner for every slab.
type revolveLoftFacesHeelRetainedTrack struct {
	stations []revolveLoftFacesHeelTrackStation
	slabs    []revolveLoftFacesHeelRetainedSlab
	arc      []revolveLoftFacesSeamVertex
	calls    int
}

type revolveLoftFacesHeelRetainedSlab struct {
	band int
	time loftFaceInterval
	root [2]*revolveLoftFacesHeelRootEnclosure
}

type revolveLoftFacesHeelRetainedTracks struct {
	birth    *revolveLoftFacesHeelRetainedTrack
	terminal *revolveLoftFacesHeelRetainedTrack
}

// The birth, terminal, and tail certificates are rederived from the live
// producers before their event boxes can guide a retained cap patch.
func revolveLoftFacesHeelRetainedTracksOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, budget *big.Rat) (
	*revolveLoftFacesHeelRetainedTracks, bool, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || budget == nil || budget.Sign() <= 0 {
		return nil, false, nil
	}
	birth, ok, err := certifyRevolveLoftFacesHeelBirth(ctx, proof)
	if err != nil || !ok || birth == nil {
		return nil, false, err
	}
	terminal, ok, err := certifyRevolveLoftFacesHeelTerminal(ctx, proof)
	if err != nil || !ok || terminal == nil {
		return nil, false, err
	}
	if _, ok, err = certifyRevolveLoftFacesHeelTail(ctx, proof); err != nil || !ok {
		return nil, false, err
	}
	if birth.capFace == nil || birth.capFace != terminal.capFace ||
		birth.lower == nil || terminal.upper == nil ||
		birth.lower.circle.x.Cmp(terminal.upper.circle.x) >= 0 {
		return nil, false, nil
	}
	first, ok, err := revolveLoftFacesHeelRetainedTrackFromCurrent(ctx, proof,
		birth.meridian, birth.slices, birth.arc, birth.lower.band, birth.root, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	second, ok, err := revolveLoftFacesHeelRetainedTrackFromCurrent(ctx, proof,
		terminal.meridian, []revolveLoftFacesHeelRegionSlice{terminal.before},
		terminal.arc, terminal.upper.band, terminal.root, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	if !revolveLoftFacesHeelTrackCapEnds(&revolveLoftFacesHeelTrack{
		stations: []revolveLoftFacesHeelTrackStation{
			first.stations[len(first.stations)-1], second.stations[0]}},
		birth.lower, terminal.upper) {
		return nil, false, nil
	}
	return &revolveLoftFacesHeelRetainedTracks{birth: first, terminal: second}, true, nil
}

func revolveLoftFacesHeelRetainedTrackFromCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, meridian revolveSectionSpan,
	slices []revolveLoftFacesHeelRegionSlice, arc *sketch.Arc, band int,
	root loftFaceInterval, budget *big.Rat) (*revolveLoftFacesHeelRetainedTrack, bool, error) {
	if len(slices) == 0 || arc == nil || root.lo == nil || root.hi == nil {
		return nil, false, nil
	}
	graph := &revolveLoftFacesHeelRegionEvents{meridian: meridian, slices: slices}
	track := &revolveLoftFacesHeelRetainedTrack{}
	for sliceIndex, slice := range slices {
		stack := []revolveLoftFacesHeelTrackSlab{{
			sliceIndex: sliceIndex, lo: slice.lo, hi: slice.hi}}
		for len(stack) != 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			if track.calls+2 > 512 || len(track.slabs) >= 256 {
				return nil, false, nil
			}
			slab := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			var roots [2]*revolveLoftFacesHeelRootEnclosure
			both := true
			for event := range roots {
				track.calls++
				var boxed bool
				var err error
				roots[event], boxed, err = revolveLoftFacesHeelEventBoxFromCurrent(
					ctx, proof, graph, sliceIndex, event, slab.lo, slab.hi, budget)
				if err != nil {
					return nil, false, err
				}
				both = both && boxed && roots[event] != nil
			}
			if both {
				station := revolveLoftFacesHeelTrackStation{
					box: [2][3]loftFaceInterval{roots[0].box, roots[1].box}}
				if !revolveLoftFacesHeelTrackPlace(&station, budget) {
					return nil, false, nil
				}
				track.slabs = append(track.slabs, revolveLoftFacesHeelRetainedSlab{
					band: slice.band, time: loftFaceInterval{slab.lo, slab.hi}, root: roots})
				merged := &revolveLoftFacesHeelTrack{stations: track.stations}
				if !revolveLoftFacesHeelTrackAppend(merged, station, budget) {
					return nil, false, nil
				}
				track.stations = merged.stations
				continue
			}
			if slice.point || slab.depth >= 10 || slab.lo.Cmp(slab.hi) >= 0 {
				return nil, false, nil
			}
			mid := lfDiv(lfAdd(slab.lo, slab.hi), big.NewRat(2, 1))
			if mid.Cmp(slab.lo) <= 0 || mid.Cmp(slab.hi) >= 0 {
				return nil, false, nil
			}
			stack = append(stack,
				revolveLoftFacesHeelTrackSlab{sliceIndex, slab.depth + 1, mid, slab.hi},
				revolveLoftFacesHeelTrackSlab{sliceIndex, slab.depth + 1, slab.lo, mid})
		}
	}
	if len(track.stations) < 2 || len(track.slabs) < 2 {
		return nil, false, nil
	}
	arcIndex := -1
	for index, entity := range proof.trim.source.source.entities {
		if entity == arc {
			arcIndex = index
		}
	}
	if arcIndex < 0 {
		return nil, false, nil
	}
	var stations []int
	for index, segment := range proof.trim.stationGroup {
		if segment == arcIndex {
			stations = append(stations, index)
		}
	}
	if len(stations) < 2 {
		return nil, false, nil
	}
	stations = append(stations, (stations[len(stations)-1]+1)%len(proof.trim.stationGroup))
	var ok bool
	var err error
	track.arc, ok, err = revolveLoftFacesArcSeamVertices(ctx, proof.trim,
		band, stations, root, meridian)
	if err != nil || !ok || len(track.arc) < 2 {
		return nil, false, err
	}
	return track, true, nil
}
