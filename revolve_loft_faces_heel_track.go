package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A heel track is a bounded construction guide for the two moving source
// contacts. It has no retained-face or occupied-volume proof of its own.
type revolveLoftFacesHeelTrack struct {
	region   *revolveLoftFacesHeelRegionCertificate
	stations []revolveLoftFacesHeelTrackStation
	calls    int
}

type revolveLoftFacesHeelTrackStation struct {
	box         [2][3]loftFaceInterval
	point       [2]r3.Vec
	sourceBound [2]float64
}

type revolveLoftFacesHeelTrackSlab struct {
	sliceIndex, depth int
	lo, hi            *big.Rat
}

// The source graph proves two continuous, ordered contacts. Each accepted
// time slab encloses both contacts in exact rational XYZ boxes. Adjacent
// boxes share the same true source point at their common time boundary.
// The returned bound includes the box's L1 radius and final float rounding.
// A later mesh consumer must add its own chord, facet, and volume charges.
func revolveLoftFacesHeelTrackOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, budget *big.Rat) (
	*revolveLoftFacesHeelTrack, bool, error,
) {
	if ctx == nil || proof == nil || budget == nil || budget.Sign() <= 0 {
		return nil, false, nil
	}
	region, ok, err := certifyRevolveLoftFacesHeelRegion(ctx, proof)
	if err != nil || !ok || region == nil {
		return nil, false, err
	}
	track := &revolveLoftFacesHeelTrack{region: region}
	for sliceIndex, slice := range region.slices {
		stack := []revolveLoftFacesHeelTrackSlab{{
			sliceIndex: sliceIndex, lo: slice.lo, hi: slice.hi}}
		for len(stack) > 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			if track.calls+2 > 256 || len(track.stations) > 256 {
				return nil, false, nil
			}
			slab := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			var boxes [2]*revolveLoftFacesHeelRootEnclosure
			both := true
			for event := range boxes {
				track.calls++
				boxes[event], ok, err = revolveLoftFacesHeelEventBoxFromCurrent(
					ctx, proof, region.revolveLoftFacesHeelRegionEvents,
					slab.sliceIndex, event, slab.lo, slab.hi, budget)
				if err != nil {
					return nil, false, err
				}
				both = both && ok && boxes[event] != nil
			}
			if both {
				station := revolveLoftFacesHeelTrackStation{
					box: [2][3]loftFaceInterval{boxes[0].box, boxes[1].box}}
				if !revolveLoftFacesHeelTrackAppend(track, station, budget) {
					return nil, false, nil
				}
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
	if len(track.stations) < 2 ||
		!revolveLoftFacesHeelTrackCapEnds(track, region.lower, region.upper) {
		return nil, false, nil
	}
	return track, true, nil
}

func revolveLoftFacesHeelTrackAppend(track *revolveLoftFacesHeelTrack,
	station revolveLoftFacesHeelTrackStation, budget *big.Rat) bool {
	if track == nil || budget == nil {
		return false
	}
	if !revolveLoftFacesHeelTrackPlace(&station, budget) {
		return false
	}
	if len(track.stations) == 0 {
		track.stations = append(track.stations, station)
		return true
	}
	previous := &track.stations[len(track.stations)-1]
	for event := range station.box {
		for axis := range station.box[event] {
			a := previous.box[event][axis]
			b := station.box[event][axis]
			if a.hi.Cmp(b.lo) < 0 || b.hi.Cmp(a.lo) < 0 {
				return false
			}
		}
	}
	if station.point[0].X == previous.point[0].X {
		merged := *previous
		for event := range merged.box {
			for axis := range merged.box[event] {
				merged.box[event][axis] = lfIntervalHull(
					merged.box[event][axis], station.box[event][axis])
			}
		}
		if !revolveLoftFacesHeelTrackPlace(&merged, budget) ||
			merged.point[0].X != previous.point[0].X {
			return false
		}
		*previous = merged
		return true
	}
	if station.point[0].X <= previous.point[0].X {
		return false
	}
	track.stations = append(track.stations, station)
	return true
}

func revolveLoftFacesHeelTrackPlace(station *revolveLoftFacesHeelTrackStation,
	budget *big.Rat) bool {
	if station == nil || budget == nil {
		return false
	}
	for event, box := range station.box {
		radius := revolveLoftFacesHeelBoxRadius(box)
		if radius == nil || radius.Cmp(budget) > 0 || box[1].lo.Sign() <= 0 {
			return false
		}
		var center loftFaceRatVec
		for axis := range center {
			center[axis] = lfDiv(lfAdd(box[axis].lo, box[axis].hi),
				big.NewRat(2, 1))
		}
		point, rounding, ok := revolveLoftFacesHeldPoint(center)
		if !ok {
			return false
		}
		station.point[event] = point
		station.sourceBound[event] = proofbound.AbsSumUpper(
			proofbound.RatFloatUp(radius), rounding)
		if proofbound.IsNonFinite(station.sourceBound[event]) {
			return false
		}
	}
	if station.box[0][0].lo.Cmp(station.box[1][0].lo) != 0 ||
		station.box[0][0].hi.Cmp(station.box[1][0].hi) != 0 ||
		station.point[0].X != station.point[1].X ||
		station.point[0].Y <= 0 || station.point[1].Y <= 0 ||
		station.point[0].Z <= 0 || station.point[1].Z >= 0 ||
		station.box[0][2].lo.Sign() <= 0 || station.box[1][2].hi.Sign() >= 0 {
		return false
	}
	angular := mappedArcIntervalSub(
		mappedArcIntervalMul(station.box[1][1], station.box[0][2]),
		mappedArcIntervalMul(station.box[1][2], station.box[0][1]))
	return angular.lo.Sign() > 0 &&
		station.point[1].Y*station.point[0].Z-
			station.point[1].Z*station.point[0].Y > 0
}

func revolveLoftFacesHeelTrackCapEnds(track *revolveLoftFacesHeelTrack,
	lower, upper *revolveLoftFacesToolCapOccupiedArc) bool {
	if track == nil || lower == nil || upper == nil || len(track.stations) < 2 {
		return false
	}
	for _, entry := range []struct {
		station revolveLoftFacesHeelTrackStation
		arc     *revolveLoftFacesToolCapOccupiedArc
	}{{track.stations[0], lower}, {track.stations[len(track.stations)-1], upper}} {
		for event, endpoint := range [...]revolveLoftFacesToolCapBoundary{
			entry.arc.positiveZ, entry.arc.negativeZ} {
			if endpoint.outward != (event == 1) {
				return false
			}
			for axis := range entry.station.box[event] {
				if entry.station.box[event][axis].hi.Cmp(endpoint.box[axis].lo) < 0 ||
					endpoint.box[axis].hi.Cmp(entry.station.box[event][axis].lo) < 0 {
					return false
				}
			}
		}
	}
	return true
}
