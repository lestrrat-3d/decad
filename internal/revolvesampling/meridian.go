package revolvesampling

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file forms the certified meridian samples used by revolve tessellation.
// MeridianJunctions reads each recorded walk's start. Circular and free-form
// walks add interior stations at exact rational parameters. Each sample carries a
// certified bound against the point its record denotes (tessellation §8–§9).

// ArcStation is one interior meridian sample of a circular walk: the
// axis coordinates the RECORD denotes at station k of n, enclosed exactly, and
// the float pair this mesh stores for it.
//
// The station's recorded parameter is TStart + (k/n)·(TEnd − TStart), taken as
// an EXACT rational — rounding it to a float first would enclose the recorded
// curve at a neighbouring parameter and prove a bound about a point this
// chording never named (stationbound.ChordStationBound's own rule). The plane-local
// enclosure comes from circularbounds.EndpointInterval, and
// revolvemesh.AxisCoordInterval carries it
// into (z, ρ) through the payload's own axis frame with no rounding.
//
// The stored pair is the float NEAREST the enclosure's midpoint. That is what
// makes the station's construction gap a fact about this build: it is at most
// the enclosure's own width plus half an ulp, both of which the tolerance split
// reserved before the chord count and the caller measures afterward.
// Nothing here calls math.Cos or math.Sin.
//
// A station the record cannot enclose refuses. A station the arithmetic puts
// ON or BEYOND the axis is ErrDegenerate rather than a pole: a generator that
// meets the axis at an interior point sweeps no manifold solid, and §12 forbids
// rounding a near-axis ring onto the axis to make one.
func ArcStation(lift revolvemesh.RevolveLift, seg sectionrecord.CurveSegment, k, n int) (revolvemesh.RevMeridian, float64, error) {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return revolvemesh.RevMeridian{}, 0, err
	}
	start, span, ok := loftmesh.CircularSegmentRange(seg)
	if !ok || n <= 0 || k <= 0 || k >= n {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	frac := new(big.Rat).SetFrac64(int64(k), int64(n))
	rt := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
	uIv, vIv, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(seg), rt)
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	zIv, rhoIv, ok := revolvemesh.AxisCoordInterval(lift.AU, lift.AV, lift.DU, lift.DV, uIv, vIv)
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	z, _ := midpoint(zIv).Float64()
	rho, _ := midpoint(rhoIv).Float64()
	gap := math.Max(proofbound.IntervalFloatError(zIv, z), proofbound.IntervalFloatError(rhoIv, rho))
	if proofbound.IsNonFinite(z) || proofbound.IsNonFinite(rho) || proofbound.IsNonFinite(gap) {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	if rho <= 0 {
		return revolvemesh.RevMeridian{}, 0, fmt.Errorf(`%w: a revolve meridian chord station lands on the axis or across it, so the recorded generator sweeps no manifold solid there`, decaderr.ErrDegenerate)
	}
	return revolvemesh.RevMeridian{Z: z, Rho: rho, ZIv: zIv, RhoIv: rhoIv}, gap, nil
}

func midpoint(iv proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(1, 2))
}

// MeridianJunctions is one loop's count-independent meridian samples: junction k
// is walk k's own start, which is walk k−1's end, so each junction is emitted
// exactly once and the polyline closes by construction.
//
// Each carries the certified enclosure of the (z, ρ) the RECORD denotes there,
// read from the recorded plane-local point the axis re-expression consumed
// rather than from the re-expressed floats themselves. That point is walk k's
// held plane start under the bound that reaches the points both neighbours
// denote there (boundarywalk.JunctionStartBound), so an arc's natural t = 1
// end, whose held End sits off Start's radius, is enclosed. The returned gap is the
// largest distance any junction's stored pair sits from its own enclosure — the
// count-independent half of deltaC the tolerance split spends before any count
// exists.
func MeridianJunctions(lift revolvemesh.RevolveLift, r revolveaxis.ResolvedWalks) ([]revolvemesh.RevMeridian, float64, error) {
	out := make([]revolvemesh.RevMeridian, len(r.Walks))
	worst := 0.0
	n := len(r.Walks)
	for k, w := range r.Walks {
		plane := r.Plane[w.Segs[0]]
		prev := r.Walks[(k+n-1)%n]
		last := prev.Segs[len(prev.Segs)-1]
		bound := boundarywalk.JunctionStartBound(r.Segs[last], r.Plane[last], r.Segs[w.Segs[0]], plane)
		zIv, rhoIv, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, plane.StartU, plane.StartV, bound)
		if !ok {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no enclosure of the axis coordinates its record denotes`, decaderr.ErrUnsupported)
		}
		gap := math.Max(proofbound.IntervalFloatError(zIv, w.StartU), proofbound.IntervalFloatError(rhoIv, w.StartV))
		if proofbound.IsNonFinite(gap) {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no bound on its own axis coordinates`, decaderr.ErrUnsupported)
		}
		worst = math.Max(worst, gap)
		if w.StartV < 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample sits on the negative side of the axis`, decaderr.ErrDegenerate)
		}
		out[k] = revolvemesh.RevMeridian{Z: w.StartU, Rho: w.StartV, ZIv: zIv, RhoIv: rhoIv, OnAxis: w.StartV == 0, Walk: k}
	}
	return out, worst, nil
}

// MeridianSamples expands one loop's junctions into the polyline the
// current counts ask for: walk k contributes its own junction plus the
// interior stations of a curved walk, each enclosed at the exact recorded
// parameter it denotes (ArcStation or freeformStation).
//
// The returned gap is the largest interior station gap; the caller combines it
// with MeridianJunctions' gap before checking the reserved construction bound.
func MeridianSamples(lift revolvemesh.RevolveLift, loop sectionrecord.LoopRecord, r revolveaxis.ResolvedWalks,
	junctions []revolvemesh.RevMeridian, counts []int, sags []float64,
	chains []*freeform.FreeformChain) ([]revolvemesh.RevMeridian, float64, error) {
	out := make([]revolvemesh.RevMeridian, 0, len(junctions))
	worst := 0.0
	for k, w := range r.Walks {
		n := counts[k]
		if n <= 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian walk carries no chord`, decaderr.ErrUnsupported)
		}
		start := junctions[k]
		start.Sag, start.Walk = sags[k], k
		if w.Kind == survey2d.WalkFreeform {
			if len(chains) <= k || chains[k] == nil || len(chains[k].Stations) != n {
				return nil, 0, fmt.Errorf(`%w: a free-form revolve meridian has no certified station chain`, decaderr.ErrUnsupported)
			}
			chain := chains[k]
			for i := range n {
				cell := i
				if w.Reversed {
					cell = n - 1 - i
				}
				start.Freeform = &revolvemesh.RevFreeformCell{
					ArcUpper: chain.CellArcUpper[cell], RhoUpper: w.AxisRadiusUpper,
				}
				out = append(out, start)
				if i == n-1 {
					break
				}
				station := chain.Stations[i+1]
				if w.Reversed {
					station = chain.Stations[n-1-i]
				}
				next, gap, err := freeformStation(lift, station)
				if err != nil {
					return nil, 0, err
				}
				start = next
				start.Walk, start.Sag = k, sags[k]
				worst = math.Max(worst, gap)
			}
			continue
		}
		if !w.IsCircular() {
			out = append(out, start)
			continue
		}
		cell, ok := revolvemesh.RevolveArcChordCell(w.SegmentWalk, 0, n)
		if !ok {
			return nil, 0, revolvemesh.ErrRevolveArcCellSlack
		}
		start.Arc = cell
		out = append(out, start)
		for i := 1; i < n; i++ {
			station, gap, err := ArcStation(lift, loop.Segments[w.Segs[0]], i, n)
			if err != nil {
				return nil, 0, err
			}
			cell, ok := revolvemesh.RevolveArcChordCell(w.SegmentWalk, i, n)
			if !ok {
				return nil, 0, revolvemesh.ErrRevolveArcCellSlack
			}
			station.Walk, station.Sag, station.Arc = k, sags[k], cell
			worst = math.Max(worst, gap)
			out = append(out, station)
		}
	}
	return out, worst, nil
}

func freeformStation(lift revolvemesh.RevolveLift, point freeform.RatPoint) (revolvemesh.RevMeridian, float64, error) {
	_, ok := splinebezier.Point2Of(point)
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	zIv, rhoIv, ok := revolvemesh.AxisCoordInterval(lift.AU, lift.AV, lift.DU, lift.DV,
		proofbound.PointInterval(point.U), proofbound.PointInterval(point.V))
	if !ok {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	z, _ := midpoint(zIv).Float64()
	rho, _ := midpoint(rhoIv).Float64()
	gap := math.Max(proofbound.IntervalFloatError(zIv, z), proofbound.IntervalFloatError(rhoIv, rho))
	if proofbound.IsNonFinite(z) || proofbound.IsNonFinite(rho) || proofbound.IsNonFinite(gap) || rho <= 0 {
		return revolvemesh.RevMeridian{}, 0, revolvemesh.ErrRevolveStationEnclosure
	}
	return revolvemesh.RevMeridian{Z: z, Rho: rho, ZIv: zIv, RhoIv: rhoIv}, gap, nil
}
