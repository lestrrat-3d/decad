// Package compositemesh plans shared stations and maps reduced span meshes.
package compositemesh

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MaxFacets is the existing fixed composite Sweep mesh ceiling.
const MaxFacets = 65_536

// ResolveRevolve calls the root payload adapter without importing root.
type ResolveRevolve func(context.Context, revolveplan.Payload, uint64) (*revolveplan.Resolution, error)

// SpanRings maps reduced span mesh rings by their shared profile indices.
func SpanRings(ctx context.Context, span sweeprecord.SpanPayload, mesh *topo.Mesh,
	samples int, freeformTarget float64, resolve ResolveRevolve) ([]int, []int, float64, error) {
	start := make([]int, samples)
	end := make([]int, samples)
	if !span.Arc {
		if len(mesh.Vertices) != 2*samples {
			return nil, nil, 0, fmt.Errorf(`%w: a straight sweep span has a different profile station chain`, decaderr.ErrUnsupported)
		}
		for j := range samples {
			start[j], end[j] = 2*j, 2*j+1
		}
		return start, end, mesh.CoordBound, nil
	}
	res, err := resolve(ctx, span.Revolve, workLimit(freeformTarget))
	if err != nil {
		return nil, nil, 0, err
	}
	for _, loop := range res.Junctions {
		for _, junction := range loop {
			if junction.OnAxis {
				return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span's pole needs a shared station plan`, decaderr.ErrUnsupported)
			}
		}
	}
	if len(mesh.Vertices)%samples != 0 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has an incomplete angular station chain`, decaderr.ErrUnsupported)
	}
	angular := len(mesh.Vertices) / samples
	if angular < 2 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has no angular cell`, decaderr.ErrUnsupported)
	}
	for j := range samples {
		start[j], end[j] = j*angular, j*angular+angular-1
		if span.ReverseArcCaps {
			start[j], end[j] = end[j], start[j]
		}
	}
	return start, end, mesh.CoordBound, nil
}

// SpanFaceRole maps a reduced face origin to its composite role.
func SpanFaceRole(face *topo.Face, span int, reverseCaps bool) (string, error) {
	origins := face.Origins
	if len(origins) == 0 {
		return "", fmt.Errorf(`%w: a sweep span face has no origin role`, decaderr.ErrDegenerate)
	}
	return compositesweep.SpanFaceRole(origins[0].Role, span, reverseCaps, provenance.RoleCapStart, provenance.RoleCapEnd), nil
}

// StationKey identifies a segment of one recorded profile loop.
type StationKey struct{ Loop, Segment int }

func workLimit(freeformTarget float64) uint64 {
	if freeformTarget > 0 {
		return sweeprecord.CompositeFitWorkLimit
	}
	return 0
}

// PlanProfileCounts chooses one parameter count per recorded profile
// segment before any span emits vertices. Each circular reduction's own count
// is a lower bound. Fitted splines instead share the smallest certified
// chording target across their prism and Revolve spans.
func PlanProfileCounts(ctx context.Context, sp sweeprecord.Payload, chord float64,
	resolve ResolveRevolve) (map[StationKey]int, int, float64, error) {
	loops := append([]sectionrecord.LoopRecord{sp.Prism.Profile.Outer}, sp.Prism.Profile.Holes...)
	counts := map[StationKey]int{}
	work := sweeprecord.NewCompositeWork(sp.Prism.Profile)
	freeformTarget := chord
	hasFreeform := false
	for li, loop := range loops {
		for si, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return nil, 0, 0, err
			}
			key := StationKey{li, si}
			switch seg.(type) {
			case sectionrecord.LineSeg:
				counts[key] = 1
			case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				walk, err := boundarywalk.WalkOf(seg, work)
				if err != nil {
					return nil, 0, 0, err
				}
				n, _, err := tessellation.ChordCount(walk, chord, tessellation.ChordWalkMin(walk))
				if err != nil {
					return nil, 0, 0, err
				}
				counts[key] = n
			case sectionrecord.FitSplineSeg:
				hasFreeform = true
			default:
				return nil, 0, 0, fmt.Errorf(`%w: composite sweep mesh profile stations do not cover segment %T`, decaderr.ErrUnsupported, seg)
			}
		}
	}
	for _, span := range sp.Spans {
		if !span.Arc {
			continue
		}
		workLimit := uint64(0)
		if hasFreeform {
			workLimit = sweeprecord.CompositeFitWorkLimit
		}
		res, err := resolve(ctx, span.Revolve, workLimit)
		if err != nil {
			return nil, 0, 0, err
		}
		planned, err := revolveplan.PlanCounts(revolveplan.CountInput{
			Resolution: res, Chord: chord, SectionDelta: span.Revolve.SectionDelta,
			Phi0: span.Revolve.Phi0, Phi1: span.Revolve.Phi1, Full: span.Revolve.Full,
			WorkLimit: workLimit,
		})
		if err != nil {
			return nil, 0, 0, err
		}
		freeformTarget = min(freeformTarget, planned.MeridianTarget)
		for li, loop := range res.Resolved {
			for wi, walk := range loop.Walks {
				key := StationKey{li, walk.Segs[0]}
				counts[key] = max(counts[key], planned.Meridian[li][wi])
			}
		}
	}
	if hasFreeform {
		for li, loop := range loops {
			for si, seg := range loop.Segments {
				if _, ok := seg.(sectionrecord.FitSplineSeg); !ok {
					continue
				}
				walk, err := boundarywalk.WalkOf(seg, work)
				if err != nil {
					return nil, 0, 0, err
				}
				chain, err := freeform.ChainStations(walk.Spans, freeformTarget, work)
				if err != nil {
					return nil, 0, 0, err
				}
				counts[StationKey{li, si}] = len(chain.Stations)
			}
		}
	} else {
		freeformTarget = 0
	}
	total := 0
	for li, loop := range loops {
		loopSamples := 0
		for si := range loop.Segments {
			count := counts[StationKey{li, si}]
			if count > MaxFacets-loopSamples {
				return nil, 0, 0, fmt.Errorf(`%w: the composite sweep profile exceeds the fixed mesh station ceiling`, decaderr.ErrUnsupported)
			}
			loopSamples += count
		}
		if loopSamples < 3 {
			return nil, 0, 0, fmt.Errorf(`%w: a composite sweep mesh loop needs at least three profile samples`, decaderr.ErrDegenerate)
		}
		if loopSamples > MaxFacets-total {
			return nil, 0, 0, fmt.Errorf(`%w: the composite sweep profile exceeds the fixed mesh station ceiling`, decaderr.ErrUnsupported)
		}
		total += loopSamples
	}
	return counts, total, freeformTarget, nil
}

// RevolveFloor maps shared profile counts to a reduced arc span.
func RevolveFloor(ctx context.Context, rp revolveplan.Payload,
	counts map[StationKey]int, freeformTarget float64, resolve ResolveRevolve) ([][]int, error) {
	res, err := resolve(ctx, rp, workLimit(freeformTarget))
	if err != nil {
		return nil, err
	}
	floor := make([][]int, len(res.Resolved))
	for li, loop := range res.Resolved {
		floor[li] = make([]int, len(loop.Walks))
		for wi, walk := range loop.Walks {
			key := StationKey{li, walk.Segs[0]}
			count, ok := counts[key]
			if !ok {
				return nil, fmt.Errorf(`%w: an arc sweep meridian has no shared profile station count`, decaderr.ErrUnsupported)
			}
			floor[li][wi] = count
		}
	}
	return floor, nil
}
