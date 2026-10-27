package stackedunion

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/sketch"
)

// This file states agent-docs/general-boolean-design.md §3 A8: a cut-built
// stack whose one interface holds opposed exclusive holes that overlap in
// plan. The slabs are the ones stackedrecord already proved; every other
// interface keeps its derived exposed records. The opposed interface's faces
// are cells of one private scene over only the exclusive holes' interiors:
// the lower side's holes as the scene's A regions, the upper side's as its
// B regions. A cell inside B and outside A is a floor, one inside A and
// outside B a ceiling. The A1 engine then splits the hole walls where the
// cells' vertices fall on them, as it does for A7.

// opposedClass is one interface cell's reading: material below exposed
// upwards, material above exposed downwards, or neither.
type opposedClass int

const (
	opposedNone opposedClass = iota
	opposedFloor
	opposedCeiling
)

// OpposedCut states oc as a brepPayload. A topology this build does not
// cover, opposed holes that touch, and any scene sketch leaves unresolved or
// uncertified return brepgeom.ErrStackedWallMiss for the caller to read as a
// silent miss.
func OpposedCut(ctx context.Context, budget *proofbound.WorkBudget, oc stackedrecord.OpposedCut) (breprecord.Payload, error) {
	n := len(oc.Slabs)
	levels := make([]stackedbrep.Level, 0, n+1)
	levels = append(levels, stackedbrep.Level{Held: oc.Slabs[0].Z0, Delta: oc.Slabs[0].Z0Delta})
	for _, slab := range oc.Slabs {
		levels = append(levels, stackedbrep.Level{Held: slab.Z1, Delta: slab.Z1Delta})
	}
	held := make([]float64, len(levels))
	for i, level := range levels {
		held[i] = level.Held
	}
	geom := stackedbrep.NewEngine(held)
	regionLoops := func(region profileRecord) (ubLoop, []ubLoop, error) {
		outer, err := geom.LoopOf(brepgeom.Profile{Outer: region.Outer}, budget)
		if err != nil {
			return ubLoop{}, nil, err
		}
		holes := make([]ubLoop, 0, len(region.Holes))
		for _, ring := range region.Holes {
			hole, err := geom.LoopOf(brepgeom.Profile{Outer: ring}, budget)
			if err != nil {
				return ubLoop{}, nil, err
			}
			holes = append(holes, hole)
		}
		return outer, holes, nil
	}
	addRegion := func(region profileRecord, level int, outward bool) error {
		outer, holes, err := regionLoops(region)
		if err != nil {
			return err
		}
		return geom.AddFaceRegion(outer, holes, level, outward)
	}
	// Slab loops first: their recorded corners seed the engine's vertex table.
	for _, slab := range oc.Slabs {
		var loops []ubLoop
		for _, region := range slab.Regions {
			outer, holes, err := regionLoops(region)
			if err != nil {
				return breprecord.Payload{}, err
			}
			loops = append(append(loops, outer), holes...)
		}
		geom.SlabLoops = append(geom.SlabLoops, loops)
	}
	for _, region := range oc.Slabs[0].Regions {
		if err := addRegion(region, 0, false); err != nil {
			return breprecord.Payload{}, err
		}
	}
	for _, region := range oc.Slabs[n-1].Regions {
		if err := addRegion(region, n, true); err != nil {
			return breprecord.Payload{}, err
		}
	}
	for i, iface := range oc.Interfaces {
		if i == oc.At {
			cells, err := opposedCells(ctx, budget, oc.LowerOnly, oc.UpperOnly)
			if err != nil {
				return breprecord.Payload{}, err
			}
			for _, cell := range cells {
				if err := addRegion(cell.region, i+1, cell.floor); err != nil {
					return breprecord.Payload{}, err
				}
			}
			continue
		}
		for _, exposed := range iface.LowerExposed {
			if err := addRegion(exposed, i+1, true); err != nil {
				return breprecord.Payload{}, err
			}
		}
		for _, exposed := range iface.UpperExposed {
			if err := addRegion(exposed, i+1, false); err != nil {
				return breprecord.Payload{}, err
			}
		}
	}
	geom.RecordJunctions(n)
	// Every scene vertex is two axis-aligned lines' crossing, every span both
	// sides share lies on one exact line, and every line edge ends exactly on
	// its own carrier, so each vertex sits at exact levels. The only allowance
	// is the one the engine records for the slab loops' own junctions.
	return breprecord.StackedRecord(ctx, budget, geom, levels, oc.Frame, oc.Xform, geom.Allow())
}

// opposedCell is one recorded interface face: a floor (outward +N) or a
// ceiling (outward −N).
type opposedCell struct {
	region profileRecord
	floor  bool
}

// opposedCells arranges the exclusive holes' interiors in one private scene
// and records its floor and ceiling cells. Every partial edge must carry
// TExact, every line edge must end exactly on its own carrier, the scene must
// carry no walk charge, every shared span must have exact carriers, and every
// cell must classify.
func opposedCells(ctx context.Context, budget *proofbound.WorkBudget,
	lowerOnly, upperOnly []sectionrecord.LoopRecord) ([]opposedCell, error) {
	regionsA, err := stackedrecord.Exposed(ctx, lowerOnly)
	if err != nil {
		return nil, err
	}
	regionsB, err := stackedrecord.Exposed(ctx, upperOnly)
	if err != nil {
		return nil, err
	}
	segments, within, err := prismcells.RegionsWithinWorkCap(budget, append(append([]profileRecord{}, regionsA...), regionsB...)...)
	if err != nil {
		return nil, err
	}
	if !within {
		return nil, prismcells.WithWorkLimit(fmt.Errorf(
			`%w: the opposed interface scene charges at least %d arranger segments against this evaluator's cap of %d`,
			decaderr.ErrUnsupported, segments, prismcells.MaxArrangementSegments), segments)
	}
	s, tags, delta, err := prismcells.BuildProfileScene(budget, regionsA, regionsB, &prismReexpression{Identity: true})
	if err != nil {
		return nil, err
	}
	if delta.A != 0 || delta.B != 0 {
		return nil, errUBMiss
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return nil, err
	}
	if len(profiles) == 0 {
		return nil, errUBMiss
	}
	for _, cell := range profiles {
		for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, edge := range ring {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				if edge.Partial && !edge.TExact {
					return nil, errUBMiss
				}
				// Sketch merges two lines whose carriers differ within its
				// identity band. Where the displaced line's own endpoint ends
				// the merged span, no span reaches the coincident reading, but
				// the edge then ends off its own carrier.
				if _, isLine := edge.Entity.(*sketch.Line); isLine {
					if len(edge.Polyline) == 0 ||
						!prismcells.OnLineCarrier(edge.Polyline[0], edge.Entity) ||
						!prismcells.OnLineCarrier(edge.Polyline[len(edge.Polyline)-1], edge.Entity) {
						return nil, errUBMiss
					}
				}
			}
		}
	}
	matter, resolved, err := prismcells.ClassifyRegionsWithHoles(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, errUBMiss
	}
	classes := make([]opposedClass, len(profiles))
	for i := range profiles {
		inA, inB := false, false
		for r := range regionsA {
			inA = inA || matter[prismcells.RegionKey{Region: r}][i]
		}
		for r := range regionsB {
			inB = inB || matter[prismcells.RegionKey{IsB: true, Region: r}][i]
		}
		switch {
		case inB && !inA:
			classes[i] = opposedFloor
		case inA && !inB:
			classes[i] = opposedCeiling
		}
	}
	coincident, ok, err := prismcells.CoincidentEdgesRegions(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, errUBMiss
	}
	// Sketch resolves two lines as one shared span when their carriers differ
	// within its identity band. Such a span states no exact face edge, so both
	// recorded lines of every span must lie on precisely one line.
	exact, err := coincident.ExactLineCarriers(budget)
	if err != nil {
		return nil, err
	}
	if !exact {
		return nil, errUBMiss
	}
	manifold, err := opposedManifold(budget, tags, coincident, profiles, classes)
	if err != nil {
		return nil, err
	}
	if !manifold {
		return nil, errUBMiss
	}
	var out []opposedCell
	for i, p := range profiles {
		if classes[i] == opposedNone {
			continue
		}
		region, err := prismcells.RecordArrangedProfileContext(ctx, p)
		if err != nil {
			return nil, err
		}
		out = append(out, opposedCell{region: region, floor: classes[i] == opposedFloor})
	}
	return out, nil
}

// opposedManifold refuses opposed holes that touch. A floor and a ceiling
// that meet at a point leave the body manifold there only where one lower
// and one upper hole boundary cross: each of the two cells then turns at the
// point from one side's edge to the other side's. Every other meeting puts
// one of them between two edges of one side (holes touching at a corner,
// or two reflex corners whose outsides meet) or on a span both sides share
// (holes touching along an edge, whose ends are such points). Under the
// axis-line gate at most four scene edges meet at any point, one per axis
// direction, so no other configuration reaches here. Every point and edge
// is sketch's own, matched by exact equality; the check only refuses.
func opposedManifold(budget *proofbound.WorkBudget, tags map[sketch.Entity]prismcells.Origin,
	coincident prismcells.CoincidentReading, profiles []*sketch.Profile, classes []opposedClass) (bool, error) {
	type point struct {
		classes map[opposedClass]struct{}
		turnsOK bool
	}
	points := map[[2]float64]*point{}
	for i, cell := range profiles {
		for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for j, e := range ring {
				if err := budget.Step(); err != nil {
					return false, err
				}
				next := ring[(j+1)%len(ring)]
				if len(e.Polyline) == 0 || len(next.Polyline) == 0 ||
					next.Polyline[0] != e.Polyline[len(e.Polyline)-1] {
					return false, nil
				}
				end := e.Polyline[len(e.Polyline)-1]
				pt, ok := points[end]
				if !ok {
					pt = &point{classes: map[opposedClass]struct{}{}, turnsOK: true}
					points[end] = pt
				}
				pt.classes[classes[i]] = struct{}{}
				if classes[i] == opposedNone {
					continue
				}
				in, inOK := tags[e.Entity]
				out, outOK := tags[next.Entity]
				_, inShared := coincident.Lookup(e.Entity, e.TStart, e.TEnd)
				_, outShared := coincident.Lookup(next.Entity, next.TStart, next.TEnd)
				if !inOK || !outOK || in.IsB == out.IsB || inShared || outShared {
					pt.turnsOK = false
				}
			}
		}
	}
	for _, pt := range points {
		_, floor := pt.classes[opposedFloor]
		_, ceiling := pt.classes[opposedCeiling]
		if floor && ceiling && !pt.turnsOK {
			return false, nil
		}
	}
	return true, nil
}
