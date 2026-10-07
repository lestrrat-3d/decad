package prismcells

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// SceneProfile is the loop portion of a recorded prism region. The caller
// retains its public record and passes the same segment values here.
type SceneProfile struct {
	Outer LoopRecord
	Holes []LoopRecord
}

// SceneMapper supplies the caller's already composed relative placement.
// Rewind maps and reverses a reflected region before its entities are built.
type SceneMapper interface {
	Reflection() bool
	MapPoint(sectionrecord.Point2) sectionrecord.Point2
	Rewind(*proofbound.WorkBudget, SceneProfile) (SceneProfile, float64, error)
}

// SceneWalkCharge is each operand's largest charge from walking its recorded
// segments into the scene. The caller composes these with its other bounds.
type SceneWalkCharge struct{ A, B float64 }

// BuildSceneRegions creates one private sketch scene over all regions of both
// operands. Entities are deduplicated within each operand only, and region
// and hole indices in tags follow the input order.
func BuildSceneRegions(budget *proofbound.WorkBudget, regionsA, regionsB []SceneProfile,
	reexpress SceneMapper) (*sketch.Sketch, map[sketch.Entity]Origin, SceneWalkCharge, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, nil, SceneWalkCharge{}, fmt.Errorf(`decad: failed to build the private prism-boolean scene: %w`, err)
	}

	points := map[sectionrecord.Point2]*sketch.Point{}
	point := func(p sectionrecord.Point2) *sketch.Point {
		if existing, ok := points[p]; ok {
			return existing
		}
		created := s.CreatePoint(p.U, p.V)
		points[p] = created
		return created
	}

	tags := map[sketch.Entity]Origin{}
	sceneDelta := SceneWalkCharge{}

	// mapPt is the identity for operand A, and for an operand B whose record
	// was already re-expressed and re-wound.
	addOperand := func(profile SceneProfile, isB bool, region int,
		mapPt func(sectionrecord.Point2) sectionrecord.Point2) error {
		type entityKey struct {
			kind    uint8 // 1 = line, 2 = whole circle, 3 = arc (including a partial circle)
			a, b, c sectionrecord.Point2
			radius  float64
		}
		// A new table per operand prevents cross-operand deduplication.
		entities := map[entityKey]struct{}{}
		reexpressPt := func(p sectionrecord.Point2) sectionrecord.Point2 {
			if mapPt == nil {
				return p
			}
			return mapPt(p)
		}
		loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
		for li, loop := range loops {
			hole := li - 1
			tag := func(ent sketch.Entity, authoredReversed bool) {
				tags[ent] = Origin{IsB: isB, Region: region, Hole: hole, AuthoredReversed: authoredReversed}
			}
			for _, seg := range loop.Segments {
				if err := budget.Step(); err != nil {
					return err
				}
				w, err := boundarywalk.WalkOf(seg, nil)
				if err != nil {
					return err
				}
				charge, err := WalkChargeOf(seg, w)
				if err != nil {
					return err
				}
				if isB {
					sceneDelta.B = math.Max(sceneDelta.B, charge)
				} else {
					sceneDelta.A = math.Max(sceneDelta.A, charge)
				}
				// A circular walk runs opposite its sketch entity's natural
				// parameterization when its angle decreases. This records that
				// fact before sketch reports a BoundaryEdge.Reversed value.
				authoredReversed := w.IsCircular() && w.Th1 < w.Th0
				switch {
				case w.IsLine():
					start := reexpressPt(sectionrecord.Point2{U: w.StartU, V: w.StartV})
					end := reexpressPt(sectionrecord.Point2{U: w.EndU, V: w.EndV})
					key := entityKey{kind: 1, a: start, b: end}
					if _, ok := entities[key]; !ok {
						tag(s.CreateLine(point(start), point(end)), authoredReversed)
						entities[key] = struct{}{}
					}
				case w.IsCircular() && w.Closed:
					center := reexpressPt(sectionrecord.Point2{U: w.CU, V: w.CV})
					key := entityKey{kind: 2, a: center, radius: w.Radius}
					if _, ok := entities[key]; !ok {
						tag(s.CreateCircle(point(center), w.Radius), authoredReversed)
						entities[key] = struct{}{}
					}
				case w.IsCircular():
					// sketch.CreateArc sweeps CCW from its second point to its
					// third. Pass the endpoints in ascending-angle order even
					// when the recorded walk runs the other way.
					loU, loV, hiU, hiV := w.StartU, w.StartV, w.EndU, w.EndV
					if w.Th1 < w.Th0 {
						loU, loV, hiU, hiV = hiU, hiV, loU, loV
					}
					center := reexpressPt(sectionrecord.Point2{U: w.CU, V: w.CV})
					lo := reexpressPt(sectionrecord.Point2{U: loU, V: loV})
					hi := reexpressPt(sectionrecord.Point2{U: hiU, V: hiV})
					key := entityKey{kind: 3, a: center, b: lo, c: hi}
					if _, ok := entities[key]; !ok {
						tag(s.CreateArc(point(center), point(lo), point(hi)), authoredReversed)
						entities[key] = struct{}{}
					}
				default:
					return fmt.Errorf(`%w: a %T segment is not part of the admitted class`, decaderr.ErrUnsupported, seg)
				}
			}
		}
		return nil
	}

	for r, region := range regionsA {
		if err := addOperand(region, false, r, nil); err != nil {
			return nil, nil, SceneWalkCharge{}, err
		}
	}
	if !reexpress.Reflection() {
		for r, region := range regionsB {
			if err := addOperand(region, true, r, reexpress.MapPoint); err != nil {
				return nil, nil, SceneWalkCharge{}, err
			}
		}
		return s, tags, sceneDelta, nil
	}
	// A reflection reverses each loop's winding. Map and rewind B's
	// records before creating entities or recording authored direction.
	for r, region := range regionsB {
		rewound, walkCharge, err := reexpress.Rewind(budget, region)
		if err != nil {
			return nil, nil, SceneWalkCharge{}, err
		}
		sceneDelta.B = math.Max(sceneDelta.B, walkCharge)
		if err := addOperand(rewound, true, r, nil); err != nil {
			return nil, nil, SceneWalkCharge{}, err
		}
	}
	return s, tags, sceneDelta, nil
}
