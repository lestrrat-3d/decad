package sketchrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// RecordProfileLoops records a fresh arrangement. The caller constructs the
// root ProfileRecord, whose measurement methods belong to that package.
func RecordProfileLoops(p *sketch.Profile) (LoopRecord, []LoopRecord, error) {
	if !p.Valid {
		return LoopRecord{}, nil, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, decaderr.ErrInvalidProfile)
	}
	outer, err := recordLoop("outer", p.Outer)
	if err != nil {
		return LoopRecord{}, nil, err
	}
	var holes []LoopRecord
	for i, h := range p.Holes {
		loop, err := recordLoop(fmt.Sprintf("hole %d", i), h)
		if err != nil {
			return LoopRecord{}, nil, err
		}
		holes = append(holes, loop)
	}
	return outer, holes, nil
}

// RecordChain authenticates and records one open sketch walk and its plane.
func RecordChain(s *sketch.Sketch, ch *sketch.Chain) (sectionrecord.ChainRecord, sectionrecord.PlaneRecord, error) {
	trusted, err := admitChain(s, ch)
	if err != nil {
		return sectionrecord.ChainRecord{}, sectionrecord.PlaneRecord{}, err
	}
	frame, err := s.Plane().Frame()
	if err != nil {
		return sectionrecord.ChainRecord{}, sectionrecord.PlaneRecord{},
			fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := sectionrecord.PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	segs, err := recordChainSegments(trusted.Edges)
	if err != nil {
		return sectionrecord.ChainRecord{}, sectionrecord.PlaneRecord{}, err
	}
	return sectionrecord.ChainRecord{Segments: segs}, plane, nil
}
