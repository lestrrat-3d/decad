package sweepmitre

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Loops returns the recorded vertices and loop indexes for whole LineSeg
// boundaries. A trimmed line or mismatched pair of endpoints has no exact
// recorded polygon vertex under the mitred construction.
func Loops(outer sectionrecord.LoopRecord, holes []sectionrecord.LoopRecord) ([]sectionrecord.Point2, [][]int, error) {
	loops := append([]sectionrecord.LoopRecord{outer}, holes...)
	var pts []sectionrecord.Point2
	var loopIdx [][]int
	for i, loop := range loops {
		n := len(loop.Segments)
		starts := make([]sectionrecord.Point2, n)
		ends := make([]sectionrecord.Point2, n)
		for j, raw := range loop.Segments {
			segment, err := sectionrecord.NormalizeSegment(raw)
			if err != nil {
				return nil, nil, err
			}
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok {
				return nil, nil, fmt.Errorf(`%w: a mitred or scaled sweep supports line profile segments only; loop %d segment %d is %T`,
					decaderr.ErrUnsupported, i, j, segment)
			}
			switch {
			case line.TStart == 0 && line.TEnd == 1:
				starts[j], ends[j] = line.Start, line.End
			case line.TStart == 1 && line.TEnd == 0:
				starts[j], ends[j] = line.End, line.Start
			default:
				return nil, nil, fmt.Errorf(`%w: a mitred or scaled sweep needs whole profile lines; loop %d segment %d is trimmed`,
					decaderr.ErrUnsupported, i, j)
			}
		}
		idx := make([]int, n)
		for j := range n {
			if ends[j] != starts[(j+1)%n] {
				return nil, nil, fmt.Errorf(`%w: loop %d segment %d does not end exactly where the next one starts`,
					decaderr.ErrUnsupported, i, j)
			}
			idx[j] = len(pts)
			pts = append(pts, starts[j])
		}
		loopIdx = append(loopIdx, idx)
	}
	return pts, loopIdx, nil
}

// Preflight applies SM10's span and facet-pair ceilings from counts alone.
func Preflight(loopIdx [][]int, spans, spanCap int) error {
	if spans > spanCap {
		return fmt.Errorf(`%w: a mitred sweep exceeds the fixed span ceiling of %d`, decaderr.ErrUnsupported, spanCap)
	}
	segments := uint64(0)
	for _, idx := range loopIdx {
		segments += uint64(len(idx))
	}
	holes := uint64(len(loopIdx) - 1)
	// A polygon with h holes and S boundary vertices triangulates into
	// S + 2h − 2 triangles; each cap holds one such triangulation.
	if segments > math.MaxUint32 {
		return fmt.Errorf(`%w: the mitred sweep's triangle count overflows`, decaderr.ErrUnsupported)
	}
	capTris := segments + 2*holes - 2
	walls := 2 * uint64(spans) * segments
	pairs, ok := proofbound.WallChoose2(walls + 2*capTris)
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the mitred sweep crossing audit exceeds its fixed facet-pair ceiling`, decaderr.ErrUnsupported)
	}
	return nil
}
