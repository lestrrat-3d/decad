package coil

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Loops returns the recorded polygon of a LineSeg-only profile: every
// vertex once, loop-major (outer loop first, then each hole), and per loop
// the vertex indices in walk order (docs/helix-design.md Table CS row CS7).
// A segment that is not a LineSeg is ErrUnsupported, as is a trimmed line or
// a loop whose segments do not meet exactly: neither has an exact recorded
// polygon vertex for the held station-0 section to hold.
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
				return nil, nil, fmt.Errorf(`%w: a coil sweeps line profile segments only; loop %d segment %d is %T`,
					decaderr.ErrUnsupported, i, j, segment)
			}
			switch {
			case line.TStart == 0 && line.TEnd == 1:
				starts[j], ends[j] = line.Start, line.End
			case line.TStart == 1 && line.TEnd == 0:
				starts[j], ends[j] = line.End, line.Start
			default:
				return nil, nil, fmt.Errorf(`%w: a coil sweeps whole profile lines only; loop %d segment %d is trimmed`,
					decaderr.ErrUnsupported, i, j)
			}
		}
		idx := make([]int, n)
		for j := range n {
			if ends[j] != starts[(j+1)%n] {
				return nil, nil, fmt.Errorf(`%w: coil profile loop %d segment %d does not end exactly where the next one starts`,
					decaderr.ErrUnsupported, i, j)
			}
			idx[j] = len(pts)
			pts = append(pts, starts[j])
		}
		loopIdx = append(loopIdx, idx)
	}
	return pts, loopIdx, nil
}
