package coilshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// WallAreas encloses every recorded segment's wall area (Table CM's
// Area row), in rec.Profile.Segments order, each widened by the denoted
// map's defect: a line's closed form over its two ends, an arc's or a
// circle's §11.1 bracket.
func WallAreas(ctx context.Context, rec Record) ([]coil.Iv, error) {
	segs := rec.Profile.Segments
	walls := make([]coil.Iv, len(segs))
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if seg.IsArc() {
			a, ok, err := coil.ArcArea(ctx, seg, rec.Axis, rec.Pitch, rec.Turns)
			if err != nil {
				return nil, err
			}
			if !ok {
				return nil, fmt.Errorf(`%w: the coil wall of profile loop %d segment %d has no area enclosure`, decaderr.ErrUnsupported, seg.Loop, seg.Index)
			}
			walls[i] = rec.Stretched(a)
			continue
		}
		v, w := seg.First, rec.Profile.Segments[rec.Profile.Next(i)].First
		a, ok := coil.SegmentArea(rec.Rho[v], rec.Zeta[v], rec.Rho[w], rec.Zeta[w], rec.Pitch, rec.Turns)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil wall of profile loop %d segment %d has no area enclosure`, decaderr.ErrUnsupported, seg.Loop, seg.Index)
		}
		walls[i] = rec.Stretched(a)
	}
	return walls, nil
}
