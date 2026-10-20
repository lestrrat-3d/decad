package prismshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ShellWallBands lists the bands a shell's wall section holds between the
// outer region O and the cavity region C: the band between O's outer loop and
// C's (C's outer reversed as its one hole), then, for each hole i of O, the
// band between C's hole i (reversed, an outer) and O's hole i. Inward O is the
// receiver's section P and C its erosion; outward O is the dilation and C is
// P. The §5 audit has proven C's loops simple and nested in O's, so the bands
// are regular and pairwise disjoint (agent-docs/modify-reach-design.md §9.2).
func ShellWallBands(ctx context.Context, outer, cavity momentinput.Profile) ([]momentinput.Profile, error) {
	if len(outer.Holes) != len(cavity.Holes) {
		return nil, fmt.Errorf(`%w: the shell's outer and cavity regions have different loop counts`, decaderr.ErrDegenerate)
	}
	cavityOuter, err := offset2d.ReverseLoopRecordContext(ctx, cavity.Outer)
	if err != nil {
		return nil, err
	}
	bands := []momentinput.Profile{{Outer: outer.Outer, Holes: []sectionrecord.LoopRecord{cavityOuter}}}
	for i, hole := range cavity.Holes {
		post, err := offset2d.ReverseLoopRecordContext(ctx, hole)
		if err != nil {
			return nil, err
		}
		bands = append(bands, momentinput.Profile{Outer: post, Holes: []sectionrecord.LoopRecord{outer.Holes[i]}})
	}
	return bands, nil
}
