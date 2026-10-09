package brepgeom

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RestoreSideSegment moves a trimmed planar face segment back to its original
// reference level with the ends of its two neighbouring lines.
func RestoreSideSegment(region *momentinput.Profile, z0 float64, e Embed,
	li, seg, axis int, sideRef, levelRef float64) error {
	loop := region.Outer
	if li > 0 {
		loop = region.Holes[li-1]
	}
	count := len(loop.Segments)
	move := func(p sectionrecord.Point2) sectionrecord.Point2 {
		c := e.Canon(p.U, p.V, z0)
		if c[axis] != sideRef {
			return p
		}
		c[axis] = levelRef
		l := e.Local(c)
		return sectionrecord.Point2{U: l[0], V: l[1]}
	}
	for _, k := range [3]int{(seg + count - 1) % count, seg, (seg + 1) % count} {
		line, ok := loop.Segments[k].(sectionrecord.LineSeg)
		if !ok {
			return fmt.Errorf(`%w: a face beside a fillet band holds a curved segment where the band trimmed it`,
				decaderr.ErrUnsupported)
		}
		line.Start, line.End = move(line.Start), move(line.End)
		loop.Segments[k] = line
	}
	return nil
}
