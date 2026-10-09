package throughshell

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Removal is the removed caps and outer-section walls admitted by SG4 and SG5.
type Removal struct {
	Bottom, Top bool
	Walls       []int
	Sides       map[int]struct{}
}

// KeptCaps counts the caps that remain after the shell's openings are removed.
func (rm Removal) KeptCaps() int {
	n := 0
	for _, removed := range []bool{rm.Bottom, rm.Top} {
		if !removed {
			n++
		}
	}
	return n
}

// ReadRemoval classifies selected face indices against the through-cut record.
// An index of -1 means the selected live face names no face in that record.
// A removed wall must claim one straight outer-section side; connected-run
// admission remains with the shell opening builder.
func ReadRemoval(faces []brepgeom.PrismRectFace, embeds []brepgeom.Embed,
	kinds []brepgeom.ThroughFace, caps brepgeom.PrismCaps, axis int, indices []int) (Removal, error) {
	rm := Removal{Sides: map[int]struct{}{}}
	work := freeform.NewFreeformWork()
	for _, fi := range indices {
		switch {
		case fi < 0 || fi >= len(faces):
			return Removal{}, fmt.Errorf(`%w: this evaluator shells a through-cut record by removing its caps or a run of its walls, and a removed face names no face of its record (modify-general SG5)`, decaderr.ErrUnsupported)
		case fi == caps.Bottom:
			rm.Bottom = true
			continue
		case fi == caps.Top:
			rm.Top = true
			continue
		case kinds[fi] == brepgeom.ThroughTool || kinds[fi] == brepgeom.ThroughHoleWall:
			return Removal{}, holeRemovedError(faces[fi].Role)
		}
		si, ok := claimedSide(faces[fi], embeds[fi], caps, axis, work)
		if !ok {
			return Removal{}, holeRemovedError(faces[fi].Role)
		}
		seg := caps.Section.Outer.Segments[si]
		if from, to, ok := brepgeom.NaturalLine(seg); !ok || (from.U == to.U) == (from.V == to.V) {
			return Removal{}, fmt.Errorf(`%w: this evaluator removes a wall of a through-cut record only where it is a straight wall along a section axis, whose rim is a planar face; %s is not (modify-general SG5)`, decaderr.ErrUnsupported, faces[fi].Role)
		}
		rm.Walls = append(rm.Walls, fi)
		rm.Sides[si] = struct{}{}
	}
	return rm, nil
}

func holeRemovedError(role string) error {
	return fmt.Errorf(`%w: this evaluator shells a through-cut record by removing its caps or a run of its walls, and %s is a wall of a through tool or of a hole of the section (modify-general SG4)`, decaderr.ErrUnsupported, role)
}

// claimedSide reads a wall's claim on one segment of the section's outer loop.
func claimedSide(f brepgeom.PrismRectFace, e brepgeom.Embed, caps brepgeom.PrismCaps,
	axis int, work *freeform.FreeformWork) (int, bool) {
	var match func(survey2d.SegmentWalk) bool
	if f.Region != nil {
		outer := *f.Region
		outer.Holes = nil
		face := brepgeom.PrismRectFace{Region: &outer, Z0: f.Z0, Z1: f.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Outward: f.Outward}
		rect, ok := brepgeom.PlanarPrismRect(face, e, caps.Embed, axis, caps.Zlo, caps.Zhi)
		if !ok {
			return 0, false
		}
		match = rect.Matches
	} else {
		seg, ok := brepgeom.NewPrismMap(e, caps.Embed).Segment(f.Wall)
		if !ok {
			return 0, false
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return 0, false
		}
		key := brepgeom.WalkKeyOf(w)
		match = func(s survey2d.SegmentWalk) bool { return brepgeom.WalkKeyOf(s) == key }
	}
	for si, seg := range caps.Section.Outer.Segments {
		w, err := boundarywalk.WalkOf(seg, work)
		if err == nil && match(w) {
			return si, true
		}
	}
	return 0, false
}
