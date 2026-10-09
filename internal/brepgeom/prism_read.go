package brepgeom

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// PrismCaps is the cap and section reading shared by route P and route S.
type PrismCaps struct {
	Bottom, Top int
	Zlo, Zhi    float64
	Frame       r3.Frame
	Embed       Embed
	Section     momentinput.Profile
	BottomLoop  []int
}

// ReadPrismCaps checks P1, P3 and P4 for a reference axis without reading walls.
func ReadPrismCaps(faces []PrismRectFace, embeds []Embed, axis int) (PrismCaps, bool) {
	bottom, top := -1, -1
	planar := 0
	for fi, f := range faces {
		if f.Region == nil || embeds[fi].Axis[2] != axis {
			continue
		}
		planar++
		if prismOutwardSign(f.Outward, embeds[fi]) < 0 {
			bottom = fi
		} else {
			top = fi
		}
	}
	if planar != 2 || bottom < 0 || top < 0 {
		return PrismCaps{}, false
	}
	zlo := embeds[bottom].Sign[2]*faces[bottom].Z0 + 0
	zhi := embeds[top].Sign[2]*faces[top].Z0 + 0
	if !(zlo < zhi) {
		return PrismCaps{}, false
	}
	var frame r3.Frame
	var eF Embed
	switch {
	case embeds[top].Sign[2] > 0:
		frame, eF = faces[top].Frame, embeds[top]
	case embeds[bottom].Sign[2] > 0:
		frame, eF = faces[bottom].Frame, embeds[bottom]
	default:
		var err error
		frame, eF, err = AxisFrame(faces[0].Frame, axis)
		if err != nil {
			return PrismCaps{}, false
		}
	}
	section, ok := NewPrismMap(embeds[top], eF).Region(*faces[top].Region)
	if !ok {
		return PrismCaps{}, false
	}
	bottomRegion, ok := NewPrismMap(embeds[bottom], eF).Region(*faces[bottom].Region)
	if !ok {
		return PrismCaps{}, false
	}
	bottomLoop, ok := SameRegion(section, bottomRegion)
	if !ok {
		return PrismCaps{}, false
	}
	return PrismCaps{Bottom: bottom, Top: top, Zlo: zlo, Zhi: zhi,
		Frame: frame, Embed: eF, Section: section, BottomLoop: bottomLoop}, true
}

func prismOutwardSign(outward bool, e Embed) float64 {
	if outward {
		return e.Sign[2]
	}
	return -e.Sign[2]
}

// PrismWallSet collects P2's side walks and rectangles for P5's section claim.
type PrismWallSet struct {
	Walls              []survey2d.SegmentWalk
	Rects              []PrismRect
	ZloDelta, ZhiDelta float64
}

// Add reads one face as a wall along the axis or a rectangle across it.
func (w *PrismWallSet) Add(f PrismRectFace, e, section Embed, axis int, zlo, zhi float64,
	work *freeform.FreeformWork) bool {
	switch {
	case f.Region == nil && e.Axis[2] == axis:
		if f.Split0 || f.Split1 {
			return false
		}
		l0, l1 := e.Sign[2]*f.Z0+0, e.Sign[2]*f.Z1+0
		d0, d1 := f.Z0Delta, f.Z1Delta
		if l0 > l1 {
			l0, l1, d0, d1 = l1, l0, d1, d0
		}
		if l0 != zlo || l1 != zhi {
			return false
		}
		seg, ok := NewPrismMap(e, section).Segment(f.Wall)
		if !ok {
			return false
		}
		walk, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return false
		}
		w.Walls = append(w.Walls, walk)
		w.ZloDelta, w.ZhiDelta = max(w.ZloDelta, d0), max(w.ZhiDelta, d1)
		return true
	case f.Region != nil:
		rect, ok := PlanarPrismRect(f, e, section, axis, zlo, zhi)
		if ok {
			w.Rects = append(w.Rects, rect)
		}
		return ok
	default:
		rect, ok := SweptPrismRect(f, e, section, axis, zlo, zhi)
		if ok {
			w.Rects = append(w.Rects, rect)
		}
		return ok
	}
}

// ClaimsSection checks that every wall or rectangle claims exactly one section segment.
func (w *PrismWallSet) ClaimsSection(section momentinput.Profile, work *freeform.FreeformWork) bool {
	return PrismWallsClaim(w.Walls, w.Rects, section, work)
}

// ClassifyPrismWalls checks P2 and P5 over every face other than the caps.
// It reports false for an unrecognized record and returns only context errors.
func ClassifyPrismWalls(ctx context.Context, faces []PrismRectFace, embeds []Embed,
	axis int, caps PrismCaps) (PrismWallSet, bool, error) {
	var walls PrismWallSet
	work := freeform.NewFreeformWork()
	for fi, f := range faces {
		if err := ctx.Err(); err != nil {
			return PrismWallSet{}, false, err
		}
		if fi == caps.Bottom || fi == caps.Top {
			continue
		}
		if !walls.Add(f, embeds[fi], caps.Embed, axis, caps.Zlo, caps.Zhi, work) {
			return PrismWallSet{}, false, nil
		}
	}
	if !walls.ClaimsSection(caps.Section, work) {
		return PrismWallSet{}, false, nil
	}
	return walls, true, nil
}
