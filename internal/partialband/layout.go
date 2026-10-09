// Package partialband lays out selected cap-edge fillet band mesh samples.
package partialband

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// PartialBandWalk records the columns and sagitta of one selected side walk.
type PartialBandWalk struct {
	Index int
	Cols  []int
	Sag   float64
}

// PartialBandColumn pairs one side sample with its cap sample and bounds.
type PartialBandColumn struct {
	Side, Cap sectionrecord.Point2
	SideBound proofbound.WalkEndBound
	CapBound  proofbound.WalkEndBound
}

// PartialBandLayout holds the selected walks' side and cap columns. Adjacent
// walks share mesh vertices when the caller places equal column coordinates.
type PartialBandLayout struct {
	Walks    []PartialBandWalk
	Columns  []PartialBandColumn
	Cells    []tessellation.FilletCell
	PatchSag []float64
	ArcCols  map[int][]int
}

// PartialFilletLayout chords selected side walks and their cap connectors.
// A selected sphere walk collapses to its pole on the cap contour.
func PartialFilletLayout(walks []survey2d.SideWalk, orig, contour []sectionrecord.CurveSegment,
	selected []bool, capWalk, capArc []int, setback, chord float64, work *freeform.FreeformWork,
) (*PartialBandLayout, error) {
	if len(walks) != len(selected) {
		return nil, fmt.Errorf(`%w: a partial fillet band's side walks disagree with its selection`, decaderr.ErrUnsupported)
	}
	partial := &PartialBandLayout{ArcCols: map[int][]int{}}
	walkCols := make([][2]int, len(selected))
	for i, on := range selected {
		if !on {
			continue
		}
		side := walks[i]
		if capWalk[i] < 0 {
			if !filletband.SphereWalk(side, setback) {
				return nil, fmt.Errorf(`%w: a partial fillet band's selected cap segment is missing`, decaderr.ErrUnsupported)
			}
			segment := orig[i]
			count, sag, err := tessellation.ChordCount(side.SegmentWalk, chord,
				tessellation.ChordWalkMin(side.SegmentWalk))
			if err != nil {
				return nil, err
			}
			cols := make([]int, count+1)
			pole := sectionrecord.Point2{U: side.CU, V: side.CV}
			dth := (side.Th1 - side.Th0) / float64(count)
			for j := range cols {
				theta := side.Th0 + float64(j)*dth
				point := sectionrecord.Point2{U: side.CU + side.Radius*math.Cos(theta),
					V: side.CV + side.Radius*math.Sin(theta)}
				switch j {
				case 0:
					point = sectionrecord.Point2{U: side.StartU, V: side.StartV}
				case count:
					point = sectionrecord.Point2{U: side.EndU, V: side.EndV}
				}
				cols[j] = len(partial.Columns)
				bound := stationbound.ChordStationBound(segment, j, count, point.U, point.V)
				switch j {
				case 0:
					bound = boundarywalk.DenotedStartBound(segment, side.SegmentWalk)
				case count:
					bound = boundarywalk.DenotedEndBound(segment, side.SegmentWalk)
				}
				partial.Columns = append(partial.Columns, PartialBandColumn{Side: point, Cap: pole, SideBound: bound})
			}
			walkCols[i] = [2]int{cols[0], cols[count]}
			partial.Walks = append(partial.Walks, PartialBandWalk{Index: i, Cols: cols, Sag: sag})
			continue
		}
		if capWalk[i] >= len(contour) {
			return nil, fmt.Errorf(`%w: a partial fillet band's selected cap segment is missing`, decaderr.ErrUnsupported)
		}
		cap, err := boundarywalk.WalkOf(contour[capWalk[i]], work)
		if err != nil {
			return nil, err
		}
		if !side.IsLine() || !cap.IsLine() {
			return nil, fmt.Errorf(`%w: a selected partial fillet walk is curved`, decaderr.ErrUnsupported)
		}
		walkCols[i] = [2]int{len(partial.Columns), len(partial.Columns) + 1}
		partial.Columns = append(partial.Columns,
			PartialBandColumn{Side: sectionrecord.Point2{U: side.StartU, V: side.StartV},
				Cap: sectionrecord.Point2{U: cap.StartU, V: cap.StartV}},
			PartialBandColumn{Side: sectionrecord.Point2{U: side.EndU, V: side.EndV},
				Cap: sectionrecord.Point2{U: cap.EndU, V: cap.EndV}})
		partial.Walks = append(partial.Walks, PartialBandWalk{Index: i, Cols: []int{walkCols[i][0], walkCols[i][1]}})
	}
	if len(partial.Walks) == 0 {
		return nil, fmt.Errorf(`%w: a partial fillet band selects no walks`, decaderr.ErrUnsupported)
	}
	for _, walk := range partial.Walks {
		i := walk.Index
		for j := range len(walk.Cols) - 1 {
			partial.Cells = append(partial.Cells, tessellation.FilletCell{
				C0: walk.Cols[j], C1: walk.Cols[j+1], Patch: len(partial.PatchSag)})
		}
		partial.PatchSag = append(partial.PatchSag, walk.Sag)
		k := (i + 1) % len(selected)
		if capArc[k] < 0 {
			continue
		}
		if capArc[k] >= len(contour) || !selected[k] {
			return nil, fmt.Errorf(`%w: a partial fillet band's reflex connector is missing`, decaderr.ErrUnsupported)
		}
		segment := contour[capArc[k]]
		arc, err := boundarywalk.WalkOf(segment, work)
		if err != nil {
			return nil, err
		}
		if !arc.IsCircular() || arc.Closed {
			return nil, fmt.Errorf(`%w: a partial fillet's connector is not an open arc`, decaderr.ErrUnsupported)
		}
		count, sag, err := tessellation.ChordCount(arc, chord/2, 1)
		if err != nil {
			return nil, err
		}
		cols := []int{walkCols[i][1]}
		corner := walks[k]
		for j := 1; j < count; j++ {
			theta := arc.Th0 + (arc.Th1-arc.Th0)*float64(j)/float64(count)
			point := sectionrecord.Point2{U: arc.CU + arc.Radius*math.Cos(theta),
				V: arc.CV + arc.Radius*math.Sin(theta)}
			cols = append(cols, len(partial.Columns))
			partial.Columns = append(partial.Columns, PartialBandColumn{
				Side: sectionrecord.Point2{U: corner.StartU, V: corner.StartV}, Cap: point,
				CapBound: stationbound.ChordStationBound(segment, j, count, point.U, point.V)})
		}
		cols = append(cols, walkCols[k][0])
		partial.ArcCols[capArc[k]] = cols
		for j := range len(cols) - 1 {
			partial.Cells = append(partial.Cells, tessellation.FilletCell{
				C0: cols[j], C1: cols[j+1], Patch: len(partial.PatchSag)})
		}
		partial.PatchSag = append(partial.PatchSag, sag)
	}
	return partial, nil
}
