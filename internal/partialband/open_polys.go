package partialband

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Terminal names a fillet band's open terminal arc on another planar face.
type Terminal struct{ Face, Loop, Seg int }

// OpenPolyInput holds the recorded uses and shared ring stations needed to
// orient each terminal meridian, cap connector, and sphere side contour.
type OpenPolyInput struct {
	Layout     *PartialBandLayout
	Face, Loop int
	Orig       []sectionrecord.CurveSegment
	CapWalk    []int
	Terminals  []Terminal
	RingV      [][]int
	Canon      [][3]float64
	Uses       []brepgeom.Use
	FaceUses   [][]int
	Open       []int
	Embed      brepgeom.Embed
	SideZ      float64
}

// OpenPolys gives open band curves the stations of their adjacent patches in
// each recorded face's walk direction.
func OpenPolys(in OpenPolyInput) (map[int][]int, error) {
	out := map[int][]int{}
	lastRing := len(in.RingV) - 1
	for _, terminal := range in.Terminals {
		ui := -1
		for _, candidate := range in.FaceUses[terminal.Face] {
			use := in.Uses[candidate]
			if use.Part == brepgeom.LoopSeg && use.Loop == terminal.Loop && use.Seg == terminal.Seg {
				ui = candidate
				break
			}
		}
		if ui < 0 {
			return nil, fmt.Errorf(`%w: a partial fillet's terminal has no face use`, decaderr.ErrUnsupported)
		}
		use := in.Uses[ui]
		for c := range in.Layout.Columns {
			lo, hi := in.Canon[in.RingV[0][c]], in.Canon[in.RingV[lastRing][c]]
			if use.From != lo || use.To != hi {
				if use.From != hi || use.To != lo {
					continue
				}
			}
			poly := make([]int, len(in.RingV))
			for k := range poly {
				poly[k] = in.RingV[k][c]
			}
			if use.From == hi {
				slices.Reverse(poly)
			}
			out[ui] = poly
			break
		}
		if out[ui] == nil {
			return nil, fmt.Errorf(`%w: a partial fillet's terminal arc has no matching meridian`, decaderr.ErrUnsupported)
		}
	}
	for seg, cols := range in.Layout.ArcCols {
		ui := -1
		for _, candidate := range in.FaceUses[in.Face] {
			use := in.Uses[candidate]
			if use.Part == brepgeom.LoopSeg && use.Loop == in.Loop && use.Seg == seg {
				ui = candidate
				break
			}
		}
		if ui < 0 {
			return nil, fmt.Errorf(`%w: a partial fillet's cap connector has no face use`, decaderr.ErrUnsupported)
		}
		poly := make([]int, len(cols))
		for j, col := range cols {
			poly[j] = in.RingV[lastRing][col]
		}
		use := in.Uses[ui]
		switch {
		case in.Canon[poly[0]] == use.From && in.Canon[poly[len(poly)-1]] == use.To:
		case in.Canon[poly[0]] == use.To && in.Canon[poly[len(poly)-1]] == use.From:
			slices.Reverse(poly)
		default:
			return nil, fmt.Errorf(`%w: a partial fillet's cap connector disagrees with its samples`, decaderr.ErrUnsupported)
		}
		out[ui] = poly
	}
	work := freeform.NewFreeformWork()
	for _, walk := range in.Layout.Walks {
		if in.CapWalk[walk.Index] >= 0 {
			continue
		}
		view, err := boundarywalk.WalkOf(in.Orig[walk.Index], work)
		if err != nil {
			return nil, err
		}
		key, _ := brepgeom.CurveKey(in.Embed, view, in.SideZ)
		ui := -1
		for _, candidate := range in.Open {
			if in.Uses[candidate].Key == key {
				ui = candidate
				break
			}
		}
		if ui < 0 {
			return nil, fmt.Errorf(`%w: a partial fillet's sphere has no side contour use`, decaderr.ErrUnsupported)
		}
		poly := make([]int, len(walk.Cols))
		for j, col := range walk.Cols {
			poly[j] = in.RingV[0][col]
		}
		use := in.Uses[ui]
		switch {
		case in.Canon[poly[0]] == use.From && in.Canon[poly[len(poly)-1]] == use.To:
		case in.Canon[poly[0]] == use.To && in.Canon[poly[len(poly)-1]] == use.From:
			slices.Reverse(poly)
		default:
			return nil, fmt.Errorf(`%w: a partial fillet's sphere side disagrees with its samples`, decaderr.ErrUnsupported)
		}
		out[ui] = poly
	}
	return out, nil
}
