package tessellation

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CapBlendChordInput names the resolved contour and its common chord budget.
type CapBlendChordInput struct {
	Walks            []survey2d.SideWalk
	Joins            []CapBlendJoin
	Whole, Chamfered bool
	D, Chord         float64
}

// CapBlendChords records one count per wall shared by every side and cap ring.
type CapBlendChords struct {
	Count                     []int
	SideSag, CapSag           []float64
	CapRadius, CapTh0, CapTh1 []float64
	ArcCount                  []int
	ArcSag, ArcTh0, ArcTh1    []float64
	LocusGap                  []float64
}

// ChordCapBlendLoop chooses the side, cap, and reflex connector counts in the
// same walk order as the resolved contour. The caller supplies the cap radius,
// offset window, and miter-locus reading used by the analytic body builder.
func ChordCapBlendLoop(in CapBlendChordInput, budget *proofbound.WorkBudget,
	radius func(survey2d.SideWalk, float64) (float64, error),
	wallSweep func(float64, float64, sectionrecord.Point2, sectionrecord.Point2, float64) (float64, float64, int),
	locusGap func(int) (float64, error),
) (CapBlendChords, error) {
	n := len(in.Walks)
	out := CapBlendChords{
		Count: make([]int, n), SideSag: make([]float64, n), CapSag: make([]float64, n),
		CapRadius: make([]float64, n), CapTh0: make([]float64, n), CapTh1: make([]float64, n),
		ArcCount: make([]int, n), ArcSag: make([]float64, n),
		ArcTh0: make([]float64, n), ArcTh1: make([]float64, n), LocusGap: make([]float64, n),
	}
	for i, w := range in.Walks {
		if err := budget.Step(); err != nil {
			return CapBlendChords{}, err
		}
		if !w.IsCircular() {
			if !w.IsLine() {
				return CapBlendChords{}, fmt.Errorf(`%w: chording a cap-loop chamfer does not support walk kind %d`, decaderr.ErrUnsupported, w.Kind)
			}
			out.Count[i] = 1
			continue
		}
		// A whole closed wall takes the three-chord minimum from ChordWalkMin.
		nSide, _, err := ChordCount(w.SegmentWalk, in.Chord, ChordWalkMin(w.SegmentWalk))
		if err != nil {
			return CapBlendChords{}, err
		}
		count := nSide
		if in.Chamfered {
			r, err := radius(w, in.D)
			if err != nil {
				return CapBlendChords{}, err
			}
			out.CapRadius[i] = r
			out.CapTh0[i], out.CapTh1[i] = w.Th0, w.Th1
			if !in.Whole {
				start, end := capBlendWallFoot(in.Joins, i, n)
				out.CapTh0[i], out.CapTh1[i], _ = wallSweep(w.CU, w.CV, start, end, w.Th1-w.Th0)
			}
			capWalk := survey2d.SegmentWalk{Kind: survey2d.WalkCircular, Radius: r,
				Th0: out.CapTh0[i], Th1: out.CapTh1[i], Closed: w.Closed}
			nCap, _, err := ChordCount(capWalk, in.Chord, ChordWalkMin(capWalk))
			if err != nil {
				return CapBlendChords{}, err
			}
			count = max(nSide, nCap)
		}
		out.Count[i] = count
		out.SideSag[i] = ChordSagitta(w.Radius, math.Abs(w.Th1-w.Th0), count)
		if in.Chamfered {
			out.CapSag[i] = ChordSagitta(out.CapRadius[i], math.Abs(out.CapTh1[i]-out.CapTh0[i]), count)
		}
	}
	for i := range n {
		if in.Joins == nil {
			break
		}
		if err := budget.Step(); err != nil {
			return CapBlendChords{}, err
		}
		j := in.Joins[i]
		if !j.Arc {
			gap, err := locusGap(i)
			if err != nil {
				return CapBlendChords{}, err
			}
			out.LocusGap[i] = gap
			continue
		}
		// Reflex connectors turn clockwise between distinct offset feet.
		th0 := math.Atan2(j.PA.V-j.VV, j.PA.U-j.VU)
		th1 := math.Atan2(j.PB.V-j.VV, j.PB.U-j.VU)
		for th1 > th0 {
			th1 -= 2 * math.Pi
		}
		connector := survey2d.SegmentWalk{Kind: survey2d.WalkCircular, Radius: in.D, Th0: th0, Th1: th1}
		cnt, _, err := ChordCount(connector, in.Chord, 1)
		if err != nil {
			return CapBlendChords{}, err
		}
		out.ArcCount[i] = cnt
		out.ArcTh0[i], out.ArcTh1[i] = th0, th1
		out.ArcSag[i] = ChordSagitta(in.D, th0-th1, cnt)
	}
	return out, nil
}

func capBlendWallFoot(joins []CapBlendJoin, i, n int) (sectionrecord.Point2, sectionrecord.Point2) {
	j0 := joins[i]
	start := j0.M
	if j0.Arc {
		start = j0.PB
	}
	j1 := joins[(i+1)%n]
	end := j1.M
	if j1.Arc {
		end = j1.PA
	}
	return start, end
}
