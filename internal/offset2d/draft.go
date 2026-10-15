package offset2d

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// DraftWall names one recorded segment of a draft's near section: its loop
// (0 the outer loop, i+1 hole i, the index side(i, j) roles carry) and its
// index in that loop.
type DraftWall struct{ Loop, Seg int }

// DraftWalkAmounts is the offset amount of each coalesced walk of loop li: t for a
// walk the draft moves and zero for one it keeps (kept). A walk is kept when
// its first recorded segment is; DraftSectionWalks refuses a walk whose
// segments disagree before any amount is read.
func DraftWalkAmounts(kept map[DraftWall]struct{}, li int, walks []survey2d.SideWalk, t float64) []float64 {
	out := UniformAmounts(len(walks), t)
	if len(kept) == 0 {
		return out
	}
	for i, w := range walks {
		if _, ok := kept[DraftWall{Loop: li, Seg: w.Segs[0]}]; ok {
			out[i] = 0
		}
	}
	return out
}

// DraftSectionWalks is stage 3: every loop of the section resolved into its
// coalesced walks, each walk analytic (SD3), and every corner classified by the
// sharp rule at a unit offset, each walk at its own amount (SD4 for a corner at
// a circular walk that is not a G1 join or whose two walks move differently,
// SD15 for a cusp). A walk the draft moves reads 1 and a kept walk 0
// (DraftWalkAmounts), and a walk whose recorded segments are part kept and part
// moved names no one wall to tilt. The classification does not depend on the
// amount's size, so stage 4 reads the same corners at d.
func DraftSectionWalks(budget *proofbound.WorkBudget, profile momentinput.Profile, kept map[DraftWall]struct{}, work *freeform.FreeformWork, tol float64) ([][]survey2d.SideWalk, error) {
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	out := make([][]survey2d.SideWalk, len(loops))
	for li, loop := range loops {
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := boundarywalk.RequireAnalyticWalk(w, "a tapered extrude"); err != nil {
				return nil, fmt.Errorf(`%w; the offset of a free-form wall is not a recorded kind (draft SD3)`, err)
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		}
		walks, err := boundarywalk.CoalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		if len(walks) == 0 {
			return nil, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
		}
		if err := requireWholeKeptWalks(kept, li, walks); err != nil {
			return nil, err
		}
		if len(walks) > 1 || !walks[0].Closed {
			if _, err := SharpJoinsBudget(budget, walks, DraftWalkAmounts(kept, li, walks, 1), tol); err != nil {
				return nil, DraftOffsetError(InLoop(err, li))
			}
		}
		out[li] = walks
	}
	return out, nil
}

// requireWholeKeptWalks refuses a coalesced walk of loop li whose recorded
// segments a subset draft partly keeps and partly moves: a walk is one wall
// face, so it either tilts or stays.
func requireWholeKeptWalks(kept map[DraftWall]struct{}, li int, walks []survey2d.SideWalk) error {
	if len(kept) == 0 {
		return nil
	}
	for _, w := range walks {
		n := 0
		for _, j := range w.Segs {
			if _, ok := kept[DraftWall{Loop: li, Seg: j}]; ok {
				n++
			}
		}
		if n != 0 && n != len(w.Segs) {
			return fmt.Errorf(`%w: the draft selects part of one wall of loop %d (recorded segments %v form one face), so it names no single wall to tilt (draft SD21)`, decaderr.ErrUnsupported, li, w.Segs)
		}
	}
	return nil
}

// DraftFarSection is stage 4: every loop offset sharply, each walk by its own
// amount (t, or zero for a wall a subset draft keeps), its walk sense kept
// (BuildSharpLoop). A circular wall whose radius collapses is SD5 and
// a wall its own corners consume is SD7, both ErrUnsupported; an outer loop
// shrunk until every one of its walls is consumed is SD6, ErrDegenerate,
// because the taper has consumed the region before the far end. Then SD13: an
// amount that rounds to zero, a far corner of a moved wall bit-identical to
// its near corner, or a moved far radius bit-identical to its near radius names
// a far section float64 cannot tell from the near one. A corner between two
// kept walls stays where it is by construction.
func DraftFarSection(budget *proofbound.WorkBudget, walks [][]survey2d.SideWalk, kept map[DraftWall]struct{}, t, tol float64) (momentinput.Profile, error) {
	if t == 0 {
		return momentinput.Profile{}, errDraftAmountRoundsAway(t)
	}
	loops := make([]sectionrecord.LoopRecord, len(walks))
	for li, ws := range walks {
		amounts := DraftWalkAmounts(kept, li, ws, t)
		segs, joins, err := BuildSharpLoop(budget, ws, amounts, tol)
		if errors.Is(err, ErrLoopConsumed) && li == 0 && t > 0 {
			return momentinput.Profile{}, fmt.Errorf(`%w: the taper offsets every wall of the outer loop past its neighbours before the far end, so the region is consumed and no solid reaches that far; a smaller taper or a shorter sweep states a body (draft SD6)`, decaderr.ErrDegenerate)
		}
		if err != nil {
			return momentinput.Profile{}, DraftOffsetError(InLoop(err, li))
		}
		n := len(ws)
		for i, j := range joins {
			if amounts[i] == 0 && amounts[(i+n-1)%n] == 0 {
				continue
			}
			if j.M.U == j.VertU && j.M.V == j.VertV {
				return momentinput.Profile{}, errDraftAmountRoundsAway(t)
			}
		}
		for i, w := range ws {
			if !w.IsCircular() || amounts[i] == 0 {
				continue
			}
			if rr, ok := OffsetRadius(w, 1, amounts[i], tol); ok && rr == w.Radius {
				return momentinput.Profile{}, errDraftAmountRoundsAway(t)
			}
		}
		loops[li] = sectionrecord.LoopRecord{Segments: segs}
	}
	return momentinput.Profile{Outer: loops[0], Holes: loops[1:]}, nil
}

// errDraftAmountRoundsAway is SD13: the requested body exists, a real if tiny
// taper, but float64 cannot name its far section at the section's own scale,
// and a far section equal to the near one would publish walls with no lean.
func errDraftAmountRoundsAway(t float64) error {
	return fmt.Errorf(`%w: the taper's offset amount %v mm is below the float64 spacing of the section's own coordinates, so the far section rounds back onto the near one; a larger taper or a longer sweep states a draft this evaluator can build (draft SD13)`, decaderr.ErrUnsupported, t)
}

// DraftSharpJoins is the draft band's corner rule
// (capBlendPayload.offsetJoins): every corner of one coalesced loop resolved
// by SharpJoinsBudget, walk i moved amounts[i], in walk order, as the
// Join records capblend.BuildBand reads. No join is an arc.
func DraftSharpJoins(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, amounts []float64, tol float64) ([]Join, error) {
	result, err := SharpJoinsBudget(budget, walks, amounts, tol)
	if err != nil {
		return nil, DraftOffsetError(err)
	}
	return result, nil
}

// DraftOffsetError states an offset refusal as the Table SD row it is.
// Each keeps the sentinel its row decides: SD4 and SD15 are ErrUnsupported, as
// are SD5 and SD7, which ErrDrop already carries.
func DraftOffsetError(err error) error {
	switch {
	case errors.Is(err, ErrMixedCircularCorner):
		return fmt.Errorf(`%w: the draft moves one of two walls that meet at a circular wall's corner and keeps the other, so the moved wall and the kept one no longer meet tangentially and their corner moves along a conic, which no line-and-arc far section records; select both walls or neither (draft SD4)`, decaderr.ErrUnsupported)
	case errors.Is(err, ErrCircularMiter):
		return fmt.Errorf(`%w: a circular wall meets its neighbour other than tangentially, and that corner moves along a conic as the taper offsets it, which no line-and-arc far section records; join the wall tangentially (draft SD4)`, decaderr.ErrUnsupported)
	case errors.Is(err, ErrTopology):
		return fmt.Errorf(`%w; the two walls meet at a cusp (draft SD15)`, err)
	case errors.Is(err, ErrLoopConsumed):
		return fmt.Errorf(`%w: the taper consumes every wall of a loop before the far end, so the section changes topology there (draft SD7)`, decaderr.ErrUnsupported)
	case errors.Is(err, ErrDrop):
		return fmt.Errorf(`%w; the taper's far section loses a circular wall whose radius collapses (draft SD5) or a wall its own corners consume (draft SD7)`, err)
	}
	return err
}

// DraftAuditError names the far section as the section the shared offset
// audit refused, keeping the audit's sentinel: S8's ErrDegenerate (SD6),
// the crossing test's ErrUnsupported (SD8) and the nesting rows' (SD9).
func DraftAuditError(err error) error {
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%w; the section that failed is the tapered extrude's far section (draft SD6, SD8, SD9)`, err)
}
