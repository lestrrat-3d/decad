package wallsurvey

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/verifypublish"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvesurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// RevolveRecord is the meridian and sweep data needed by a wall survey.
type RevolveRecord struct {
	Profile      momentinput.Profile
	Axis         revolveaxis.Frame
	SectionDelta float64
	Full         bool
	Phi0, Phi1   float64
	AngularDelta float64
}

// RevolveLoops resolves recorded meridian loops into axis coordinates.
func RevolveLoops(budget *proofbound.WorkBudget, profile momentinput.Profile,
	axis revolveaxis.Frame) ([][]survey2d.SideWalk, error) {
	loops, _, err := RevolveLoopsPlane(budget, profile, axis)
	return loops, err
}

// RevolveLoopsPlane also retains each loop's plane-local walks, indexed by
// recorded segment. One free-form work counter covers the whole record.
func RevolveLoopsPlane(budget *proofbound.WorkBudget, profile momentinput.Profile,
	axis revolveaxis.Frame) ([][]survey2d.SideWalk, [][]survey2d.SegmentWalk, error) {
	work := freeform.NewFreeformWork()
	var out [][]survey2d.SideWalk
	var planes [][]survey2d.SegmentWalk
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, err
		}
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		plane := make([]survey2d.SegmentWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, nil, err
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, nil, err
			}
			if err := boundarywalk.RequireAnalyticWalk(w, "the survey boundary walk"); err != nil {
				return nil, nil, err
			}
			plane[i] = w
			raw[i] = survey2d.SideWalk{SegmentWalk: axis.Walk(w), Segs: []int{i}}
		}
		walks, err := boundarywalk.CoalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, nil, err
		}
		out = append(out, walks)
		planes = append(planes, plane)
	}
	return out, planes, nil
}

// PrismWall reads the spanning balls of a prism's recorded section and cap
// interval. A displaced section is undecided because the reading has no bound
// to widen. Only an undecomposable free-form section is an undecided walk;
// all other survey errors reach the caller.
func PrismWall(budget *proofbound.WorkBudget, profile momentinput.Profile,
	height survey2d.PrismHeight, sectionDelta, alpha float64) (verifypublish.ScalarSurvey, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return verifypublish.ScalarSurvey{}, err
	}
	if sectionDelta != 0 {
		return verifypublish.ScalarSurvey{}, nil
	}
	loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(profile))
	if err != nil {
		if errors.Is(err, boundarywalk.ErrFreeformSection) {
			// runSurveys polls the budget again after mapping this undecided
			// result, so cancellation still reaches Verify as an error.
			return verifypublish.ScalarSurvey{}, nil
		}
		return verifypublish.ScalarSurvey{}, err
	}
	reading, err := survey2d.PrismWallReading(budget, loops, height, alpha)
	return verifypublish.ScalarSurvey{Reading: reading.Reading, Bound: reading.Bound, OK: reading.Ok}, err
}

// RevolveWall reads the meridian's bounded spanning-ball survey. A displaced
// meridian is undecided for the same reason as a displaced prism section.
func RevolveWall(budget *proofbound.WorkBudget, record RevolveRecord,
	alpha float64) (verifypublish.ScalarSurvey, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return verifypublish.ScalarSurvey{}, err
	}
	if record.SectionDelta != 0 {
		return verifypublish.ScalarSurvey{}, nil
	}
	loops, err := RevolveLoops(budget, record.Profile, record.Axis)
	if err != nil {
		return verifypublish.ScalarSurvey{}, err
	}
	var carriers [][]int
	if record.Full {
		carriers, err = revolveWallCarriers(budget, record.Profile, record.Axis, loops)
		if err != nil {
			return verifypublish.ScalarSurvey{}, err
		}
	}
	reading, err := revolvesurvey.WallReading(budget, loops, carriers, record.Axis,
		record.Full, record.Phi0, record.Phi1, record.AngularDelta, alpha)
	return verifypublish.ScalarSurvey{Reading: reading.Reading, Bound: reading.Bound, OK: reading.Ok}, err
}

// wholeArcCarrierKey identifies a whole source arc by its exact stored circle.
func wholeArcCarrierKey(segment sectionrecord.CurveSegment) (string, bool) {
	normalized, err := sectionrecord.NormalizeSegment(segment)
	if err != nil {
		return "", false
	}
	arc, ok := normalized.(sectionrecord.ArcSeg)
	if !ok || !((arc.TStart == 0 && arc.TEnd == 1) || (arc.TStart == 1 && arc.TEnd == 0)) {
		return "", false
	}
	u, ok := proof.DyOf(arc.Center.U)
	if !ok {
		return "", false
	}
	v, ok := proof.DyOf(arc.Center.V)
	if !ok {
		return "", false
	}
	start, ok := proof.DySquaredDistance3(arc.Start.U, arc.Start.V, 0, arc.Center.U, arc.Center.V, 0)
	if !ok {
		return "", false
	}
	end, ok := proof.DySquaredDistance3(arc.End.U, arc.End.V, 0, arc.Center.U, arc.Center.V, 0)
	if !ok || start.Sign() <= 0 || proof.DyCmp(start, end) != 0 {
		return "", false
	}
	key := u.AppendKey(nil)
	key = v.AppendKey(key)
	key = start.AppendKey(key)
	return string(key), true
}

func revolveWallCarriers(budget *proofbound.WorkBudget, profile momentinput.Profile, axis revolveaxis.Frame, loops [][]survey2d.SideWalk) ([][]int, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return nil, err
	}
	if axis.DUBound != 0 || axis.DVBound != 0 || !((axis.DU == 0 && (axis.DV == 1 || axis.DV == -1)) || (axis.DV == 0 && (axis.DU == 1 || axis.DU == -1))) {
		return nil, nil
	}
	sources := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	if len(sources) != len(loops) {
		return nil, fmt.Errorf("revolve wall carrier table: loop count mismatch")
	}
	result := make([][]int, len(loops))
	ids := make(map[string]int)
	for i, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		result[i] = make([]int, len(loop))
		for j, walk := range loop {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			if !revolvesurvey.OnAxisCentre(walk) {
				continue
			}
			if len(walk.Segs) == 0 {
				return nil, fmt.Errorf("revolve wall carrier table: empty source indices")
			}
			key := ""
			eligible := true
			for _, index := range walk.Segs {
				if err := survey2d.WallBudgetStep(budget); err != nil {
					return nil, err
				}
				if index < 0 || index >= len(sources[i].Segments) {
					return nil, fmt.Errorf("revolve wall carrier table: invalid source index")
				}
				k, ok := wholeArcCarrierKey(sources[i].Segments[index])
				if !ok || (key != "" && key != k) {
					eligible = false
				}
				if key == "" {
					key = k
				}
			}
			if !eligible {
				continue
			}
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			id, ok := ids[key]
			if !ok {
				id = len(ids) + 1
				ids[key] = id
			}
			result[i][j] = id
		}
	}
	return result, nil
}
