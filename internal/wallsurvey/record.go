package wallsurvey

import (
	"errors"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
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
	height survey2d.PrismHeight, sectionDelta, alpha float64) (reportvocab.ScalarSurvey, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return reportvocab.ScalarSurvey{}, err
	}
	if sectionDelta != 0 {
		return reportvocab.ScalarSurvey{}, nil
	}
	loops, err := boundarywalk.SurveyLoops(budget, boundarywalk.Profile(profile))
	if err != nil {
		if errors.Is(err, boundarywalk.ErrFreeformSection) {
			// runSurveys polls the budget again after mapping this undecided
			// result, so cancellation still reaches Verify as an error.
			return reportvocab.ScalarSurvey{}, nil
		}
		return reportvocab.ScalarSurvey{}, err
	}
	reading, err := survey2d.PrismWallReading(budget, loops, height, alpha)
	return reportvocab.ScalarSurvey{Reading: reading.Reading, Bound: reading.Bound, OK: reading.Ok}, err
}

// RevolveWall reads the meridian's bounded spanning-ball survey. A displaced
// meridian is undecided for the same reason as a displaced prism section.
func RevolveWall(budget *proofbound.WorkBudget, record RevolveRecord,
	alpha float64) (reportvocab.ScalarSurvey, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return reportvocab.ScalarSurvey{}, err
	}
	if record.SectionDelta != 0 {
		return reportvocab.ScalarSurvey{}, nil
	}
	loops, err := RevolveLoops(budget, record.Profile, record.Axis)
	if err != nil {
		return reportvocab.ScalarSurvey{}, err
	}
	reading, err := revolvesurvey.WallReading(budget, loops, record.Axis,
		record.Full, record.Phi0, record.Phi1, record.AngularDelta, alpha)
	return reportvocab.ScalarSurvey{Reading: reading.Reading, Bound: reading.Bound, OK: reading.Ok}, err
}
