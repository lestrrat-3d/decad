package radiussurvey

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvesurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
)

// Resolve returns the bounded minimum over every admitted concave radius.
// An empty aggregate proves the body has no concave principal radius.
func Resolve(agg survey2d.ExtremeAggregate) reportvocab.ScalarSurvey {
	if agg.Empty() && !agg.Unbounded {
		return reportvocab.ScalarSurvey{OK: true}
	}
	mid, bound, ok := agg.Resolve()
	if !ok {
		return reportvocab.ScalarSurvey{}
	}
	return reportvocab.ScalarSurvey{Reading: &mid, Bound: bound, OK: true}
}

// Prism reads concave circular walls from a recorded prism section. A
// displaced section cannot prove its denoted section's radius.
func Prism(profile momentinput.Profile, sectionDelta float64) reportvocab.ScalarSurvey {
	if sectionDelta != 0 {
		return reportvocab.ScalarSurvey{}
	}
	loops, err := boundarywalk.SurveyLoops(nil, boundarywalk.Profile(profile))
	if err != nil {
		return reportvocab.ScalarSurvey{}
	}
	agg := survey2d.MinAggregate()
	for _, loop := range loops {
		for _, w := range loop {
			if w.IsCircular() && w.Th1 < w.Th0 {
				agg.Take(w.Radius, w.RadiusBound)
			}
		}
	}
	return Resolve(agg)
}

// Revolve reads the meridian's bounded curvature after axis resolution.
func Revolve(profile momentinput.Profile, axis revolveaxis.Frame, sectionDelta float64) reportvocab.ScalarSurvey {
	if sectionDelta != 0 {
		return reportvocab.ScalarSurvey{}
	}
	loops, err := wallsurvey.RevolveLoops(nil, profile, axis)
	if err != nil {
		return reportvocab.ScalarSurvey{}
	}
	return Resolve(revolvesurvey.RadiusAggregate(loops, axis))
}

// Cup reads the outer region and the reversed cavity region. The cavity's
// offset displacement widens every finite recorded radius reading.
func Cup(outer, cavity momentinput.Profile, offsetDelta float64) reportvocab.ScalarSurvey {
	profile := momentinput.Profile{Outer: outer.Outer}
	profile.Holes = append(profile.Holes, outer.Holes...)
	loops := append([]sectionrecord.LoopRecord{cavity.Outer}, cavity.Holes...)
	for _, loop := range loops {
		reversed, err := offset2d.ReverseLoopRecord(loop)
		if err != nil {
			return reportvocab.ScalarSurvey{}
		}
		profile.Holes = append(profile.Holes, reversed)
	}
	out := Prism(profile, 0)
	if out.OK && out.Reading != nil && offsetDelta > 0 {
		out.Bound = proofbound.AbsSumUpper(out.Bound, offsetDelta)
	}
	return out
}
