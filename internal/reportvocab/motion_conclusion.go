package reportvocab

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// MotionPoseFinding contains the facts report publication reads from one pose.
type MotionPoseFinding[BodyT comparable, CellT any] struct {
	At          units.Value
	Unbuildable bool
	Violated    bool
	Diagnostics []Diagnostic[BodyT, CellT]
}

// MotionSpanFinding contains one adjacent pose interval's certificate.
type MotionSpanFinding struct {
	Outcome   IntervalOutcome
	Clearance *measurement.Measurement
	Note      string
}

// MotionConclusion is the verdict shared by motion and linkage reports.
type MotionConclusion[BodyT comparable, CellT any] struct {
	Request     MotionRequest
	Against     []BodyT
	Intervals   []MotionInterval
	Clearance   *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic[BodyT, CellT]
	Status      Status
}

// ConcludeMotion assembles interval findings before pose findings, merges
// intervals around unbuildable interior poses, and reads a path clearance only
// when every interval is certified clear. Poses must hold one more entry than spans.
func ConcludeMotion[BodyT comparable, CellT any](request MotionRequest, against []BodyT,
	poses []MotionPoseFinding[BodyT, CellT], spans []MotionSpanFinding, minimum *big.Rat,
	pathClearance func(*measurement.Measurement) (*ScalarReading, *Diagnostic[BodyT, CellT]),
) MotionConclusion[BodyT, CellT] {
	c := MotionConclusion[BodyT, CellT]{
		Request:     request,
		Against:     against,
		Diagnostics: []Diagnostic[BodyT, CellT]{},
	}
	violated := false
	for _, pose := range poses {
		violated = violated || pose.Violated
	}
	allClear, met := true, true
	var lowest *measurement.Measurement
	for k := 0; k < len(spans); k++ {
		span := spans[k]
		a := poses[k]
		// A refused interior pose joins the intervals on either side into one
		// undecided interval. The path's own endpoints remain in the report.
		for poses[k+1].Unbuildable && k+2 < len(poses) {
			k++
			span = MotionSpanFinding{Outcome: IntervalUndecided, Note: firstMotionNote(span.Note, spans[k].Note)}
		}
		b := poses[k+1]
		interval := MotionInterval{From: a.At, To: b.At, Outcome: span.Outcome, Clearance: span.Clearance}
		c.Intervals = append(c.Intervals, interval)
		if span.Outcome != IntervalClear {
			allClear, met = false, false
		}
		if span.Clearance != nil && (lowest == nil || span.Clearance.Value.Base() < lowest.Value.Base()) {
			lowest = span.Clearance
		}
		switch {
		case span.Outcome == IntervalUndecided:
			msg := fmt.Sprintf("the motion from %s to %s is neither certified clear nor bounded by a proven collision", a.At, b.At)
			if span.Note != "" {
				msg += ": " + span.Note
			}
			at := a.At
			c.Diagnostics = append(c.Diagnostics, Diagnostic[BodyT, CellT]{
				Code:    DiagMotionUndecidedInterval,
				Status:  Suspect,
				Reading: ReadingNone,
				Message: msg,
				At:      &at,
			})
		case span.Outcome == IntervalClear && minimum != nil && span.Clearance != nil &&
			proofarith.FloatRat(span.Clearance.Value.Base()).Cmp(minimum) < 0:
			met = false
			if violated {
				break
			}
			obs, at := *span.Clearance, a.At
			c.Diagnostics = append(c.Diagnostics, Diagnostic[BodyT, CellT]{
				Code:     DiagMotionUndecidedClearance,
				Status:   Suspect,
				Reading:  ReadingGap,
				Observed: &obs,
				Required: request.MinClearance,
				Message:  fmt.Sprintf("the motion from %s to %s is certified clear, but its proven lower bound does not reach the required minimum", a.At, b.At),
				At:       &at,
			})
		}
	}
	switch {
	case minimum == nil:
		c.Assessment = AssessmentNotEvaluated
	case violated:
		c.Assessment = AssessmentViolated
	case met:
		c.Assessment = AssessmentMet
	default:
		c.Assessment = AssessmentUndecided
	}
	for _, pose := range poses {
		c.Diagnostics = append(c.Diagnostics, pose.Diagnostics...)
	}
	if allClear && lowest != nil {
		if reading, diag := pathClearance(lowest); reading != nil {
			c.Clearance = reading
			if diag != nil {
				c.Diagnostics = append(c.Diagnostics, *diag)
			}
		}
	}
	c.Status = WorstStatus(c.Diagnostics)
	return c
}

func firstMotionNote(notes ...string) string {
	for _, note := range notes {
		if note != "" {
			return note
		}
	}
	return ""
}

// WorstStatus reports the highest status of the findings, or Sound when empty.
func WorstStatus[BodyT comparable, CellT any](diagnostics []Diagnostic[BodyT, CellT]) Status {
	status := Sound
	for _, diagnostic := range diagnostics {
		status = max(status, diagnostic.Status)
	}
	return status
}
