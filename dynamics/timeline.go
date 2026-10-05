package dynamics

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/units"
)

// ErrTimelineStopped reports an Advance after the timeline stopped at an
// Undecided step.
var ErrTimelineStopped = errors.New("dynamics: timeline stopped at an undecided step")

// Timeline chains the steps of one world from one start state
// (docs/multibody-dynamics-design.md §7.2). Its certified end is the
// completed time of the last advanced step; it stops at the first Undecided
// step and never advances past it.
//
// Advance is the only writer: it must not run concurrently with Sample or
// with another Advance. Sample writes nothing, so any number of Sample calls
// may run concurrently between Advances, and each returns the same State for
// the same time.
type Timeline struct {
	world *World
	start State
	steps []timelineStep
	stop  *StepReport
}

// timelineStep is one advanced step and the exact held time it starts at.
type timelineStep struct {
	report *StepReport
	begin  *big.Rat
	end    *big.Rat
}

// NewTimeline starts a timeline of w at start. w must not be nil, and start
// must be a state of w.
func NewTimeline(w *World, start State) (*Timeline, error) {
	if w == nil || start.world != w {
		return nil, fmt.Errorf("%w: timeline needs a world and one of its states", ErrInvalidInput)
	}
	return &Timeline{world: w, start: start}, nil
}

// Advance runs World.Step for dt from the state the last advanced step
// published, or from the start state. An Advanced report is appended; an
// Undecided report stops the timeline and is returned with a nil error, since
// an undecided step is a result, not a failure. A step error leaves the
// timeline unchanged. After the timeline stops, Advance returns
// ErrTimelineStopped.
func (tl *Timeline) Advance(ctx context.Context, input StepInput, dt units.Value) (*StepReport, error) {
	if tl.stop != nil {
		return nil, ErrTimelineStopped
	}
	from := tl.start
	begin := new(big.Rat)
	if n := len(tl.steps); n != 0 {
		from, begin = *tl.steps[n-1].report.Next, tl.steps[n-1].end
	}
	report, err := tl.world.Step(ctx, from, input, dt)
	if err != nil {
		return nil, err
	}
	if report.Status != Advanced {
		tl.stop = report
		return report, nil
	}
	end := new(big.Rat).Add(begin, exactBase(dt))
	tl.steps = append(tl.steps, timelineStep{report: report, begin: begin, end: end})
	return report, nil
}

// End is the exact sum of the advanced durations, at the nearest float when
// that sum is not one.
func (tl *Timeline) End() units.Value {
	if len(tl.steps) == 0 {
		return units.Seconds(0)
	}
	seconds, _ := tl.steps[len(tl.steps)-1].end.Float64()
	return units.Seconds(seconds)
}

// Steps returns a copy of every advanced step's report, in order.
func (tl *Timeline) Steps() []*StepReport {
	out := make([]*StepReport, len(tl.steps))
	for i, step := range tl.steps {
		report := *step.report
		out[i] = &report
	}
	return out
}

// Stopped returns the Undecided report that stopped the timeline, or nil.
func (tl *Timeline) Stopped() *StepReport { return tl.stop }

// Sample returns the certified state at t from the timeline's start. It
// locates the step whose held interval holds t and delegates to that step's
// Trace.Sample with the local time; a step boundary belongs to the later
// step's start, which is the earlier step's end. t equal to End returns the
// last advanced step's end state. A t below zero or beyond End is
// ErrUnsupported.
//
// The local time is t less the step's exact start. When that difference is
// not a float, the nearest float inside the step labels it; the pose
// returned is the certified pose at that label.
func (tl *Timeline) Sample(t units.Value) (State, error) {
	at := exactBase(t)
	if t.Kind() != units.Time || at == nil || !finite(t.Base()) {
		return State{}, fmt.Errorf("%w: timeline sample time is not a finite time", ErrInvalidInput)
	}
	if at.Sign() < 0 || at.Cmp(exactBase(tl.End())) > 0 {
		return State{}, fmt.Errorf("%w: timeline sample time is outside [0, End()]", ErrUnsupported)
	}
	if len(tl.steps) == 0 {
		return tl.start, nil
	}
	last := tl.steps[len(tl.steps)-1]
	if at.Cmp(last.end) >= 0 {
		return *last.report.Next, nil
	}
	for _, step := range tl.steps {
		if at.Cmp(step.end) >= 0 {
			continue
		}
		local := new(big.Rat).Sub(at, step.begin)
		seconds, _ := local.Float64()
		duration := new(big.Rat).Sub(step.end, step.begin)
		if ratFloat(seconds).Cmp(duration) > 0 {
			seconds, _ = duration.Float64()
		}
		return step.report.Trace.Sample(units.Seconds(seconds))
	}
	return *last.report.Next, nil
}
