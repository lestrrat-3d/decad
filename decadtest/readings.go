package decadtest

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the one comparison rule, stated once and applied to decad's
// three bounded shapes: a decad.Measurement{Value: V, Bound: B} is decad's
// claim that the true quantity lies in [V-B, V+B]. The author's want is a
// second claim W ± s, where s is the error of the AUTHOR's own oracle. The
// two are consistent exactly when |V - W| <= B + s, all four terms in base
// units of the same units.Kind. B is read from the reading, never supplied.
//
// units.Value.Equal makes every scalar pass/fail decision; nothing here
// subtracts two Base() floats to decide anything. Subtraction appears only
// in the display text a failure prints.

const defaultSlackAdvice = "\n  (slack is the default 1e-12 relative; state your oracle's error with Within or WithinRel)"

// Measures fails tb unless the reading's proven interval and the author's
// own claim about want overlap: |V - W| <= B + s. what is the label the
// message opens with. B is read from got and never supplied; the default
// slack is 1e-12 * |want|. Kind is checked before magnitude: a wrong-Kind
// want or Within is a test bug the helper reports before it compares any
// number.
func Measures(tb testing.TB, what string, got decad.Measurement, want units.Value, opts ...Option) {
	tb.Helper()

	cfg := resolveOptions(opts)

	if got.Value.Kind() != want.Kind() {
		tb.Fatalf("%s", kindMismatch(what, "expected value", want.Kind(), got.Value.Kind()))
		return
	}

	if got.Exactness == decad.Exact && got.Bound.Base() != 0 {
		tb.Fatalf("%s: reading %s claims Exact but carries a nonzero bound", what, measurementText(got))
		return
	}

	if cfg.exactly && got.Exactness != decad.Exact {
		tb.Fatalf("%s: Exactly() was required but the reading is Approximate: %s", what, measurementText(got))
		return
	}

	s, isDefault := slack(tb, what, cfg, want.Kind(), want.Base())
	tol := got.Bound.Base() + s
	if got.Value.Equal(want, tol) {
		return
	}

	u, _ := units.BaseUnit(want.Kind())
	offBy := units.FromBase(math.Abs(got.Value.Base()-want.Base()), u)
	slackVal := units.FromBase(s, u)
	tolVal := units.FromBase(tol, u)

	msg := fmt.Sprintf("%s: reading %s does not enclose expected %s\n  off by %s; allowed bound %s + slack %s = %s",
		what, measurementText(got), want, offBy, got.Bound, slackVal, tolVal)
	if isDefault {
		msg += defaultSlackAdvice
	}
	tb.Fatalf("%s", msg)
}

// MeasuresVec fails tb unless want lies in the ball of radius B + s around
// the reading: |V - W|2 <= B + s. got.Bound.Kind() is Length for a position
// and Dimensionless for a direction; the helper READS it and never assumes.
// want carries no Kind of its own — it is an r3.Vec, the
// docs/api-design.md §5.2 coordinate carve-out — so a wrong-Kind Within or
// WithinRel is instead caught against got.Bound.Kind().
func MeasuresVec(tb testing.TB, what string, got decad.VecMeasurement, want r3.Vec, opts ...Option) {
	tb.Helper()

	cfg := resolveOptions(opts)

	if got.Exactness == decad.Exact && got.Bound.Base() != 0 {
		tb.Fatalf("%s: reading %s claims Exact but carries a nonzero bound", what, vecMeasurementText(got))
		return
	}

	if cfg.exactly && got.Exactness != decad.Exact {
		tb.Fatalf("%s: Exactly() was required but the reading is Approximate: %s", what, vecMeasurementText(got))
		return
	}

	s, isDefault := slack(tb, what, cfg, got.Bound.Kind(), want.Len())
	tol := got.Bound.Base() + s
	d := got.Value.Sub(want).Len()
	if d <= tol {
		return
	}

	u, _ := units.BaseUnit(got.Bound.Kind())
	offBy := units.FromBase(d, u)
	slackVal := units.FromBase(s, u)
	tolVal := units.FromBase(tol, u)

	msg := fmt.Sprintf("%s: reading %s does not enclose expected %v\n  off by %s; allowed bound %s + slack %s = %s",
		what, vecMeasurementText(got), want, offBy, got.Bound, slackVal, tolVal)
	if isDefault {
		msg += defaultSlackAdvice
	}
	tb.Fatalf("%s", msg)
}

// MeasuresBox fails tb unless every one of the box's six corner coordinates
// is within B + s of the expected one. The default relative slack scales
// off the EXPECTED box's own diagonal length (hi.Sub(lo).Len()), never the
// reading's own diagonal: s is about the author's oracle, never about
// decad's box (see Within).
func MeasuresBox(tb testing.TB, what string, got decad.Box, lo, hi r3.Vec, opts ...Option) {
	tb.Helper()

	cfg := resolveOptions(opts)

	if got.Exactness == decad.Exact && got.Bound.Base() != 0 {
		tb.Fatalf("%s: reading %s claims Exact but carries a nonzero bound", what, boxText(got))
		return
	}

	if cfg.exactly && got.Exactness != decad.Exact {
		tb.Fatalf("%s: Exactly() was required but the reading is Approximate: %s", what, boxText(got))
		return
	}

	s, _ := slack(tb, what, cfg, got.Bound.Kind(), hi.Sub(lo).Len())
	tol := got.Bound.Base() + s

	u, _ := units.BaseUnit(got.Bound.Kind())
	slackVal := units.FromBase(s, u)

	coords := []struct {
		name      string
		got, want float64
	}{
		{"min.X", got.Min.X, lo.X},
		{"min.Y", got.Min.Y, lo.Y},
		{"min.Z", got.Min.Z, lo.Z},
		{"max.X", got.Max.X, hi.X},
		{"max.Y", got.Max.Y, hi.Y},
		{"max.Z", got.Max.Z, hi.Z},
	}

	var msg strings.Builder
	bad := false
	for _, c := range coords {
		diff := math.Abs(c.got - c.want)
		if diff <= tol {
			continue
		}
		bad = true
		offBy := units.FromBase(diff, u)
		fmt.Fprintf(&msg, "\n  %s off by %s; allowed bound %s + slack %s", c.name, offBy, got.Bound, slackVal)
	}
	if !bad {
		return
	}

	tb.Fatalf("%s: reading %s; want min %v max %v%s", what, boxText(got), lo, hi, msg.String())
}

// Encloses fails tb unless p lies inside the box grown by its own bound. It
// takes no slack option: the design gives it none, since the box already
// carries decad's own proven bound and there is no author oracle in play.
func Encloses(tb testing.TB, what string, got decad.Box, p r3.Vec) {
	tb.Helper()

	b := got.Bound.Base()
	u, _ := units.BaseUnit(got.Bound.Kind())

	axes := []struct {
		name        string
		p, min, max float64
	}{
		{"X", p.X, got.Min.X, got.Max.X},
		{"Y", p.Y, got.Min.Y, got.Max.Y},
		{"Z", p.Z, got.Min.Z, got.Max.Z},
	}

	var msg strings.Builder
	bad := false
	for _, a := range axes {
		switch {
		case a.p < a.min-b:
			bad = true
			d := units.FromBase(a.min-b-a.p, u)
			fmt.Fprintf(&msg, "\n  p.%s is %s below min.%s", a.name, d, a.name)
		case a.p > a.max+b:
			bad = true
			d := units.FromBase(a.p-a.max-b, u)
			fmt.Fprintf(&msg, "\n  p.%s is %s above max.%s", a.name, d, a.name)
		}
	}
	if !bad {
		return
	}

	tb.Fatalf("%s: box %s does not enclose point %v%s", what, boxText(got), p, msg.String())
}

// Agree fails tb unless two readings' proven intervals meet:
// |Va - Vb| <= Ba + Bb + s. The default relative slack scales off
// math.Max(|Va|, |Vb|), the larger of the two readings, since neither is
// the author's own number.
func Agree(tb testing.TB, what string, a, b decad.Measurement, opts ...Option) {
	tb.Helper()

	cfg := resolveOptions(opts)

	if a.Value.Kind() != b.Value.Kind() {
		tb.Fatalf("%s", kindMismatch(what, "the first reading", a.Value.Kind(), b.Value.Kind()))
		return
	}

	if a.Exactness == decad.Exact && a.Bound.Base() != 0 {
		tb.Fatalf("%s: reading %s claims Exact but carries a nonzero bound", what, measurementText(a))
		return
	}
	if b.Exactness == decad.Exact && b.Bound.Base() != 0 {
		tb.Fatalf("%s: reading %s claims Exact but carries a nonzero bound", what, measurementText(b))
		return
	}

	if cfg.exactly {
		if a.Exactness != decad.Exact {
			tb.Fatalf("%s: Exactly() was required but the reading is Approximate: %s", what, measurementText(a))
			return
		}
		if b.Exactness != decad.Exact {
			tb.Fatalf("%s: Exactly() was required but the reading is Approximate: %s", what, measurementText(b))
			return
		}
	}

	mag := math.Max(math.Abs(a.Value.Base()), math.Abs(b.Value.Base()))
	s, _ := slack(tb, what, cfg, a.Value.Kind(), mag)
	tol := a.Bound.Base() + b.Bound.Base() + s
	if a.Value.Equal(b.Value, tol) {
		return
	}

	u, _ := units.BaseUnit(a.Value.Kind())
	offBy := units.FromBase(math.Abs(a.Value.Base()-b.Value.Base()), u)
	slackVal := units.FromBase(s, u)
	tolVal := units.FromBase(tol, u)

	tb.Fatalf("%s: readings do not agree: %s and %s\n  off by %s; allowed bounds %s + %s + slack %s = %s",
		what, measurementText(a), measurementText(b), offBy, a.Bound, b.Bound, slackVal, tolVal)
}

// HasBoundAtMost fails tb unless bound is at or under the stated ceiling.
//
// Leave orders of magnitude between the ceiling and any bound actually
// observed: a bound differs between amd64 and arm64 through FMA, so a
// ceiling set just above an observed value is an architecture-specific
// test. HasBoundAtMost offers no relative form: the verifier's gate anchors
// a relative bound on a per-body reference with a noise floor this kit
// cannot derive, and a test that wants that judgement should ask
// decad.Document.Verify with decad.WithTolerance instead.
func HasBoundAtMost(tb testing.TB, what string, bound, limit units.Value) {
	tb.Helper()

	if bound.Kind() != limit.Kind() {
		tb.Fatalf("%s", kindMismatch(what, "ceiling", limit.Kind(), bound.Kind()))
		return
	}

	if bound.Base() <= limit.Base() {
		return
	}

	tb.Fatalf("%s: bound %s exceeds the stated ceiling %s", what, bound, limit)
}
