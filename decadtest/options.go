package decadtest

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// Option configures a decadtest comparison helper: how much slack the
// author's own oracle carries, and whether the reading must be decad.Exact.
// The house pattern decad's own verify.go uses: a sealed marker interface
// embedding option.Interface, a private wrapper, and one empty ident struct
// per option.
type Option interface {
	option.Interface
	decadtestOption()
}

type decadtestOption struct{ option.Interface }

func (decadtestOption) decadtestOption() {}

type identWithin struct{}
type identWithinRel struct{}
type identExactly struct{}

// defaultRelativeSlack is the rounding of a float64 oracle across a handful
// of operations, roughly fifty ulps.
const defaultRelativeSlack = 1e-12

// Within sets the absolute oracle slack s: the error of the AUTHOR's own
// oracle, never of decad's reading. Its Kind must be the reading's Kind — a
// millimetre slack handed to a volume reading never silently compares mm
// against mm^3, which is why the parameter is a units.Value and not a
// float64 (docs/api-design.md §5.1). A wrong-Kind Within is a test bug the
// helper reports before it compares any number.
//
// A later Within or WithinRel replaces an earlier one in the same call: they
// are alternatives stating the same thing in different units, not layers
// that combine.
func Within(v units.Value) Option {
	return decadtestOption{option.New(identWithin{}, v)}
}

// WithinRel sets the oracle slack as a fraction of |want|: s = rel * |W|.
// rel must be Dimensionless (units.Scalar). This is the usual choice when
// the oracle is a closed-form float64 formula such as math.Pi*r*r*h, where
// the slack should scale with the expected magnitude rather than stay fixed.
//
// A later Within or WithinRel replaces an earlier one in the same call: see
// Within.
func WithinRel(rel units.Value) Option {
	return decadtestOption{option.New(identWithinRel{}, rel)}
}

// Exactly additionally requires the reading to be decad.Exact with a zero
// bound. decad.Exact means the number is the truth and the bound beside it
// is zero; the author's slack still applies on top, because a float64
// oracle can still be off by rounding, so the rule simply reduces to
// |V - W| <= s.
//
// No helper ever takes an expected BOUND. A bound differs between amd64 and
// arm64 through FMA, so a test that pinned one would be architecture
// specific. The only equality ever asserted on a bound is zero, through
// Exactly.
func Exactly() Option {
	return decadtestOption{option.New(identExactly{}, struct{}{})}
}

// cmpConfig is one comparison's resolved options.
//
// A later Within or WithinRel replaces an earlier one: last one wins,
// because they are alternatives stating the author's oracle slack in
// different units, never layers that combine.
type cmpConfig struct {
	absolute  units.Value
	hasAbs    bool
	relative  units.Value
	hasRel    bool
	exactly   bool
	nilOption bool
}

// resolveOptions folds opts into a cmpConfig. A later Within or WithinRel
// replaces an earlier one; a nil Option is recorded for the caller to
// reject, since an option constructor has no error to return.
func resolveOptions(opts []Option) cmpConfig {
	var cfg cmpConfig
	for _, o := range opts {
		if o == nil {
			cfg.nilOption = true
			continue
		}
		switch o.Ident().(type) {
		case identWithin:
			if v, ok := option.Get[units.Value](o); ok {
				cfg.absolute = v
				cfg.hasAbs = true
				cfg.hasRel = false
			}
		case identWithinRel:
			if v, ok := option.Get[units.Value](o); ok {
				cfg.relative = v
				cfg.hasRel = true
				cfg.hasAbs = false
			}
		case identExactly:
			cfg.exactly = true
		}
	}
	return cfg
}

// slack computes the author's slack s in base units, for an expectation of
// magnitude mag (base units) and Kind kind. It fails tb on a wrong-Kind
// option and returns the slack and whether it came from the default.
func slack(tb testing.TB, what string, cfg cmpConfig, kind units.Kind, mag float64) (float64, bool) {
	tb.Helper()

	if cfg.nilOption {
		tb.Fatalf("%s: a nil Option was passed", what)
		return 0, false
	}

	if cfg.hasAbs {
		if cfg.absolute.Kind() != kind {
			tb.Fatalf("%s", kindMismatch(what, "Within slack", cfg.absolute.Kind(), kind))
			return 0, false
		}
		return math.Abs(cfg.absolute.Base()), false
	}

	if cfg.hasRel {
		if cfg.relative.Kind() != units.Dimensionless {
			tb.Fatalf("%s", kindMismatch(what, "WithinRel", cfg.relative.Kind(), units.Dimensionless))
			return 0, false
		}
		return math.Abs(cfg.relative.Base()) * math.Abs(mag), false
	}

	return defaultRelativeSlack * math.Abs(mag), true
}
