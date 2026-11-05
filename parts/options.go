package parts

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// config is what the options fold into; each query makes its own.
type config struct {
	fitTol     float64 // mm
	axisTol    float64 // radians
	maxFit     float64 // mm
	contactTol float64 // mm
	gap        float64 // mm
	ground     string
	poses      map[string]r3.Transform
	verify     *decad.Report
	witness    bool
	stages     []Stage
	partGaps   map[string]units.Value
	scale      float64
}

func defaultConfig() config {
	return config{fitTol: 0.05, axisTol: 0.5 * math.Pi / 180, maxFit: 1, contactTol: 0.25, gap: 10, scale: 1}
}

// option is the one concrete option type. The three public option interfaces
// name which queries accept it.
type option struct{ f func(*config) error }

func (o option) apply(c *config) error { return o.f(c) }
func (option) explodeOption()          {}
func (option) viewOption()             {}
func (option) checkOption()            {}

// ExplodeOption configures Explode. The tolerance options of CheckOption are
// accepted too.
type ExplodeOption interface {
	apply(*config) error
	explodeOption()
}

// ViewOption configures Views. The tolerance options of CheckOption are
// accepted too.
type ViewOption interface {
	apply(*config) error
	viewOption()
}

// CheckOption configures Check and Infer, and sets the tolerances Explode and
// Views read.
type CheckOption interface {
	ExplodeOption
	ViewOption
	checkOption()
}

func foldOptions[O interface{ apply(*config) error }](opts []O) (config, error) {
	c := defaultConfig()
	for _, o := range opts {
		// A nil interface value has a nil dynamic type.
		if any(o) == nil {
			return config{}, fmt.Errorf("parts: %w: nil option", decad.ErrDegenerate)
		}
		if err := o.apply(&c); err != nil {
			return config{}, err
		}
	}
	return c, nil
}

func lengthOption(v units.Value, what string, allowZero bool, set func(*config, float64)) (option, error) {
	if err := checkLength(v, what, false); err != nil {
		return option{}, err
	}
	if v == (units.Value{}) || (!allowZero && v.Base() == 0) {
		return option{}, fmt.Errorf("parts: %w: %s must be positive", decad.ErrDegenerate, what)
	}
	return option{func(c *config) error { set(c, v.Base()); return nil }}, nil
}

// failing returns an option that reports err when applied, so a bad argument is
// refused by the query it was given to, before ctx is read.
func failing(err error) option { return option{func(*config) error { return err }} }

func lengthCheckOption(v units.Value, what string, allowZero bool, set func(*config, float64)) CheckOption {
	o, err := lengthOption(v, what, allowZero, set)
	if err != nil {
		return failing(err)
	}
	return o
}

// WithFitTolerance sets the largest axis distance and radius disagreement a
// match allows; the default is 0.25 mm.
func WithFitTolerance(tol units.Value) CheckOption {
	return lengthCheckOption(tol, "fit tolerance", false, func(c *config, v float64) { c.fitTol = v })
}

// WithContactTolerance sets the gap at which two parts count as touching; the
// default is 0.05 mm.
func WithContactTolerance(tol units.Value) CheckOption {
	return lengthCheckOption(tol, "contact tolerance", false, func(c *config, v float64) { c.contactTol = v })
}

// WithAxisTolerance sets the largest angle between axes that count as
// parallel; the default is 0.5 degrees. a is an Angle in (0, 90 degrees).
func WithAxisTolerance(a units.Value) CheckOption {
	if a.Kind() != units.Angle {
		return failing(fmt.Errorf("parts: axis tolerance: %w", decad.ErrUnitKind))
	}
	r := a.Base()
	switch {
	case math.IsNaN(r) || math.IsInf(r, 0):
		return failing(fmt.Errorf("parts: axis tolerance: %w", decad.ErrNotFinite))
	case r <= 0 || r >= math.Pi/2:
		return failing(fmt.Errorf("parts: %w: axis tolerance outside (0, 90 degrees)", decad.ErrDegenerate))
	}
	return option{func(c *config) error { c.axisTol = r; return nil }}
}

// WithMaxFit sets the largest clearance a match accepts, in either sign; the
// default is 1 mm.
func WithMaxFit(fit units.Value) CheckOption {
	return lengthCheckOption(fit, "max fit", false, func(c *config, v float64) { c.maxFit = v })
}

// WithVerifyReport makes Check read the pair partition from r instead of
// running Document.Verify. Explode and Views ignore it.
func WithVerifyReport(r *decad.Report) CheckOption {
	if r == nil {
		return failing(fmt.Errorf("parts: %w: nil report", decad.ErrDegenerate))
	}
	return option{func(c *config) error { c.verify = r; return nil }}
}

// WithExplodeGround names the part that stays put; the default is the
// manufactured part of largest volume.
func WithExplodeGround(name string) ExplodeOption {
	return option{func(c *config) error { c.ground = name; return nil }}
}

// WithExplodeGap sets how far past full disengagement each part moves; the
// default is 10 mm. A zero gap is allowed.
func WithExplodeGap(gap units.Value) ExplodeOption {
	if err := checkLength(gap, "explode gap", false); err != nil {
		return failing(err)
	}
	return option{func(c *config) error { c.gap = gap.Base(); return nil }}
}

// WithPartPose makes Views pose the named part by t instead of centring it. t
// maps assembled coordinates to view coordinates and must be a proper rigid
// motion: a reflection or the zero Transform is decad.ErrDegenerate. A name no
// part has is decad.ErrDegenerate at the query.
func WithPartPose(part string, t r3.Transform) ViewOption {
	if !t.IsValid() || t.IsReflection() {
		return failing(fmt.Errorf("parts: %w: pose of %q is not a proper rigid motion", decad.ErrDegenerate, part))
	}
	return option{func(c *config) error {
		if c.poses == nil {
			c.poses = map[string]r3.Transform{}
		}
		c.poses[part] = t
		return nil
	}}
}

// WithWitnessSearch makes Check also look for a witness for each unexplained
// pair: a point whose distances to the two parts prove the pair within the
// contact tolerance of each other, which Check then reports as PairFinding.Near.
// A pair without Near may be apart. The search samples up to 128 mesh vertices
// per pair and calls Body.DistanceToPoint at each.
func WithWitnessSearch() CheckOption {
	return option{func(c *config) error { c.witness = true; return nil }}
}
