package verifyoption

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// VerifyOption configures Verify.
type VerifyOption interface {
	option.Interface
	verifyOption()
}

type verifyOption struct{ option.Interface }

func (verifyOption) verifyOption() {}

// WallOption parameterizes WithMinWallThickness.
type WallOption interface {
	option.Interface
	wallOption()
}

type wallOption struct{ option.Interface }

func (wallOption) wallOption() {}

type identTolerance struct{}
type identMinWall struct{}
type identPullDirection struct{}
type identConcaveRadius struct{}
type identClearances struct{}
type identDraftAllowance struct{}

// WallSpec is WithMinWallThickness's recorded question: the tool, and the
// draft allowance drawing the line between a wall and an edge. nilOption
// records a nil WallOption for Verify to reject — an option constructor has
// no error to return.
type WallSpec struct {
	Tool      units.Value
	Allowance units.Value
	NilOption bool
}

// WithTolerance sets the relative tolerance gate of verification §2: the
// largest error the caller accepts as a fraction of the quantity measured.
// Dimensionless; the default is units.Scalar(1e-3) — three significant
// figures. Exact answers carry a zero proven bound and pass at any
// tolerance.
func WithTolerance(rel units.Value) VerifyOption {
	return verifyOption{option.New(identTolerance{}, rel)}
}

// WithMinWallThickness states the spec that no wall may be thinner than
// minimum (verification §2), describing the comparison without assuming a
// tool type. The reading is the infimum diameter over the body's spanning
// inscribed balls — material between skins opposing within the draft
// allowance — and minimum enters only where the interval rule decides the
// reading against it (verification §6). A wall proven thinner is Violating.
func WithMinWallThickness(minimum units.Value, opts ...WallOption) VerifyOption {
	spec := WallSpec{Tool: minimum, Allowance: units.Degrees(15)}
	for _, o := range opts {
		if o == nil {
			// An option constructor has no error to return; Verify rejects
			// the recorded marker with decaderr.ErrDegenerate.
			spec.NilOption = true
			continue
		}
		switch o.Ident().(type) {
		case identDraftAllowance:
			if a, ok := option.Get[units.Value](o); ok {
				spec.Allowance = a
			}
		}
	}
	return verifyOption{option.New(identMinWall{}, spec)}
}

// WithDraftAllowance sets how much draft opposition tolerates — where the
// wall ends and the edge begins (verification §2). An angle in [0°, 90°);
// the default is units.Degrees(15).
func WithDraftAllowance(a units.Value) WallOption {
	return wallOption{option.New(identDraftAllowance{}, a)}
}

// WithPullDirection states the direction the part must pull along; every
// reported undercut is a proven violation of it (verification §2), decided
// per face from its normal range: a face with a provenly opposing point is
// listed, exactly perpendicular is not opposed (the vertical wall clears),
// and a non-empty listing is Violating. A bounded analytic stand-in widens
// its range by its own proven departure before this comparison. It also
// answers on a proven-valid surface-extruded prism sheet, reading each
// wall's positive side in place of a solid's outward normal
// (docs/surface-design.md §2.3, §9.1); every other sheet family reads
// CoverageUnavailable.
func WithPullDirection(v r3.Vec) VerifyOption {
	return verifyOption{option.New(identPullDirection{}, v)}
}

// WithConcaveRadius asks for the tightest concave radius — a measurement,
// not a verdict; Verify introduces no radius threshold or machining-access
// assessment of its own (verification §2). On the analytic faces convexity
// and curvature are exact facts, so the survey answers outright: the
// tightest concave principal radius over every face, or nil — the proven
// determination that no concave feature exists.
func WithConcaveRadius() VerifyOption {
	return verifyOption{option.New(identConcaveRadius{}, true)}
}

// WithClearances asks for the minimum gap between disjoint pairs — a
// measurement, not a verdict (verification §2): the clearance spec lives
// with the caller, and the gate judges only the measurement's own figures.
// Each proven-disjoint pair gets a row whose Gap the clearance kernel proves
// (docs/clearance-design.md); a gap the kernel cannot prove yields no row
// and reads Suspect — asked and unanswered, never a fabricated number.
func WithClearances() VerifyOption {
	return verifyOption{option.New(identClearances{}, true)}
}

// Config is the folded option set. toolMM and allowRad carry the wall
// spec resolved to the solver's base units (millimetres, radians).
type Config struct {
	Rel           float64
	Wall          *WallSpec
	ToolMM        float64
	AllowRad      float64
	Pull          *r3.Vec
	ConcaveRadius bool
	Clearances    bool
}

// Resolve folds and validates the options. Every parameter
// error is returned from Verify — never deferred into the report
// (verification §2, core §10).
func Resolve(opts []VerifyOption) (Config, error) {
	cfg := Config{Rel: 1e-3}
	for _, o := range opts {
		if o == nil {
			return Config{}, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identTolerance:
			v, ok := option.Get[units.Value](o)
			if !ok {
				return Config{}, fmt.Errorf(`%w: WithTolerance carries no value`, decaderr.ErrDegenerate)
			}
			rel, err := sectionrecord.MagnitudeIn(v, units.Dimensionless, units.One, "tolerance")
			if err != nil {
				return Config{}, err
			}
			cfg.Rel = rel
		case identMinWall:
			spec, ok := option.Get[WallSpec](o)
			if !ok {
				return Config{}, fmt.Errorf(`%w: WithMinWallThickness carries no spec`, decaderr.ErrDegenerate)
			}
			if spec.NilOption {
				return Config{}, fmt.Errorf(`%w: a nil wall option names nothing to apply`, decaderr.ErrDegenerate)
			}
			tool, err := sectionrecord.MagnitudeIn(spec.Tool, units.Length, units.Millimeter, "wall tool")
			if err != nil {
				return Config{}, err
			}
			if tool == 0 {
				// No thickness is thinner than zero: a comparison with a
				// single outcome states no spec at all (verification §2).
				return Config{}, fmt.Errorf(`%w: a zero wall tool poses no question`, decaderr.ErrDegenerate)
			}
			allow, err := sectionrecord.MagnitudeIn(spec.Allowance, units.Angle, units.Radian, "draft allowance")
			if err != nil {
				return Config{}, err
			}
			if allow >= math.Pi/2 {
				// At 90° or beyond, skins meeting at a square corner would
				// count as opposing: no longer a question about walls
				// (verification §2). The legal range is [0°, 90°).
				return Config{}, fmt.Errorf(`%w: a draft allowance must be under 90 degrees, got %s`, decaderr.ErrDegenerate, spec.Allowance)
			}
			cfg.Wall = &spec
			cfg.ToolMM = tool
			cfg.AllowRad = allow
		case identPullDirection:
			v, ok := option.Get[r3.Vec](o)
			if !ok {
				return Config{}, fmt.Errorf(`%w: WithPullDirection carries no direction`, decaderr.ErrDegenerate)
			}
			for _, c := range []float64{v.X, v.Y, v.Z} {
				if math.IsNaN(c) || math.IsInf(c, 0) {
					return Config{}, fmt.Errorf(`%w: a pull direction must be finite, got %v`, decaderr.ErrNotFinite, v)
				}
			}
			if v.X == 0 && v.Y == 0 && v.Z == 0 {
				return Config{}, fmt.Errorf(`%w: a zero pull direction poses no direction at all`, decaderr.ErrDegenerate)
			}
			cfg.Pull = &v
		case identConcaveRadius:
			cfg.ConcaveRadius = true
		case identClearances:
			cfg.Clearances = true
		}
	}
	return cfg, nil
}

// EffectiveRequest resolves cfg into the effective-request record
// (proposal §5): the validated settings this Verify call actually used,
// canonicalized to millimetres and radians, recorded even for an empty
// document. The pull vector is kept exactly as accepted — never normalized —
// so the recorded request shows the actual input the survey consumed.
func EffectiveRequest(cfg Config) reportvocab.VerifyRequest {
	req := reportvocab.VerifyRequest{
		RelativeTolerance: units.Scalar(cfg.Rel),
		ConcaveRadius:     cfg.ConcaveRadius,
		Clearances:        cfg.Clearances,
	}
	if cfg.Wall != nil {
		req.Wall = &reportvocab.WallRequest{
			Minimum:        units.Millimeters(cfg.ToolMM),
			DraftAllowance: units.Radians(cfg.AllowRad),
		}
	}
	if cfg.Pull != nil {
		req.Undercut = &reportvocab.UndercutRequest{PullDirection: *cfg.Pull}
	}
	return req
}
