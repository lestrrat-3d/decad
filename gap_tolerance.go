package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// gapOption is the DocumentOption WithGapTolerance and ExactOnly return.
type gapOption struct{ option.Interface }

func (gapOption) documentOption() {}

type identGapTolerance struct{}

// WithGapTolerance sets the farthest the evaluator may move a held vertex from
// the exact point it stands for to keep a result embedded. A result the exact
// rounding would fold through itself, and so refuse, is kept when moving the
// offending vertices by up to tol clears it; the move is charged to that
// vertex's bound, so the result reads Approximate with a Bound that covers it
// (agent-docs/gap-tolerance-design.md). A document with no option uses
// 0.01 mm. A value that is not a length is ErrUnitKind, a non-finite one
// ErrNotFinite and a negative one ErrNegativeMagnitude, all when an operation
// that reads it starts. A zero length is ExactOnly.
func WithGapTolerance(tol units.Value) DocumentOption {
	return gapOption{option.New(identGapTolerance{}, tol)}
}

// ExactOnly keeps every result exact: the evaluator never moves a held vertex
// off the exact point it stands for beyond the float rounding that point
// needs, and refuses a result that only such a move would keep embedded.
func ExactOnly() DocumentOption {
	return WithGapTolerance(units.Millimeters(0))
}

// gapReachOf resolves a document's options to the reach in millimetres its
// operations carry. The last gap option wins. It returns the error the first
// invalid option names, which every operation that reads the reach reports.
func gapReachOf(opts []DocumentOption) (float64, error) {
	reach := gaptol.Default
	for _, opt := range opts {
		if opt == nil {
			return 0, fmt.Errorf(`%w: a nil document option names nothing to apply`, ErrDegenerate)
		}
		if p, ok := opt.(paramsOption); ok {
			if p.Interface == nil {
				return 0, fmt.Errorf(`%w: a zero document option names nothing to apply`, ErrDegenerate)
			}
			continue
		}
		if _, ok := opt.(chainOption); ok {
			continue
		}
		if g, ok := opt.(gapOption); ok && g.Interface == nil {
			return 0, fmt.Errorf(`%w: a zero document option names nothing to apply`, ErrDegenerate)
		}
		tol, ok := option.Get[units.Value](opt)
		if !ok {
			return 0, fmt.Errorf(`%w: a document option carries no tolerance`, ErrDegenerate)
		}
		if tol.Kind() != units.Length {
			return 0, fmt.Errorf(`%w: the gap tolerance must be a length`, ErrUnitKind)
		}
		mm := tol.Base()
		if proofbound.IsNonFinite(mm) {
			return 0, fmt.Errorf(`%w: the gap tolerance must be finite`, ErrNotFinite)
		}
		if mm < 0 {
			return 0, fmt.Errorf(`%w: the gap tolerance must not be negative`, ErrNegativeMagnitude)
		}
		reach = mm
	}
	return reach, nil
}

// evalContext returns ctx carrying d's gap tolerance for the evaluator, or the
// error its options named. Every root operation that can keep a held mesh
// embedded by moving a vertex calls it before it evaluates.
func (d *Document) evalContext(ctx context.Context) (context.Context, error) {
	if d.gapErr != nil {
		return nil, d.gapErr
	}
	if d.chainErr != nil {
		return nil, d.chainErr
	}
	return chaintol.With(gaptol.With(ctx, d.gapReach), d.chainTol), nil
}
