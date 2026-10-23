package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// chainOption is the DocumentOption WithChainTolerance returns.
type chainOption struct{ option.Interface }

func (chainOption) documentOption() {}

type identChainTolerance struct{}

// WithChainTolerance sets the coarsest bound a held facet may carry where a
// boolean's two operands meet. The default is the pair's own chord tolerance,
// so a Cut, Union or Intersect whose contact touches facets that an earlier
// boolean left coarser than that is refused with a message naming the facet's
// bound and the tolerance. A document that sets this admits a contact whose
// touched facets are within tol. Every bound the result carries is still
// proven; a larger tol lets a chain of booleans over curved solids reach a
// larger bound, and the result reads Approximate with a Bound that covers it.
// A value that is not a length is ErrUnitKind, a non-finite one ErrNotFinite
// and a negative one ErrNegativeMagnitude, all when an operation that reads
// it starts. A zero length is the default.
func WithChainTolerance(tol units.Value) DocumentOption {
	return chainOption{option.New(identChainTolerance{}, tol)}
}

// chainToleranceOf resolves a document's options to the chain tolerance in
// millimetres its operations carry. The last chain option wins. It returns the
// error the first invalid chain option names.
func chainToleranceOf(opts []DocumentOption) (float64, error) {
	var tol float64
	for _, opt := range opts {
		c, ok := opt.(chainOption)
		if !ok {
			continue
		}
		if c.Interface == nil {
			return 0, fmt.Errorf(`%w: a zero document option names nothing to apply`, ErrDegenerate)
		}
		v, ok := option.Get[units.Value](c)
		if !ok {
			return 0, fmt.Errorf(`%w: a document option carries no tolerance`, ErrDegenerate)
		}
		if v.Kind() != units.Length {
			return 0, fmt.Errorf(`%w: the chain tolerance must be a length`, ErrUnitKind)
		}
		mm := v.Base()
		if proofbound.IsNonFinite(mm) {
			return 0, fmt.Errorf(`%w: the chain tolerance must be finite`, ErrNotFinite)
		}
		if mm < 0 {
			return 0, fmt.Errorf(`%w: the chain tolerance must not be negative`, ErrNegativeMagnitude)
		}
		tol = mm
	}
	return tol, nil
}
