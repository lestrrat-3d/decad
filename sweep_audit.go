package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/r3"
)

// sweepAuditSpan is one closed analytic span before composite Sweep removes
// its internal caps. The cap pointers follow path direction.
type sweepAuditSpan struct {
	body     *Body
	startCap *Face
	endCap   *Face
}

func auditCompositeSweep(ctx context.Context, spans []sweepAuditSpan) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(spans) < 2 {
		return compositesweep.Audit(ctx, nil)
	}
	audit := make([]compositesweep.AuditSpan, len(spans))
	for i, span := range spans {
		if span.body == nil || span.startCap == nil || span.endCap == nil {
			return fmt.Errorf(`%w: composite sweep span %d has incomplete audit geometry`, ErrUnsupported, i)
		}
		if span.startCap.body != span.body || span.endCap.body != span.body {
			return fmt.Errorf(`%w: composite sweep span %d has a cap from another body`, ErrUnsupported, i)
		}
		startPlane, startPlanar := span.startCap.surface.(Plane)
		_, endPlanar := span.endCap.surface.(Plane)
		bounds := span.body.bounds
		audit[i] = compositesweep.AuditSpan{
			StartPlane:       startPlane,
			StartPlanar:      startPlanar,
			EndPlanar:        endPlanar,
			EndpointSupports: sweepAuditEndpointSupports(span.body),
			Box: compositesweep.AuditBox{
				Min: bounds.Min, Max: bounds.Max, Bound: bounds.Bound.Base(),
			},
			Extent: span,
		}
	}
	return compositesweep.Audit(ctx, audit)
}

// AuditExtent reads the analytic extent of a built sweep span.
func (span sweepAuditSpan) AuditExtent(
	ctx context.Context, direction r3.Vec, work *freeform.FreeformWork,
) (float64, float64, float64, error) {
	switch payload := span.body.payload.(type) {
	case prismPayload:
		return payload.extentBoundedAlong(ctx, direction, work, payload.walks)
	case revolvePayload:
		return payload.extentBoundedAlong(ctx, direction, work)
	case draftPayload:
		return payload.band().extentBoundedAlong(ctx, direction, work)
	default:
		return 0, 0, 0, fmt.Errorf(`%w: a composite sweep span has no analytic extent certificate`, ErrUnsupported)
	}
}

// sweepAuditEndpointSupports reports whether a span's local construction
// proves each endpoint cap is a supporting plane. A prism has that property
// for both ends. For an exact-turn arc no wider than a half turn, Revolve's
// half-plane gate then keeps the whole swept section between its two endpoint
// tangent planes.
func sweepAuditEndpointSupports(body *Body) bool {
	switch payload := body.payload.(type) {
	case prismPayload:
		return true
	case draftPayload:
		// Each draft slab stays between its two axial cap planes.
		return true
	case revolvePayload:
		if payload.full {
			return false
		}
		from, to := payload.den.Phi0, payload.den.Phi1
		if !from.Valid() || !to.Valid() || from.Span != nil || to.Span != nil ||
			from.Rad.Sign() != 0 || to.Rad.Sign() != 0 {
			return false
		}
		width := new(big.Rat).Sub(to.Turn, from.Turn)
		return width.Sign() > 0 && width.Cmp(big.NewRat(1, 2)) <= 0
	default:
		return false
	}
}
