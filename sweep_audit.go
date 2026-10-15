package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
	return auditCompositeSweepWithClosure(ctx, spans, false)
}

func auditClosedCompositeSweep(ctx context.Context, spans []sweepAuditSpan) error {
	return auditCompositeSweepWithClosure(ctx, spans, true)
}

func auditCompositeSweepWithClosure(ctx context.Context, spans []sweepAuditSpan, closed bool) error {
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
		if span.startCap.n.Body != bodyNode(span.body) || span.endCap.n.Body != bodyNode(span.body) {
			return fmt.Errorf(`%w: composite sweep span %d has a cap from another body`, ErrUnsupported, i)
		}
		startPlane, startPlanar := span.startCap.n.Surface.(surfacegeom.Plane)
		_, endPlanar := span.endCap.n.Surface.(surfacegeom.Plane)
		bounds := span.body.n.Bounds
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
	if closed {
		return compositesweep.AuditClosed(ctx, audit)
	}
	return compositesweep.Audit(ctx, audit)
}

// AuditExtent reads the analytic extent of a built sweep span.
func (span sweepAuditSpan) AuditExtent(
	ctx context.Context, direction r3.Vec, work *freeform.FreeformWork,
) (float64, float64, float64, error) {
	switch payload := span.body.n.Payload.(type) {
	case prismPayload:
		return payload.ExtentBoundedAlong(ctx, direction, work, payload.Walks)
	case revolvePayload:
		return payload.ExtentBoundedAlong(ctx, direction, work)
	case draftPayload:
		return payload.band().ExtentBoundedAlong(ctx, direction, work)
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
	switch payload := body.n.Payload.(type) {
	case prismPayload:
		return true
	case draftPayload:
		// Each draft slab stays between its two axial cap planes.
		return true
	case revolvePayload:
		if payload.Full {
			return false
		}
		from, to := payload.Den.Phi0, payload.Den.Phi1
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
