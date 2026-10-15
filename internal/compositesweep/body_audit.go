package compositesweep

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// EndpointSupporter is implemented by a span payload whose local construction
// proves each endpoint cap is a supporting plane (see endpointSupports).
type EndpointSupporter interface {
	AuditEndpointSupports() bool
}

// bodyExtent reads the analytic extent of one closed span's body node
// for Audit.
type bodyExtent struct {
	body *topo.Body
}

// AuditBodies audits the spans of an open composite sweep.
func AuditBodies(ctx context.Context, spans []SpanPart) error {
	return auditBodiesWithClosure(ctx, spans, false)
}

// AuditClosedBodies audits the spans of a closed composite sweep.
func AuditClosedBodies(ctx context.Context, spans []SpanPart) error {
	return auditBodiesWithClosure(ctx, spans, true)
}

func auditBodiesWithClosure(ctx context.Context, spans []SpanPart, closed bool) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(spans) < 2 {
		return Audit(ctx, nil)
	}
	audit := make([]AuditSpan, len(spans))
	for i, span := range spans {
		if span.Body == nil || span.StartCap == nil || span.EndCap == nil {
			return fmt.Errorf(`%w: composite sweep span %d has incomplete audit geometry`, decaderr.ErrUnsupported, i)
		}
		if span.StartCap.Body != span.Body || span.EndCap.Body != span.Body {
			return fmt.Errorf(`%w: composite sweep span %d has a cap from another body`, decaderr.ErrUnsupported, i)
		}
		startPlane, startPlanar := span.StartCap.Surface.(surfacegeom.Plane)
		_, endPlanar := span.EndCap.Surface.(surfacegeom.Plane)
		bounds := span.Body.Bounds
		audit[i] = AuditSpan{
			StartPlane:       startPlane,
			StartPlanar:      startPlanar,
			EndPlanar:        endPlanar,
			EndpointSupports: endpointSupports(span.Body),
			Box: AuditBox{
				Min: bounds.Min, Max: bounds.Max, Bound: bounds.Bound.Base(),
			},
			Extent: bodyExtent{body: span.Body},
		}
	}
	if closed {
		return AuditClosed(ctx, audit)
	}
	return Audit(ctx, audit)
}

// AuditExtent reads the analytic extent of a built sweep span. A payload kind
// other than a prism or a revolve states its own extent through ExtentReader.
func (span bodyExtent) AuditExtent(
	ctx context.Context, direction r3.Vec, work *freeform.FreeformWork,
) (float64, float64, float64, error) {
	switch payload := span.body.Payload.(type) {
	case prismrecord.Payload:
		return payload.ExtentBoundedAlong(ctx, direction, work, payload.Walks)
	case revolveplan.Payload:
		return payload.ExtentBoundedAlong(ctx, direction, work)
	case ExtentReader:
		return payload.AuditExtent(ctx, direction, work)
	default:
		return 0, 0, 0, fmt.Errorf(`%w: a composite sweep span has no analytic extent certificate`, decaderr.ErrUnsupported)
	}
}

// endpointSupports reports whether a span's local construction
// proves each endpoint cap is a supporting plane. A prism has that property
// for both ends. For an exact-turn arc no wider than a half turn, Revolve's
// half-plane gate then keeps the whole swept section between its two endpoint
// tangent planes.
func endpointSupports(body *topo.Body) bool {
	switch payload := body.Payload.(type) {
	case prismrecord.Payload:
		return true
	case EndpointSupporter:
		return payload.AuditEndpointSupports()
	case revolveplan.Payload:
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
