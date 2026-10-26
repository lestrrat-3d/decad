package compositesweep

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
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
			CoplanarSections: bodyHalfTurn(span.Body),
			Sections:         bodySections{body: span.Body},
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
		width, ok := exactTurnWidth(payload)
		return ok && width.Sign() > 0 && width.Cmp(big.NewRat(1, 2)) <= 0
	default:
		return false
	}
}

// bodyHalfTurn reports whether a span body is an exact half-turn revolve,
// whose two end caps both lie in the plane through its axis.
func bodyHalfTurn(body *topo.Body) bool {
	payload, ok := body.Payload.(revolveplan.Payload)
	return ok && RevolveHalfTurn(payload)
}

// RevolveHalfTurn reports whether a partial revolve denotes exactly half a
// turn. Its two end sections then lie in one plane.
func RevolveHalfTurn(payload revolveplan.Payload) bool {
	if payload.Full {
		return false
	}
	width, ok := exactTurnWidth(payload)
	return ok && width.Cmp(big.NewRat(1, 2)) == 0
}

func exactTurnWidth(payload revolveplan.Payload) (*big.Rat, bool) {
	from, to := payload.Den.Phi0, payload.Den.Phi1
	if !from.Valid() || !to.Valid() || from.Span != nil || to.Span != nil ||
		from.Rad.Sign() != 0 || to.Rad.Sign() != 0 {
		return nil, false
	}
	return new(big.Rat).Sub(to.Turn, from.Turn), true
}

// bodySections reads the section boxes of a revolve span body.
type bodySections struct {
	body *topo.Body
}

func (span bodySections) AuditSectionBoxes(
	ctx context.Context, work *freeform.FreeformWork,
) (AuditBox, AuditBox, error) {
	payload, ok := span.body.Payload.(revolveplan.Payload)
	if !ok {
		return AuditBox{}, AuditBox{}, fmt.Errorf(`%w: a composite sweep span has no section box certificate`, decaderr.ErrUnsupported)
	}
	return RevolveSections{Payload: payload}.AuditSectionBoxes(ctx, work)
}

// RevolveSections reads the section boxes of a partial revolve built as a
// sweep span. The span's path-start section is the end whose denoted angle is
// exactly zero, the section its transported frame names. Each box is the
// payload's own bounded extent reading with its angular interval closed to
// that one end angle, so it charges the same frame, placement, boundary and
// angle bounds as the span's box.
type RevolveSections struct {
	Payload revolveplan.Payload
}

func (span RevolveSections) AuditSectionBoxes(
	ctx context.Context, work *freeform.FreeformWork,
) (AuditBox, AuditBox, error) {
	payload := span.Payload
	from, to := payload.Den.Phi0, payload.Den.Phi1
	if payload.Full || !exactZeroAngle(from) && !exactZeroAngle(to) {
		return AuditBox{}, AuditBox{}, fmt.Errorf(`%w: a composite sweep arc span has no exact path-start section`, decaderr.ErrUnsupported)
	}
	first := payload
	first.Phi1, first.Den.Phi1 = payload.Phi0, payload.Den.Phi0
	last := payload
	last.Phi0, last.Den.Phi0 = payload.Phi1, payload.Den.Phi1
	firstBox, err := revolveplan.Bounds(ctx, first, work)
	if err != nil {
		return AuditBox{}, AuditBox{}, err
	}
	lastBox, err := revolveplan.Bounds(ctx, last, work)
	if err != nil {
		return AuditBox{}, AuditBox{}, err
	}
	start := AuditBox{Min: firstBox.Min, Max: firstBox.Max, Bound: firstBox.Bound.Base()}
	end := AuditBox{Min: lastBox.Min, Max: lastBox.Max, Bound: lastBox.Bound.Base()}
	if exactZeroAngle(from) {
		return start, end, nil
	}
	return end, start, nil
}

func exactZeroAngle(angle revolveangle.Angle) bool {
	return angle.Valid() && angle.Span == nil && angle.Rad.Sign() == 0 && angle.Turn.Sign() == 0
}
