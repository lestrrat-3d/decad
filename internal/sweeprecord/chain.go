package sweeprecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ChainSpanPayload keeps the two real chain reductions separate. A
// composite chain has no caps: each span's wall supplies its path-end rims.
type ChainSpanPayload struct {
	Chain          chainrecord.Payload
	Revolve        revolveplan.ChainPayload
	Arc            bool
	ReverseArcRims bool
}

func (sp ChainSpanPayload) transform() r3.Transform {
	if sp.Arc {
		return sp.Revolve.Xform
	}
	return sp.Chain.Xform
}

func (sp *ChainSpanPayload) build(ctx context.Context, env topo.Env,
	work *freeform.FreeformWork) (*topo.Body, error) {
	if sp.Arc {
		return revolveplan.BuildChain(ctx, env, sp.Revolve, work)
	}
	return chainrecord.Build(ctx, env, sp.Chain, work)
}

func ReplayChainWork(ctx context.Context, env topo.Env,
	payload ChainPayload, work *freeform.FreeformWork) (*topo.Body, error) {
	if len(payload.Spans) != 2 || payload.Spans[0].Arc || !payload.Spans[1].Arc {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no admitted span sequence`, decaderr.ErrUnsupported)
	}
	parts := make([]*topo.Body, 2)
	for i := range payload.Spans {
		part, err := payload.Spans[i].build(ctx, env, work)
		if err != nil {
			return nil, fmt.Errorf("sweep path span %d: %w", i, err)
		}
		if len(topobuild.Faces(part)) != 1 || part.Kind != topo.BodySheet {
			return nil, fmt.Errorf(`%w: composite chain Sweep span %d has no single source wall`, decaderr.ErrUnsupported, i)
		}
		parts[i] = part
	}
	if err := auditChainSeparation(ctx, payload, parts); err != nil {
		return nil, err
	}
	previous, err := chainSpanRims(parts[0], false, false)
	if err != nil {
		return nil, err
	}
	following, err := chainSpanRims(parts[1], true, payload.Spans[1].ReverseArcRims)
	if err != nil {
		return nil, err
	}
	if err := certifyChainRimPair(previous.end, following.start); err != nil {
		return nil, err
	}
	// Both edges denote the same two endpoints of the one recorded source
	// line. The coordinate comparison above only rejects a failed restatement;
	// it does not establish their source identity.
	compositesweep.RewriteVertex(parts[0], previous.end.Start, following.start.Start)
	compositesweep.RewriteVertex(parts[0], previous.end.End, following.start.End)
	compositesweep.RewriteEdge(parts[0], previous.end, following.start)
	following.start.Faces = append(following.start.Faces, previous.end.Faces...)

	body := env.Alloc.NewBody()
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	faces := []*topo.Face{topobuild.Faces(parts[0])[0], topobuild.Faces(parts[1])[0]}
	for i, face := range faces {
		compositesweep.RewriteFaceSideRoles(face, i)
		face.Body = body
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, faces)
	if err := compositesweep.AuditBoundary(ctx, body, map[*topo.Edge]struct{}{following.start: {}}); err != nil {
		return nil, err
	}
	area := proofbound.BoundedAdd(
		proofbound.MeasuredScalar(faces[0].Area, faces[0].AreaBound),
		proofbound.MeasuredScalar(faces[1].Area, faces[1].AreaBound))
	body.Area = compositesweep.ScalarMeasurement(area, units.SquareMillimeter)
	body.Bounds = chainBox(parts[0].Bounds, parts[1].Bounds)
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = payload
	return body, nil
}

type chainRimPair struct{ start, end *topo.Edge }

func chainSpanRims(part *topo.Body, arc, reverse bool) (chainRimPair, error) {
	face := topobuild.Faces(part)[0]
	if len(face.Loops) != 1 || len(face.Loops[0].CoEdges) != 4 {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain span has no four-edge source wall`, decaderr.ErrUnsupported)
	}
	coedges := face.Loops[0].CoEdges
	start, end := coedges[0].Edge, coedges[2].Edge
	if arc && reverse {
		start, end = end, start
	}
	if start == nil || end == nil || start == end {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain span has incomplete path rims`, decaderr.ErrUnsupported)
	}
	if _, ok := start.Curve.(surfacegeom.Line3); !ok {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain start rim is not source-linear`, decaderr.ErrUnsupported)
	}
	if _, ok := end.Curve.(surfacegeom.Line3); !ok {
		return chainRimPair{}, fmt.Errorf(`%w: a composite chain end rim is not source-linear`, decaderr.ErrUnsupported)
	}
	return chainRimPair{start: start, end: end}, nil
}

func certifyChainRimPair(from, to *topo.Edge) error {
	return compositesweep.CertifyRimPair(
		[2]*compositesweep.RimPost{chainRimPost(from.Start), chainRimPost(from.End)},
		[2]*compositesweep.RimPost{chainRimPost(to.Start), chainRimPost(to.End)})
}

func chainRimPost(v *topo.Vertex) *compositesweep.RimPost {
	if v == nil {
		return nil
	}
	return &compositesweep.RimPost{Position: v.Position, Bound: v.Bound.Base()}
}

type chainSweepExtent struct{ span ChainSpanPayload }

func (e chainSweepExtent) AuditExtent(ctx context.Context, direction r3.Vec,
	work *freeform.FreeformWork) (float64, float64, float64, error) {
	if e.span.Arc {
		return e.span.Revolve.Revolve().ExtentBoundedAlong(ctx, direction, work)
	}
	return e.span.Chain.Prism().ExtentBoundedAlong(ctx, direction, work, nil)
}

func auditChainSeparation(ctx context.Context, payload ChainPayload, parts []*topo.Body) error {
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, payload.Path, sectionrecord.PlaneRecord{
		Origin: payload.Spans[0].Chain.Frame.Origin(),
		U:      payload.Spans[0].Chain.Frame.U(), V: payload.Spans[0].Chain.Frame.V(),
	})
	if err != nil {
		return err
	}
	audit := make([]compositesweep.AuditSpan, 2)
	for i := range audit {
		box := parts[i].Bounds
		// Audit takes the incoming start-cap normal. A chain span has no cap,
		// so derive that opposite-tangent plane from its certified frame.
		incoming, frameErr := r3.NewFrame(frames[i].Frame.Origin(),
			frames[i].Frame.U(), frames[i].Frame.V().Scale(-1))
		if frameErr != nil {
			return fmt.Errorf(`%w: composite chain join plane has no frame: %v`, decaderr.ErrUnsupported, frameErr)
		}
		audit[i] = compositesweep.AuditSpan{
			StartPlane:  surfacegeom.Plane{Frame: incoming},
			StartPlanar: true, EndPlanar: true, EndpointSupports: true,
			Box:    compositesweep.AuditBox{Min: box.Min, Max: box.Max, Bound: box.Bound.Base()},
			Extent: chainSweepExtent{span: payload.Spans[i]},
		}
	}
	return compositesweep.Audit(ctx, audit)
}

func chainBox(a, b measurement.Box) measurement.Box {
	bound := math.Max(a.Bound.Base(), b.Bound.Base())
	return measurement.Box{
		Min:   r3.NewVec(math.Min(a.Min.X, b.Min.X), math.Min(a.Min.Y, b.Min.Y), math.Min(a.Min.Z, b.Min.Z)),
		Max:   r3.NewVec(math.Max(a.Max.X, b.Max.X), math.Max(a.Max.Y, b.Max.Y), math.Max(a.Max.Z, b.Max.Z)),
		Bound: units.Millimeters(bound), Exactness: measurement.ExactnessOf(bound),
	}
}
