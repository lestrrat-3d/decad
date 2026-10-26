package sweeprecord

import (
	"context"
	"fmt"
	"math"
	"slices"

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

// ReplayChainWork builds every span of a composite chain Sweep and sews each
// join by recorded segment index (agent-docs/sweep-design.md §15.1, §15.8).
// Each span publishes one wall per coalesced walk of the one recorded chain;
// wall j of span k and wall j of span k+1 must carry the same recorded
// segment roles, and the end rim of the first is replaced by the start rim of
// the second. The coordinate comparison at each pair can only reject.
func ReplayChainWork(ctx context.Context, env topo.Env,
	payload ChainPayload, work *freeform.FreeformWork) (*topo.Body, error) {
	if len(payload.Spans) < 2 {
		return nil, fmt.Errorf(`%w: composite chain Sweep has no admitted span sequence`, decaderr.ErrUnsupported)
	}
	parts := make([]*topo.Body, len(payload.Spans))
	rims := make([][]chainRimPair, len(payload.Spans))
	for i := range payload.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		part, err := payload.Spans[i].build(ctx, env, work)
		if err != nil {
			return nil, fmt.Errorf("sweep path span %d: %w", i, err)
		}
		if part.Kind != topo.BodySheet || len(topobuild.Faces(part)) == 0 {
			return nil, fmt.Errorf(`%w: composite chain Sweep span %d has no source wall`, decaderr.ErrUnsupported, i)
		}
		parts[i] = part
		rims[i], err = chainSpanRims(part, payload.Spans[i].Arc, payload.Spans[i].ReverseArcRims)
		if err != nil {
			return nil, fmt.Errorf("sweep path span %d: %w", i, err)
		}
	}
	if err := auditChainSeparation(ctx, payload, parts); err != nil {
		return nil, err
	}
	sewn := make(map[*topo.Edge]struct{})
	for k := 0; k+1 < len(parts); k++ {
		if err := sewChainJoin(ctx, parts[k], rims[k], rims[k+1], k, sewn); err != nil {
			return nil, err
		}
	}

	body := env.Alloc.NewBody()
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	var faces []*topo.Face
	area := proofbound.BoundedScalar{}
	box := parts[0].Bounds
	for i, part := range parts {
		for _, face := range topobuild.Faces(part) {
			compositesweep.RewriteFaceSideRoles(face, i)
			face.Body = body
			faces = append(faces, face)
			area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(face.Area, face.AreaBound))
		}
		if i > 0 {
			box = chainBox(box, part.Bounds)
		}
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, faces)
	if err := compositesweep.AuditBoundary(ctx, body, sewn); err != nil {
		return nil, err
	}
	body.Area = compositesweep.ScalarMeasurement(area, units.SquareMillimeter)
	body.Bounds = box
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = payload
	return body, nil
}

// sewChainJoin sews the join between span k (previous) and span k+1. Wall j's
// end rim on the previous span and wall j's start rim on the next span denote
// the same recorded segment j under the same exact section placement; the
// face roles prove that both walls come from the same recorded segments. The
// next span's rim edge and vertices are kept.
func sewChainJoin(ctx context.Context, previous *topo.Body, from, to []chainRimPair, k int,
	sewn map[*topo.Edge]struct{}) error {
	if len(from) != len(to) {
		return fmt.Errorf(`%w: composite chain Sweep spans %d and %d have %d and %d source walls`,
			decaderr.ErrUnsupported, k, k+1, len(from), len(to))
	}
	// The post pairing must be one-to-one: one old post with two new posts
	// would split a vertex, and two old posts with one new post would merge
	// two source endpoints.
	vertices := make(map[*topo.Vertex]*topo.Vertex)
	sources := make(map[*topo.Vertex]*topo.Vertex)
	pairVertex := func(old, replacement *topo.Vertex) error {
		if prior, ok := vertices[old]; ok && prior != replacement {
			return fmt.Errorf(`%w: composite chain Sweep join %d pairs one rim post with two`, decaderr.ErrUnsupported, k)
		}
		if prior, ok := sources[replacement]; ok && prior != old {
			return fmt.Errorf(`%w: composite chain Sweep join %d pairs two rim posts with one`, decaderr.ErrUnsupported, k)
		}
		vertices[old] = replacement
		sources[replacement] = old
		return nil
	}
	for j := range from {
		if err := ctx.Err(); err != nil {
			return err
		}
		if !slices.Equal(from[j].roles, to[j].roles) {
			return fmt.Errorf(`%w: composite chain Sweep join %d wall %d does not pair one recorded segment set`,
				decaderr.ErrUnsupported, k, j)
		}
		if err := certifyChainRimPair(from[j].end, to[j].start); err != nil {
			return err
		}
		if err := pairVertex(from[j].end.Start, to[j].start.Start); err != nil {
			return err
		}
		if err := pairVertex(from[j].end.End, to[j].start.End); err != nil {
			return err
		}
	}
	// Both edges of every pair denote the same endpoints of one recorded
	// source segment. The coordinate comparison above only rejects a failed
	// restatement; it does not establish their source identity.
	for old, replacement := range vertices {
		compositesweep.RewriteVertex(previous, old, replacement)
	}
	for j := range from {
		compositesweep.RewriteEdge(previous, from[j].end, to[j].start)
		to[j].start.Faces = append(to[j].start.Faces, from[j].end.Faces...)
		sewn[to[j].start] = struct{}{}
	}
	return nil
}

// chainRimPair is one wall's path-start and path-end rim, beside the
// recorded-segment roles of the wall that owns them.
type chainRimPair struct {
	start, end *topo.Edge
	roles      []string
}

// chainSpanRims reads one rim pair per wall face of a built chain span, in
// the walk order the span builder published.
func chainSpanRims(part *topo.Body, arc, reverse bool) ([]chainRimPair, error) {
	faces := topobuild.Faces(part)
	out := make([]chainRimPair, len(faces))
	for i, face := range faces {
		if len(face.Loops) != 1 || len(face.Loops[0].CoEdges) != 4 {
			return nil, fmt.Errorf(`%w: a composite chain span has no four-edge source wall`, decaderr.ErrUnsupported)
		}
		coedges := face.Loops[0].CoEdges
		start, end := coedges[0].Edge, coedges[2].Edge
		if arc && reverse {
			start, end = end, start
		}
		if start == nil || end == nil || start == end {
			return nil, fmt.Errorf(`%w: a composite chain span has incomplete path rims`, decaderr.ErrUnsupported)
		}
		if _, ok := start.Curve.(surfacegeom.Line3); !ok {
			return nil, fmt.Errorf(`%w: a composite chain start rim is not source-linear`, decaderr.ErrUnsupported)
		}
		if _, ok := end.Curve.(surfacegeom.Line3); !ok {
			return nil, fmt.Errorf(`%w: a composite chain end rim is not source-linear`, decaderr.ErrUnsupported)
		}
		roles := make([]string, len(face.Origins))
		for oi, origin := range face.Origins {
			roles[oi] = origin.Role
		}
		out[i] = chainRimPair{start: start, end: end, roles: roles}
	}
	return out, nil
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
	// The section plane is read off the first span's chain prism, which
	// sweepinput.CompositeChainSweepPath's line-first gate guarantees.
	if payload.Spans[0].Arc {
		return fmt.Errorf(`%w: the composite chain Sweep separation audit has no first line span`, decaderr.ErrUnsupported)
	}
	frames, err := sweeptransport.TransportSweepFramesContext(ctx, payload.Path, sectionrecord.PlaneRecord{
		Origin: payload.Spans[0].Chain.Frame.Origin(),
		U:      payload.Spans[0].Chain.Frame.U(), V: payload.Spans[0].Chain.Frame.V(),
	})
	if err != nil {
		return err
	}
	if len(frames) != len(parts)+1 {
		return fmt.Errorf(`%w: composite chain Sweep has no complete transported frame chain`, decaderr.ErrUnsupported)
	}
	audit := make([]compositesweep.AuditSpan, len(parts))
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
