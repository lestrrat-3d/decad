package compositesweep

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// SpanPart is one closed single-span reduction before its internal cap is
// removed. StartCap and EndCap follow path direction, including the cap-role
// reversal required by an arc whose Revolve axis was reoriented.
type SpanPart struct {
	Body     *topo.Body
	StartCap *topo.Face
	EndCap   *topo.Face
}

// SpanPartOf reads body's two path-oriented caps from their recorded roles.
func SpanPartOf(body *topo.Body) (SpanPart, error) {
	part := SpanPart{Body: body}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			switch origin.Role {
			case provenance.RoleCapStart:
				if part.StartCap != nil {
					return SpanPart{}, fmt.Errorf(`%w: a sweep span carries more than one start cap`, decaderr.ErrUnsupported)
				}
				part.StartCap = face
			case provenance.RoleCapEnd:
				if part.EndCap != nil {
					return SpanPart{}, fmt.Errorf(`%w: a sweep span carries more than one end cap`, decaderr.ErrUnsupported)
				}
				part.EndCap = face
			}
		}
	}
	if part.StartCap == nil || part.EndCap == nil {
		return SpanPart{}, fmt.Errorf(`%w: a sweep span does not carry both path-oriented caps`, decaderr.ErrUnsupported)
	}
	return part, nil
}

// Assemble removes every internal span cap and reuses one rim edge and its
// endpoint vertices at each path join. The two cap loops are paired by their
// recorded loop and coedge order; no coordinate search or proximity weld
// participates in the pairing. auditSpans proves the spans' separation before
// any sewing, and joinSpans sews one join (SewJoin for analytic spans,
// SewTwistLoftJoin for twisted Loft cells). A closed path also sews the last
// span's end cap to the first span's start cap and keeps no outer cap.
//
// surfaceResult additionally omits the first span's start cap and the last
// span's end cap from the published face set (agent-docs/surface-design.md §4):
// every span still builds and sews solid (sweeprecord.SpanPayload never
// carries the flag, agent-docs/surface-design.md's own span-pairing requirement —
// both cap roles must exist for SpanPartOf and SewJoin to run), so this is
// the ONLY place the flag is consumed. The two outer caps are simply never
// appended to faces and their body back-pointer is never reassigned to body:
// the live-face filter a few lines down keys on that back-pointer, so leaving
// it pointing at the temporary per-span body is what drops each outer rim edge
// to its one surviving wall face with no rim-specific code at all.
//
// Ordering matters and getting it wrong is silent: lumps must be set once
// with the provisional single shell so the body's edge and face walks work,
// then the live-face filter runs, and only THEN may topobuild.SheetLumps run,
// because it reads the adjacency the filter produces. Running it earlier
// would have every shell wrongly report open — topobuild.ShellIsOpen would see every
// discarded per-span face still on each edge's face list.
func Assemble(
	ctx context.Context,
	env topo.Env,
	parts []SpanPart,
	surfaceResult bool,
	auditSpans func(context.Context, []SpanPart) error,
	joinSpans func(context.Context, *topo.Face, *topo.Face, *topo.Body, map[*topo.Edge]struct{}) error,
	closed bool,
) (*topo.Body, error) {
	if len(parts) < 2 {
		return nil, fmt.Errorf(`%w: a composite sweep requires at least two spans`, decaderr.ErrDegenerate)
	}
	if err := ValidateRims(ctx, parts); err != nil {
		return nil, err
	}
	if closed {
		if err := ValidateRims(ctx, []SpanPart{parts[len(parts)-1], parts[0]}); err != nil {
			return nil, err
		}
	}
	spans := make([]SpanPart, len(parts))
	copy(spans, parts)
	if err := auditSpans(ctx, spans); err != nil {
		return nil, err
	}

	sewnEdges := make(map[*topo.Edge]struct{})
	for join := 0; join+1 < len(parts); join++ {
		if err := joinSpans(
			ctx,
			parts[join].EndCap,
			parts[join+1].StartCap,
			parts[join].Body,
			sewnEdges,
		); err != nil {
			return nil, WrapStageError(fmt.Sprintf(`sweep path join %d`, join+1), err)
		}
	}
	if closed {
		last := parts[len(parts)-1]
		if err := joinSpans(ctx, last.EndCap, parts[0].StartCap, last.Body, sewnEdges); err != nil {
			return nil, WrapStageError(`sweep path closure`, err)
		}
	}

	kind := topo.BodySolid
	if surfaceResult {
		kind = topo.BodySheet
	}
	body := env.Alloc.NewBody()
	body.Solid = !surfaceResult
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = kind
	body.Doc = env.Doc
	var faces []*topo.Face
	if !surfaceResult && !closed {
		faces = append(faces, parts[0].StartCap)
	}
	for k, part := range parts {
		for _, face := range topobuild.Faces(part.Body) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if face == part.StartCap || face == part.EndCap {
				continue
			}
			RewriteFaceSideRoles(face, k)
			face.Body = body
			faces = append(faces, face)
		}
	}
	if !surfaceResult && !closed {
		parts[0].StartCap.Body = body
		parts[len(parts)-1].EndCap.Body = body
		faces = append(faces, parts[len(parts)-1].EndCap)
	}
	shell := env.Alloc.NewFaceShell()
	shell.Faces = faces
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	for _, edge := range topobuild.Edges(body) {
		edge.Faces = LiveFaces(edge.Faces, body)
	}
	if surfaceResult {
		// topobuild.SheetLumps splits by connectivity (agent-docs/surface-design.md §2.2): a
		// holed profile's outer and hole wall tubes touch nowhere once the two
		// outer caps that used to bridge them are gone, so each becomes its own
		// Lump. It must run after the live-face filter above, per this
		// function's own doc comment.
		body.Lumps = topobuild.SheetLumps(env.Alloc, faces)
	}
	if err := AuditBoundary(ctx, body, sewnEdges); err != nil {
		return nil, err
	}
	if closed {
		for _, edge := range topobuild.Edges(body) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if len(edge.Faces) != 2 {
				return nil, fmt.Errorf(`%w: a closed sweep rim lacks two incident faces`, decaderr.ErrUnsupported)
			}
		}
	}
	return body, nil
}

// ValidateRims checks that each adjacent pair of parts meets with matching
// rim topology: the same loop count, the same edge count per loop, complete
// rim edges, and no rim edge reused at both path ends of one span.
func ValidateRims(ctx context.Context, parts []SpanPart) error {
	budget := proofbound.NewWorkBudget(ctx)
	for join := 0; join+1 < len(parts); join++ {
		if err := budget.Err(); err != nil {
			return err
		}
		from, to := parts[join].EndCap, parts[join+1].StartCap
		if len(from.Loops) != len(to.Loops) {
			return fmt.Errorf(`%w: adjacent sweep sections have different loop counts`, decaderr.ErrUnsupported)
		}
		for li := range from.Loops {
			if err := budget.Step(); err != nil {
				return err
			}
			if len(from.Loops[li].CoEdges) != len(to.Loops[li].CoEdges) {
				return fmt.Errorf(`%w: adjacent sweep section loop %d has different edge counts`, decaderr.ErrUnsupported, li)
			}
			for ci := range from.Loops[li].CoEdges {
				if err := budget.Step(); err != nil {
					return err
				}
				fromUse := from.Loops[li].CoEdges[ci]
				toUse := to.Loops[li].CoEdges[ci]
				fromEdge := fromUse.Edge
				toEdge := toUse.Edge
				if fromEdge == nil || toEdge == nil || fromEdge.Start == nil || fromEdge.End == nil || toEdge.Start == nil || toEdge.End == nil {
					return fmt.Errorf(`%w: an adjacent sweep section carries incomplete rim topology`, decaderr.ErrUnsupported)
				}
				if fromEdge == toEdge {
					return fmt.Errorf(`%w: a span reuses one rim edge at both path ends`, decaderr.ErrUnsupported)
				}
			}
		}
	}
	return budget.Err()
}

// WrapStageError names the stage of a build that failed. A canceled or
// expired context passes through unwrapped, so a caller reads ctx.Err() itself.
// Assemble names its join stages through it, and internal/draftrecord its
// wall and half-taper stages.
func WrapStageError(stage string, err error) error {
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	return fmt.Errorf(`%s: %w`, stage, err)
}
