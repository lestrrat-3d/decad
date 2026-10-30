package revolvebore

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// AddBoundary adds the certified hole to a freshly built Revolve body.
// The caller must finish measurement and mesh proofs before publishing it.
func AddBoundary(ctx context.Context, env topo.Env, body *topo.Body, candidate Candidate) (proofbound.BoundedScalar, error) {
	if len(body.Lumps) != 1 || len(body.Lumps[0].Shells) != 1 {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore target has no single shell", decaderr.ErrUnsupported)
	}
	faces := resultFacesByRole(body)
	for role, origins := range candidate.TargetOrigins {
		if face := faces[role]; face != nil {
			for _, origin := range origins {
				if !slices.Contains(face.Origins, origin) {
					face.Origins = append(face.Origins, origin)
				}
			}
		}
	}
	low := faces[fmt.Sprintf("side(0,%d)", candidate.Meridian.LowCapSegment)]
	high := faces[fmt.Sprintf("side(0,%d)", candidate.Meridian.HighCapSegment)]
	if low == nil || high == nil || low == high {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore target has no distinct end faces", decaderr.ErrUnsupported)
	}
	if _, ok := low.Surface.(surfacegeom.Plane); !ok {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore lower end is not planar", decaderr.ErrUnsupported)
	}
	if _, ok := high.Surface.(surfacegeom.Plane); !ok {
		return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore upper end is not planar", decaderr.ErrUnsupported)
	}
	tool := candidate.OriginalTool
	hole, err := offset2d.ReverseLoopRecordContext(ctx, tool.Profile.Outer)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	normal := tool.Dir(0, 0, 1)
	lowLevel := env.Doc.MintLevel(tool.Point(0, 0, tool.Z0), normal)
	highLevel := env.Doc.MintLevel(tool.Point(0, 0, tool.Z1), normal)
	walls, bottom, top, perimeter, err := prismrecord.BuildLoopSidesAs(ctx, env, body, tool, 1, true,
		hole, freeform.NewFreeformWork(), nil, lowLevel, highLevel, true)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	for _, face := range walls {
		for _, origin := range slices.Clone(face.Origins) {
			var segment int
			if _, err := fmt.Sscanf(origin.Role, "side(1,%d)", &segment); err != nil || segment < 0 || segment >= len(hole.Segments) {
				return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore wall has no source role", decaderr.ErrUnsupported)
			}
			sourceRole := fmt.Sprintf("side(0,%d)", len(hole.Segments)-1-segment)
			refs := candidate.ToolOrigins[sourceRole]
			if len(refs) == 0 {
				return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore wall has no source face", decaderr.ErrUnsupported)
			}
			for _, ref := range refs {
				if !slices.Contains(face.Origins, ref) {
					face.Origins = append(face.Origins, ref)
				}
			}
		}
	}
	// The wall's lower use walks forward and its upper use backward.
	// Each cap must use the shared rim in the opposite direction.
	for i, j := 0, len(bottom)-1; i < j; i, j = i+1, j-1 {
		bottom[i], bottom[j] = bottom[j], bottom[i]
	}
	for i := range bottom {
		bottom[i].Forward = !bottom[i].Forward
	}
	sewn := make(map[*topo.Edge]struct{}, len(bottom)+len(top))
	if candidate.ToolReversed {
		low, high = high, low
	}
	for _, end := range []struct {
		face *topo.Face
		rim  []topo.CoEdge
	}{{low, bottom}, {high, top}} {
		end.face.Loops = append(end.face.Loops, topobuild.NewLoop(env.Alloc, topo.Loop{CoEdges: end.rim}))
		for _, use := range end.rim {
			use.Edge.Faces = append(use.Edge.Faces, end.face)
			sewn[use.Edge] = struct{}{}
		}
	}
	shell := body.Lumps[0].Shells[0]
	shell.Faces = append(shell.Faces, walls...)
	if err := compositesweep.AuditBoundary(ctx, body, sewn); err != nil {
		return proofbound.BoundedScalar{}, err
	}
	for _, edge := range topobuild.Edges(body) {
		if len(edge.Faces) != 2 {
			return proofbound.BoundedScalar{}, fmt.Errorf("%w: bore boundary is not closed", decaderr.ErrUnsupported)
		}
	}
	shell.Open = false
	return perimeter, nil
}

func resultFacesByRole(body *topo.Body) map[string]*topo.Face {
	faces := make(map[string]*topo.Face)
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				faces[origin.Role] = face
			}
		}
	}
	return faces
}
