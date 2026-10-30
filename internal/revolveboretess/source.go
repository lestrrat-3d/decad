package revolveboretess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolvetess"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Sources carries the two closed source meshes and their shared-plane rims.
// Its faces are temporary and must be remapped before publishing a mesh.
type Sources struct {
	Host, Tool                     *topo.Mesh
	HostCaps, ToolCaps             [2]*topo.Face
	HostRims, ToolRims             [2][]int
	HostProjection, ToolProjection float64
	ProjectionAxis                 int
}

// BuildSources meshes both source records and bounds their cap-plane projections.
func BuildSources(ctx context.Context, body *topo.Body, payload revolvebore.Payload, chord float64) (Sources, error) {
	var result Sources
	c := payload.Candidate
	env := topo.Env{Doc: &topo.Document{}, Ref: body.Origin.Producer, Alloc: scratchAlloc{}}
	host, err := revolveplan.Build(ctx, env, c.Revolve, freeform.NewFreeformWork())
	if err != nil {
		return result, err
	}
	tool, err := prismrecord.Build(ctx, env, c.OriginalTool, freeform.NewFreeformWork())
	if err != nil {
		return result, err
	}
	result.Host, err = revolvetess.TessellateRevolveBoundedAudit(ctx, host, c.Revolve, chord)
	if err != nil {
		return result, err
	}
	result.Tool, err = prismtess.Build(ctx, tool, c.OriginalTool,
		func(loop, seg int) string { return fmt.Sprintf("side(%d,%d)", loop, seg) }, chord, true, nil, 0)
	if err != nil {
		return result, err
	}
	hostFaces, toolFaces := topobuild.FacesByRole(host), topobuild.FacesByRole(tool)
	result.HostCaps = [2]*topo.Face{hostFaces[fmt.Sprintf("side(0,%d)", c.Meridian.LowCapSegment)],
		hostFaces[fmt.Sprintf("side(0,%d)", c.Meridian.HighCapSegment)]}
	result.ToolCaps = [2]*topo.Face{toolFaces["capStart"], toolFaces["capEnd"]}
	if c.ToolReversed {
		result.ToolCaps[0], result.ToolCaps[1] = result.ToolCaps[1], result.ToolCaps[0]
	}
	for i, level := range [2]float64{c.Meridian.Low, c.Meridian.High} {
		if result.HostCaps[i] == nil || result.ToolCaps[i] == nil {
			return Sources{}, fmt.Errorf("%w: bore source cap is missing", decaderr.ErrUnsupported)
		}
		result.HostRims[i], err = capRim(result.Host, result.HostCaps[i])
		if err != nil {
			return Sources{}, err
		}
		result.ToolRims[i], err = capRim(result.Tool, result.ToolCaps[i])
		if err != nil {
			return Sources{}, err
		}
		hostProjection, axis, ok := capProjectionBound(c.Tool, level, capPoints(result.Host, result.HostCaps[i]))
		if !ok {
			return Sources{}, fmt.Errorf("%w: bore host cap projection is not bounded", decaderr.ErrUnsupported)
		}
		toolProjection, toolAxis, ok := capProjectionBound(c.Tool, level, capPoints(result.Tool, result.ToolCaps[i]))
		if !ok {
			return Sources{}, fmt.Errorf("%w: bore tool cap projection is not bounded", decaderr.ErrUnsupported)
		}
		if axis != toolAxis || i > 0 && axis != result.ProjectionAxis {
			return Sources{}, fmt.Errorf("%w: bore cap projection axes differ", decaderr.ErrUnsupported)
		}
		result.ProjectionAxis = axis
		result.HostProjection = math.Max(result.HostProjection, hostProjection)
		result.ToolProjection = math.Max(result.ToolProjection, toolProjection)
	}
	return result, nil
}

func capPoints(mesh *topo.Mesh, face *topo.Face) []r3.Vec {
	var points []r3.Vec
	for i, tri := range mesh.Triangles {
		if mesh.Source[i] != face {
			continue
		}
		for _, vertex := range tri {
			points = append(points, mesh.Vertices[vertex])
		}
	}
	return points
}

// scratchAlloc allocates temporary source nodes that never reach a public handle.
type scratchAlloc struct{}

func (scratchAlloc) NewBody() *topo.Body           { return &topo.Body{} }
func (scratchAlloc) NewLump() *topo.Lump           { return &topo.Lump{} }
func (scratchAlloc) NewFaceShell() *topo.FaceShell { return &topo.FaceShell{} }
func (scratchAlloc) NewFace() *topo.Face           { return &topo.Face{} }
func (scratchAlloc) NewLoop() *topo.Loop           { return &topo.Loop{} }
func (scratchAlloc) NewEdge() *topo.Edge           { return &topo.Edge{} }
func (scratchAlloc) NewVertex() *topo.Vertex       { return &topo.Vertex{} }
