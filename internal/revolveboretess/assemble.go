package revolveboretess

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Assemble joins two certified source meshes through their shared cap rims.
// For the virtual projected meshes, the monotone host meridian and strict
// whole-tool radial clearance keep their side surfaces apart. The admitted
// equal axis points and exact levels give the same two cap planes. Radial
// clearance reduced by both mesh and projection bounds keeps the bore inside
// each host cap. Loop clearance checks their separation, so each triangulated
// annulus is the regularized difference of the two projected caps. The
// source and result audits prove their vertex homotopies remain embedded;
// the projection allowances below bound the geometry moved along those paths.
func Assemble(ctx context.Context, body *topo.Body, p revolvebore.Payload, src Sources) (*topo.Mesh, error) {
	if !src.Host.SymDiffOK || !src.Tool.SymDiffOK {
		return nil, fmt.Errorf("%w: bore source meshes lack volume proofs", decaderr.ErrUnsupported)
	}
	radius := proofarith.FloatRat(p.Candidate.Meridian.MinRadius)
	hostBound, toolBound := proofarith.FloatRat(src.Host.Bound), proofarith.FloatRat(src.Tool.Bound)
	if radius == nil || hostBound == nil || toolBound == nil {
		return nil, fmt.Errorf("%w: bore source mesh clearance is not finite", decaderr.ErrUnsupported)
	}
	hostProjection, toolProjection := proofarith.FloatRat(src.HostProjection), proofarith.FloatRat(src.ToolProjection)
	if hostProjection == nil || toolProjection == nil {
		return nil, fmt.Errorf("%w: bore cap projection is not finite", decaderr.ErrUnsupported)
	}
	linear, err := massmoment.PlaneMap(p.Candidate.Tool.Frame, p.Candidate.Tool.Xform)
	if err != nil {
		return nil, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return nil, err
	}
	localRadius, ok := radiusAfterWorldAllowance(radius, charge.Stretch, hostBound, toolBound, hostProjection, toolProjection)
	if !ok {
		return nil, fmt.Errorf("%w: bore frame has no positive length lower bound", decaderr.ErrUnsupported)
	}
	if localRadius.Sign() <= 0 || !revolvebore.InsideRadius(p.Candidate.Tool.Profile.Outer, proofbound.RatFloatDown(localRadius)) {
		return nil, fmt.Errorf("%w: bore source meshes lack strict radial clearance", decaderr.ErrUnsupported)
	}
	if src.HostProjection > 0 {
		if err := auditMesh(ctx, src.Host, proofbound.AbsSumUpper(src.Host.CoordBound, src.HostProjection)); err != nil {
			return nil, err
		}
	}
	if src.ToolProjection > 0 {
		if err := auditMesh(ctx, src.Tool, proofbound.AbsSumUpper(src.Tool.CoordBound, src.ToolProjection)); err != nil {
			return nil, err
		}
	}
	byRole := make(map[string]*topo.Face)
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				byRole[origin.Role] = face
			}
		}
	}
	mesh := &topo.Mesh{Vertices: append(slices.Clone(src.Host.Vertices), src.Tool.Vertices...)}
	toolOffset := len(src.Host.Vertices)
	faceRole := func(face *topo.Face) string {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				return origin.Role
			}
		}
		return ""
	}
	for i, tri := range src.Host.Triangles {
		face := src.Host.Source[i]
		if face == src.HostCaps[0] || face == src.HostCaps[1] {
			continue
		}
		result := byRole[faceRole(face)]
		if result == nil {
			return nil, fmt.Errorf("%w: bore host face role is missing", decaderr.ErrUnsupported)
		}
		topobuild.AddTriangle(mesh, tri, result)
		topobuild.SetFaceBound(mesh, result, src.Host.FaceBound[face])
	}
	for i, tri := range src.Tool.Triangles {
		face := src.Tool.Source[i]
		if face == src.ToolCaps[0] || face == src.ToolCaps[1] {
			continue
		}
		var segment int
		if _, err := fmt.Sscanf(faceRole(face), "side(0,%d)", &segment); err != nil {
			return nil, fmt.Errorf("%w: bore tool face role is missing", decaderr.ErrUnsupported)
		}
		role := fmt.Sprintf("side(1,%d)", len(p.Candidate.OriginalTool.Profile.Outer.Segments)-1-segment)
		result := byRole[role]
		if result == nil {
			return nil, fmt.Errorf("%w: bore wall face role is missing", decaderr.ErrUnsupported)
		}
		tri = [3]int{tri[0] + toolOffset, tri[2] + toolOffset, tri[1] + toolOffset}
		topobuild.AddTriangle(mesh, tri, result)
		topobuild.SetFaceBound(mesh, result, src.Tool.FaceBound[face])
	}
	axis := src.ProjectionAxis
	for end := range 2 {
		var pts []sectionrecord.Point2
		var index []int
		loops := make([][]int, 2)
		for li, rim := range [][]int{src.HostRims[end], src.ToolRims[end]} {
			for _, vi := range rim {
				if li == 1 {
					vi += toolOffset
				}
				loops[li] = append(loops[li], len(pts))
				pts = append(pts, project(mesh.Vertices[vi], axis))
				index = append(index, vi)
			}
		}
		orientation := exactLoopSign(pts, loops[0])
		if orientation == 0 || exactLoopSign(pts, loops[1]) == 0 {
			return nil, fmt.Errorf("%w: bore cap projects to a degenerate loop", decaderr.ErrUnsupported)
		}
		if orientation < 0 {
			slices.Reverse(loops[0])
		}
		if exactLoopSign(pts, loops[1]) > 0 {
			slices.Reverse(loops[1])
		}
		if err := tessellation.RequireLoopClearance(ctx, pts, loops, []float64{
			proofbound.AbsSumUpper(src.Host.Bound, src.HostProjection),
			proofbound.AbsSumUpper(src.Tool.Bound, src.ToolProjection)}); err != nil {
			return nil, err
		}
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		if err != nil {
			return nil, err
		}
		capFace := byRole[fmt.Sprintf("side(0,%d)", []int{p.Candidate.Meridian.LowCapSegment, p.Candidate.Meridian.HighCapSegment}[end])]
		if capFace == nil {
			return nil, fmt.Errorf("%w: bore cap face role is missing", decaderr.ErrUnsupported)
		}
		for _, tri := range tris {
			if orientation < 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			topobuild.AddTriangle(mesh, [3]int{index[tri[0]], index[tri[1]], index[tri[2]]}, capFace)
		}
		topobuild.SetFaceBound(mesh, capFace, math.Max(
			proofbound.AbsSumUpper(src.Host.FaceBound[src.HostCaps[end]], src.HostProjection),
			proofbound.AbsSumUpper(src.Tool.FaceBound[src.ToolCaps[end]], src.ToolProjection)))
	}
	compactVertices(mesh)
	if err := tessellation.RequireClosedMesh(mesh.Triangles); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles); err != nil {
		return nil, err
	}
	mesh.CoordBound = math.Max(proofbound.AbsSumUpper(src.Host.CoordBound, src.HostProjection),
		proofbound.AbsSumUpper(src.Tool.CoordBound, src.ToolProjection))
	if err := auditMesh(ctx, mesh, mesh.CoordBound); err != nil {
		return nil, err
	}
	if tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, mesh.Vertices[0]) <= 0 {
		return nil, fmt.Errorf("%w: bore mesh orientation is not positive", decaderr.ErrUnsupported)
	}
	mesh.AreaSlack = proofbound.AbsSumUpper(src.Host.AreaSlack, src.Tool.AreaSlack,
		revolvemesh.CoordinateAreaAllow(mesh.Vertices, mesh.Triangles, mesh.CoordBound),
		revolvemesh.CoordinateAreaAllow(src.Host.Vertices, src.Host.Triangles, src.HostProjection),
		revolvemesh.CoordinateAreaAllow(src.Tool.Vertices, src.Tool.Triangles, src.ToolProjection))
	projection := math.Max(src.HostProjection, src.ToolProjection)
	mesh.VolSymDiff = proofbound.AbsSumUpper(src.Host.VolSymDiff, src.Tool.VolSymDiff,
		projectionVolume(src.Host, src.HostProjection), projectionVolume(src.Tool, src.ToolProjection),
		projectionVolume(mesh, projection))
	if proofbound.IsNonFinite(mesh.AreaSlack) || proofbound.IsNonFinite(mesh.VolSymDiff) {
		return nil, fmt.Errorf("%w: bore projection has no finite proof", decaderr.ErrUnsupported)
	}
	mesh.SymDiffOK = true
	return mesh, nil
}

// The map's Gram defect bounds its minimum length factor below by 1-stretch.
// Divide a world displacement by that factor before comparing local radii.
func radiusAfterWorldAllowance(radius *big.Rat, stretch float64, allowances ...*big.Rat) (*big.Rat, bool) {
	s := proofarith.FloatRat(stretch)
	if s == nil || s.Sign() < 0 || s.Cmp(big.NewRat(1, 1)) >= 0 {
		return nil, false
	}
	worldGap := new(big.Rat)
	for _, allow := range allowances {
		if allow == nil || allow.Sign() < 0 {
			return nil, false
		}
		worldGap.Add(worldGap, allow)
	}
	localGap := new(big.Rat).Quo(worldGap, new(big.Rat).Sub(big.NewRat(1, 1), s))
	return new(big.Rat).Sub(radius, localGap), true
}

func auditMesh(ctx context.Context, mesh *topo.Mesh, delta float64) error {
	budget := proofbound.NewWorkBudget(ctx)
	triangles, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles, delta)
	if err != nil {
		return err
	}
	return revolvemesh.RevolveContactAuditBounded(budget, triangles, mesh.Triangles, delta)
}

func projectionVolume(mesh *topo.Mesh, delta float64) float64 {
	if delta == 0 {
		return 0
	}
	return proofbound.SweptVolumeAllow(delta, proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, delta))
}

func project(v r3.Vec, axis int) sectionrecord.Point2 {
	switch axis {
	case 0:
		return sectionrecord.Point2{U: v.Y, V: v.Z}
	case 1:
		return sectionrecord.Point2{U: v.X, V: v.Z}
	default:
		return sectionrecord.Point2{U: v.X, V: v.Y}
	}
}

func exactLoopSign(pts []sectionrecord.Point2, loop []int) int {
	sum := new(big.Rat)
	for i, index := range loop {
		a, b := pts[index], pts[loop[(i+1)%len(loop)]]
		ax, ay, bx, by := proofarith.FloatRat(a.U), proofarith.FloatRat(a.V), proofarith.FloatRat(b.U), proofarith.FloatRat(b.V)
		if ax == nil || ay == nil || bx == nil || by == nil {
			return 0
		}
		sum.Add(sum, new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(ay, bx)))
	}
	return sum.Sign()
}

func compactVertices(mesh *topo.Mesh) {
	remap := make(map[int]int)
	vertices := make([]r3.Vec, 0, len(mesh.Vertices))
	for i, tri := range mesh.Triangles {
		for j, old := range tri {
			index, ok := remap[old]
			if !ok {
				index = len(vertices)
				remap[old] = index
				vertices = append(vertices, mesh.Vertices[old])
			}
			mesh.Triangles[i][j] = index
		}
	}
	mesh.Vertices = vertices
}
