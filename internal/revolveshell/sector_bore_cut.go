package revolveshell

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SectorCylinderCut handles only a complete coaxial cylinder across an
// authenticated Revolve/point-Loft join and builds the result in env. It
// reports false, with no error, for an operation other than Cut, a target
// with no join record, with a certificate about a general axis or with a
// tooth trimmed about an axis other than world X, or a tool that is not an unplaced axis-X prism of one whole
// origin-centered circle; other booleans then keep their usual dispatch. A joined target without a complete planar cap-ring record
// refuses before the tool is read, and so does any later failed check. This
// route runs before the general faceted contact-tolerance gate.
func SectorCylinderCut(ctx context.Context, op meshbool.OperationKind, env topo.Env,
	target, tool *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.Payload.(facetedrecord.Payload)
	if !ok || joined.RevolveLoftJoin == nil || sectorJoinGeneralAxis(joined.RevolveLoftJoin) {
		return nil, false, nil
	}
	caps, ok := RevolveSectorBoreCapRecord(joined)
	if !ok {
		return nil, true, fmt.Errorf("%w: joined section has no complete planar cap-ring record", decaderr.ErrUnsupported)
	}
	prism, ok := tool.Payload.(prismrecord.Payload)
	if !ok || prism.SurfaceResult || prism.Xform != r3.Identity() ||
		prism.SectionDelta != 0 || prism.Z0Delta != 0 || prism.Z1Delta != 0 ||
		prism.Frame.U() != (r3.Vec{Y: 1}) || prism.Frame.V() != (r3.Vec{Z: 1}) ||
		prism.Frame.N() != (r3.Vec{X: 1}) ||
		prism.Frame.Origin().Y != 0 || prism.Frame.Origin().Z != 0 ||
		len(prism.Profile.Holes) != 0 || len(prism.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := prism.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center != (sectionrecord.Point2{}) ||
		!((circle.TStart == 0 && circle.TEnd == 1) ||
			(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return nil, true, err
	}
	origin := prism.Frame.Origin()
	if !proofbound.FiniteVec(origin) ||
		!proofbound.FiniteVec(r3.Vec{X: prism.Z0, Y: prism.Z1, Z: radius}) || radius <= 0 {
		return nil, true, fmt.Errorf("%w: bore has no finite positive radius", decaderr.ErrUnsupported)
	}
	start := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.Z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.Z1))
	if start.Cmp(proofarith.FloatRat(caps.LowX)) >= 0 ||
		end.Cmp(proofarith.FloatRat(caps.HighX)) <= 0 {
		return nil, true, fmt.Errorf("%w: cylinder does not span both section caps", decaderr.ErrUnsupported)
	}
	var wall *topo.Face
	for _, face := range topobuild.Faces(tool) {
		for _, origin := range face.Origins {
			if origin.Role != prismrecord.OuterFirstSideRole {
				continue
			}
			if wall != nil {
				return nil, true, fmt.Errorf("%w: cylinder wall is ambiguous", decaderr.ErrUnsupported)
			}
			if _, ok := face.Surface.(surfacegeom.Cylinder); !ok {
				return nil, true, fmt.Errorf("%w: tool has no cylindrical side", decaderr.ErrUnsupported)
			}
			wall = face
		}
	}
	if wall == nil {
		return nil, true, fmt.Errorf("%w: cylinder wall is missing", decaderr.ErrUnsupported)
	}
	wallGroup := facetedrecord.Group{Origins: append([]provenance.Ref(nil), wall.Origins...),
		Surface: wall.Surface, NormalBound: wall.NormalBound, Reversed: !wall.Reversed}
	if radius < min(caps.LowRadius, caps.HighRadius) {
		result, err := buildSectorAnnularCut(ctx, joined, caps, radius, wallGroup)
		if err != nil {
			return nil, true, err
		}
		body, err := facetedrecord.Build(ctx, env, result)
		return body, true, err
	}
	if radius > caps.LowRadius && radius < caps.HighRadius {
		result, err := BuildSectorBandCut(ctx, joined, caps, radius, wallGroup)
		if err != nil {
			return nil, true, err
		}
		body, err := facetedrecord.Build(ctx, env, result)
		return body, true, err
	}
	return nil, true, fmt.Errorf("%w: bore has no certified section crossing", decaderr.ErrUnsupported)
}

// buildSectorAnnularCut replaces both complete axis cap fans of joined with
// annuli around a bore of radius strictly inside both cap outer radii, and
// adds the bore's wall as wallGroup.
func buildSectorAnnularCut(ctx context.Context, joined facetedrecord.Payload, caps SectorBoreCaps,
	radius float64, wallGroup facetedrecord.Group) (facetedrecord.Payload, error) {
	var empty facetedrecord.Payload
	ring := joined.RevolveLoftJoin.CapRing
	n := len(ring.Angles)
	if len(joined.Tris) > SectorBoreMaxAuditTriangles-4*n ||
		radius >= min(caps.LowRadius, caps.HighRadius) {
		return empty, fmt.Errorf("%w: bore does not fit inside both radial caps", decaderr.ErrUnsupported)
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(ring.Angles)
	if !ok {
		return empty, fmt.Errorf("%w: joined section ring has no bounded station plan", decaderr.ErrUnsupported)
	}
	inner, boreBound, err := circularbounds.BoreRingAtStep(ctx, radius, ring.Angles, maxStep)
	if err != nil {
		return empty, err
	}
	if proofbound.IsNonFinite(boreBound) || boreBound >= radius ||
		max(joined.MeshBound, boreBound) > 0.1 ||
		proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound) >=
			min(caps.LowRadius, caps.HighRadius) {
		return empty, fmt.Errorf("%w: bore has no separate cap ring", decaderr.ErrUnsupported)
	}
	clearance := proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound)
	section := joined.RevolveLoftJoin.Certificate.Section
	for i, span := range section.Spans {
		if i == section.AxisSpan || i == caps.Low || i == caps.High {
			continue
		}
		if min(span.Start.V, span.End.V) <= clearance {
			return empty, fmt.Errorf("%w: section meridian may enter the bore", decaderr.ErrUnsupported)
		}
	}
	lowPos, highPos := make(map[int]int, n), make(map[int]int, n)
	used := map[int]bool{caps.LowAxis: true, caps.HighAxis: true}
	for i := range n {
		for _, v := range [...]int{caps.LowOuter[i], caps.HighOuter[i]} {
			if v < 0 || v >= len(joined.Verts) || used[v] {
				return empty, fmt.Errorf("%w: cap ring repeats a vertex", decaderr.ErrUnsupported)
			}
			used[v] = true
		}
		lowPos[caps.LowOuter[i]], highPos[caps.HighOuter[i]] = i, i
	}
	lowSeen, highSeen := make([]bool, n), make([]bool, n)
	mesh := &facetedrecord.JoinedMesh{Verts: append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetedrecord.Group(nil), joined.Groups...)}
	for i, tri := range joined.Tris {
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		cell, fan, err := revolvemesh.SectorCapFan(tri, joined.Src[i], caps.LowAxis,
			caps.LowGroup, lowPos, n)
		if err != nil {
			return empty, err
		}
		if fan {
			if lowSeen[cell] {
				return empty, fmt.Errorf("%w: low cap repeats a cell", decaderr.ErrUnsupported)
			}
			lowSeen[cell] = true
			continue
		}
		cell, fan, err = revolvemesh.SectorCapFan(tri, joined.Src[i], caps.HighAxis,
			caps.HighGroup, highPos, n)
		if err != nil {
			return empty, err
		}
		if fan {
			if highSeen[cell] {
				return empty, fmt.Errorf("%w: high cap repeats a cell", decaderr.ErrUnsupported)
			}
			highSeen[cell] = true
			continue
		}
		if !RevolveSectorTriangleClear(tri, joined.Verts, clearance) {
			return empty, fmt.Errorf("%w: untouched face may enter the bore", decaderr.ErrUnsupported)
		}
		mesh.Add(tri, joined.Src[i])
	}
	for i := range n {
		if !lowSeen[i] || !highSeen[i] {
			return empty, fmt.Errorf("%w: cap fan misses an angular cell", decaderr.ErrUnsupported)
		}
	}
	wallIndex := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, wallGroup)
	vertexBound := append([]float64(nil), joined.VertexBound...)
	innerLow, innerHigh := make([]int, n), make([]int, n)
	for i := range n {
		low := r3.Vec{X: caps.LowX, Y: inner[i].U, Z: inner[i].V}
		high := r3.Vec{X: caps.HighX, Y: inner[i].U, Z: inner[i].V}
		if i == 0 {
			innerLow[i], innerHigh[i] = caps.LowAxis, caps.HighAxis
			mesh.Verts[caps.LowAxis], mesh.Verts[caps.HighAxis] = low, high
			vertexBound[caps.LowAxis], vertexBound[caps.HighAxis] = boreBound, boreBound
			continue
		}
		innerLow[i] = len(mesh.Verts)
		mesh.Verts = append(mesh.Verts, low)
		vertexBound = append(vertexBound, boreBound)
		innerHigh[i] = len(mesh.Verts)
		mesh.Verts = append(mesh.Verts, high)
		vertexBound = append(vertexBound, boreBound)
	}
	oldCount := len(mesh.Tris)
	for i := range n {
		j := (i + 1) % n
		mesh.Add([3]int{caps.LowOuter[i], innerLow[i], innerLow[j]}, caps.LowGroup)
		mesh.Add([3]int{caps.LowOuter[i], innerLow[j], caps.LowOuter[j]}, caps.LowGroup)
		mesh.Add([3]int{innerHigh[i], caps.HighOuter[i], caps.HighOuter[j]}, caps.HighGroup)
		mesh.Add([3]int{innerHigh[i], caps.HighOuter[j], innerHigh[j]}, caps.HighGroup)
		mesh.Add([3]int{innerLow[i], innerHigh[i], innerHigh[j]}, wallIndex)
		mesh.Add([3]int{innerLow[i], innerHigh[j], innerLow[j]}, wallIndex)
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return empty, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return empty, err
	}
	if len(mesh.Tris) != len(joined.Tris)+4*n {
		return empty, fmt.Errorf("%w: cap and wall triangle census changed", decaderr.ErrUnsupported)
	}
	patches := revolvemesh.SectorBorePatches(mesh.Src, oldCount, n, false, caps.LowGroup, caps.HighGroup, -1, wallIndex)
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, patches, SectorBoreMaxAuditTriangles,
		SectorBoreMaxAuditScans); err != nil {
		return empty, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(caps.HighX), proofarith.FloatRat(caps.LowX)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return empty, fmt.Errorf("%w: bore has no positive axial span", decaderr.ErrUnsupported)
	}
	shellArea := proofbound.ProductUpper(
		proofbound.ProductUpper(2, proofbound.TwoPiUpper()),
		proofbound.ProductUpper(radius, boreBound))
	volumeGap := proofbound.AbsSumUpper(joined.VolSymDiff,
		proofbound.ProductUpper(shellArea, height))
	bound := max(joined.MeshBound, boreBound)
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return empty, err
	}
	oldArea := joined.RevolveLoftJoin.Blank.Area
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, heldArea,
		oldArea.Value.Base(), oldArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return empty, fmt.Errorf("%w: bore has no finite measure proof", decaderr.ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.RevolveLoftJoin = nil
	result.ContactAudited = true
	return result, nil
}

// sectorJoinGeneralAxis reports whether the join's certificate came from the
// general record branch, or its tooth trim holds a limit about an axis other
// than world X. The bore route reads the joined record and its ring in +X
// arithmetic, so such a join keeps ordinary dispatch. A general certificate
// declines even about A = X, such as an XZ-plane blank revolved about U: its
// ring stands at angles in the blank's own basis, not at atan2(Z, Y)
// (agent-docs/revolve-axis-design.md §3.3, §4.6).
func sectorJoinGeneralAxis(record *facetedrecord.RevolveLoftJoinRecord) bool {
	if record.Certificate == nil {
		return false
	}
	if record.Certificate.Section.General != nil {
		return true
	}
	if record.Certificate.Trim == nil {
		return false
	}
	trim := record.Certificate.Trim
	for _, limit := range []*facetedrecord.PointConeLimit{trim.Lower, trim.Upper} {
		if limit != nil && !limit.Axis.Canonical() {
			return true
		}
	}
	return false
}
