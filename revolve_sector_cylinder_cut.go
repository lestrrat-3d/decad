package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryRevolveSectorCylinderCut handles only a complete coaxial cylinder across
// an authenticated Revolve/point-Loft join. Other booleans keep their usual
// dispatch. This route runs before the general faceted contact-tolerance gate.
func tryRevolveSectorCylinderCut(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.RevolveLoftJoin == nil {
		return nil, false, nil
	}
	caps, ok := revolveshell.RevolveSectorBoreCapRecord(joined)
	if !ok {
		return nil, true, fmt.Errorf("%w: joined section has no complete planar cap-ring record", ErrUnsupported)
	}
	prism, ok := tool.n.Payload.(prismPayload)
	if !ok || prism.SurfaceResult || prism.Xform != r3.Identity() ||
		prism.SectionDelta != 0 || prism.Z0Delta != 0 || prism.Z1Delta != 0 ||
		prism.Frame.U() != (r3.Vec{Y: 1}) || prism.Frame.V() != (r3.Vec{Z: 1}) ||
		prism.Frame.N() != (r3.Vec{X: 1}) ||
		prism.Frame.Origin().Y != 0 || prism.Frame.Origin().Z != 0 ||
		len(prism.Profile.Holes) != 0 || len(prism.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := prism.Profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (planePoint{}) ||
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
		return nil, true, fmt.Errorf("%w: bore has no finite positive radius", ErrUnsupported)
	}
	start := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.Z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.X), proofarith.FloatRat(prism.Z1))
	if start.Cmp(proofarith.FloatRat(caps.LowX)) >= 0 ||
		end.Cmp(proofarith.FloatRat(caps.HighX)) <= 0 {
		return nil, true, fmt.Errorf("%w: cylinder does not span both section caps", ErrUnsupported)
	}
	var wall *Face
	for _, face := range tool.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role != prismrecord.OuterFirstSideRole {
				continue
			}
			if wall != nil {
				return nil, true, fmt.Errorf("%w: cylinder wall is ambiguous", ErrUnsupported)
			}
			if _, ok := face.n.Surface.(surfacegeom.Cylinder); !ok {
				return nil, true, fmt.Errorf("%w: tool has no cylindrical side", ErrUnsupported)
			}
			wall = face
		}
	}
	if wall == nil {
		return nil, true, fmt.Errorf("%w: cylinder wall is missing", ErrUnsupported)
	}
	if radius < min(caps.LowRadius, caps.HighRadius) {
		return buildRevolveSectorAnnularCut(ctx, d, ref, joined, caps, radius, wall)
	}
	if radius > caps.LowRadius && radius < caps.HighRadius {
		wallGroup := facetedrecord.Group{Origins: wall.originRefs(),
			Surface: wall.n.Surface, NormalBound: wall.n.NormalBound, Reversed: !wall.n.Reversed}
		result, err := revolveshell.BuildSectorBandCut(ctx, joined, caps, radius, wallGroup)
		if err != nil {
			return nil, true, err
		}
		body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), result))
		return body, true, err
	}
	return nil, true, fmt.Errorf("%w: bore has no certified section crossing", ErrUnsupported)
}

func buildRevolveSectorAnnularCut(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, caps revolveshell.SectorBoreCaps, radius float64, wall *Face) (*Body, bool, error) {
	ring := joined.RevolveLoftJoin.CapRing
	n := len(ring.Angles)
	if len(joined.Tris) > revolveshell.SectorBoreMaxAuditTriangles-4*n ||
		radius >= min(caps.LowRadius, caps.HighRadius) {
		return nil, true, fmt.Errorf("%w: bore does not fit inside both radial caps", ErrUnsupported)
	}
	maxStep, ok := revolvemesh.SectorRingMaxStep(ring.Angles)
	if !ok {
		return nil, true, fmt.Errorf("%w: joined section ring has no bounded station plan", ErrUnsupported)
	}
	inner, boreBound, err := circularbounds.BoreRingAtStep(ctx, radius, ring.Angles, maxStep)
	if err != nil {
		return nil, true, err
	}
	if proofbound.IsNonFinite(boreBound) || boreBound >= radius ||
		max(joined.MeshBound, boreBound) > 0.1 ||
		proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound) >=
			min(caps.LowRadius, caps.HighRadius) {
		return nil, true, fmt.Errorf("%w: bore has no separate cap ring", ErrUnsupported)
	}
	clearance := proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound)
	section := joined.RevolveLoftJoin.Certificate.Section
	for i, span := range section.Spans {
		if i == section.AxisSpan || i == caps.Low || i == caps.High {
			continue
		}
		if min(span.Start.V, span.End.V) <= clearance {
			return nil, true, fmt.Errorf("%w: section meridian may enter the bore", ErrUnsupported)
		}
	}
	lowPos, highPos := make(map[int]int, n), make(map[int]int, n)
	used := map[int]bool{caps.LowAxis: true, caps.HighAxis: true}
	for i := range n {
		for _, v := range [...]int{caps.LowOuter[i], caps.HighOuter[i]} {
			if v < 0 || v >= len(joined.Verts) || used[v] {
				return nil, true, fmt.Errorf("%w: cap ring repeats a vertex", ErrUnsupported)
			}
			used[v] = true
		}
		lowPos[caps.LowOuter[i]], highPos[caps.HighOuter[i]] = i, i
	}
	lowSeen, highSeen := make([]bool, n), make([]bool, n)
	mesh := &joinedMesh{Verts: append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetGroup(nil), joined.Groups...)}
	for i, tri := range joined.Tris {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		cell, fan, err := revolvemesh.SectorCapFan(tri, joined.Src[i], caps.LowAxis,
			caps.LowGroup, lowPos, n)
		if err != nil {
			return nil, true, err
		}
		if fan {
			if lowSeen[cell] {
				return nil, true, fmt.Errorf("%w: low cap repeats a cell", ErrUnsupported)
			}
			lowSeen[cell] = true
			continue
		}
		cell, fan, err = revolvemesh.SectorCapFan(tri, joined.Src[i], caps.HighAxis,
			caps.HighGroup, highPos, n)
		if err != nil {
			return nil, true, err
		}
		if fan {
			if highSeen[cell] {
				return nil, true, fmt.Errorf("%w: high cap repeats a cell", ErrUnsupported)
			}
			highSeen[cell] = true
			continue
		}
		if !revolveshell.RevolveSectorTriangleClear(tri, joined.Verts, clearance) {
			return nil, true, fmt.Errorf("%w: untouched face may enter the bore", ErrUnsupported)
		}
		mesh.Add(tri, joined.Src[i])
	}
	for i := range n {
		if !lowSeen[i] || !highSeen[i] {
			return nil, true, fmt.Errorf("%w: cap fan misses an angular cell", ErrUnsupported)
		}
	}
	wallGroup := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, facetGroup{Origins: wall.originRefs(),
		Surface: wall.n.Surface, NormalBound: wall.n.NormalBound, Reversed: !wall.n.Reversed})
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
		mesh.Add([3]int{innerLow[i], innerHigh[i], innerHigh[j]}, wallGroup)
		mesh.Add([3]int{innerLow[i], innerHigh[j], innerLow[j]}, wallGroup)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, true, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, true, err
	}
	if len(mesh.Tris) != len(joined.Tris)+4*n {
		return nil, true, fmt.Errorf("%w: cap and wall triangle census changed", ErrUnsupported)
	}
	patches := revolvemesh.SectorBorePatches(mesh.Src, oldCount, n, false, caps.LowGroup, caps.HighGroup, -1, wallGroup)
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, patches, revolveshell.SectorBoreMaxAuditTriangles,
		revolveshell.SectorBoreMaxAuditScans); err != nil {
		return nil, true, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(caps.HighX), proofarith.FloatRat(caps.LowX)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return nil, true, fmt.Errorf("%w: bore has no positive axial span", ErrUnsupported)
	}
	shellArea := proofbound.ProductUpper(
		proofbound.ProductUpper(2, proofbound.TwoPiUpper()),
		proofbound.ProductUpper(radius, boreBound))
	volumeGap := proofbound.AbsSumUpper(joined.VolSymDiff,
		proofbound.ProductUpper(shellArea, height))
	bound := max(joined.MeshBound, boreBound)
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, true, err
	}
	oldArea, err := bodyHandle(joined.RevolveLoftJoin.Blank).Area()
	if err != nil {
		return nil, true, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, heldArea,
		oldArea.Value.Base(), oldArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, true, fmt.Errorf("%w: bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.RevolveLoftJoin = nil
	result.ContactAudited = true
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), result))
	return body, true, err
}
