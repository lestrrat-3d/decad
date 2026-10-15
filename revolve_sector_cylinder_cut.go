package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const (
	revolveSectorBoreMaxAuditTriangles = 1_321_696
	revolveSectorBoreMaxAuditScans     = uint64(400_000_000)
)

type revolveSectorBoreCaps struct {
	low, high             int
	lowAxis, highAxis     int
	lowOuter, highOuter   []int
	lowGroup, highGroup   int
	lowX, highX           float64
	lowRadius, highRadius float64
}

// revolveSectorBoreCaps derives the cap faces and ring IDs from the retained
// meridian walk. The two neighbors of its sole axial span must be complete
// planar radial caps, regardless of the number of intervening section spans.
func revolveSectorBoreCapRecord(joined facetedPayload) (revolveSectorBoreCaps, bool) {
	var caps revolveSectorBoreCaps
	record := joined.RevolveLoftJoin
	if record == nil || record.Certificate == nil || record.Blank == nil ||
		record.CapRing == nil || record.Certificate.Section.Blank != record.Blank ||
		record.BoundaryBound != joined.MeshBound ||
		record.VolumeBound != joined.VolSymDiff || record.AreaBound != joined.AreaSlack ||
		joined.Xform != r3.Identity() || !joined.ContactAudited {
		return caps, false
	}
	section, ring := record.Certificate.Section, record.CapRing
	spans := section.Spans
	count, n := len(spans), len(ring.Angles)
	if count < 4 || len(ring.Rings) != count || len(ring.Groups) != count ||
		section.AxisSpan < 0 || section.AxisSpan >= count ||
		n < 16 || n > revolvemesh.SectorBoreMaxStations ||
		len(joined.Tris) != len(joined.Src) ||
		len(joined.Verts) != len(joined.VertexBound) {
		return caps, false
	}
	axis := section.AxisSpan
	caps.low, caps.high = (axis+count-1)%count, (axis+1)%count
	low, high := spans[caps.low], spans[caps.high]
	if spans[axis].Start.V != 0 || spans[axis].End.V != 0 ||
		low.End != spans[axis].Start || high.Start != spans[axis].End ||
		low.Start.V <= 0 || high.End.V <= 0 ||
		low.Start.U != low.End.U || high.Start.U != high.End.U ||
		low.Start.U >= high.Start.U ||
		low.Face == nil || high.Face == nil ||
		len(low.Origins) == 0 || len(high.Origins) == 0 {
		return caps, false
	}
	if _, ok := low.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, false
	}
	if _, ok := high.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, false
	}
	caps.lowX, caps.highX = low.Start.U, high.Start.U
	caps.lowRadius, caps.highRadius = low.Start.V, high.End.V
	caps.lowGroup, caps.highGroup = ring.Groups[caps.low], ring.Groups[caps.high]
	if caps.lowGroup < 0 || caps.highGroup < 0 ||
		caps.lowGroup >= len(joined.Groups) || caps.highGroup >= len(joined.Groups) ||
		caps.lowGroup == caps.highGroup ||
		len(ring.Rings[axis]) != 1 || len(ring.Rings[caps.high]) != 1 ||
		len(ring.Rings[caps.low]) != n ||
		len(ring.Rings[(caps.high+1)%count]) != n {
		return caps, false
	}
	caps.lowAxis, caps.highAxis = ring.Vertex(axis, 0), ring.Vertex(caps.high, 0)
	caps.lowOuter = ring.Rings[caps.low]
	caps.highOuter = ring.Rings[(caps.high+1)%count]
	if caps.lowAxis == caps.highAxis ||
		caps.lowAxis < 0 || caps.highAxis < 0 ||
		caps.lowAxis >= len(joined.Verts) || caps.highAxis >= len(joined.Verts) ||
		joined.Verts[caps.lowAxis] != (r3.Vec{X: caps.lowX}) ||
		joined.Verts[caps.highAxis] != (r3.Vec{X: caps.highX}) {
		return caps, false
	}
	return caps, true
}

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
	caps, ok := revolveSectorBoreCapRecord(joined)
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
	if start.Cmp(proofarith.FloatRat(caps.lowX)) >= 0 ||
		end.Cmp(proofarith.FloatRat(caps.highX)) <= 0 {
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
	if radius < min(caps.lowRadius, caps.highRadius) {
		return buildRevolveSectorAnnularCut(ctx, d, ref, joined, caps, radius, wall)
	}
	if radius > caps.lowRadius && radius < caps.highRadius {
		return buildRevolveSectorBandCut(ctx, d, ref, joined, caps, radius, wall)
	}
	return nil, true, fmt.Errorf("%w: bore has no certified section crossing", ErrUnsupported)
}

func revolveSectorTriangleClear(t [3]int, verts []r3.Vec, clearance float64) bool {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	loY, hiY := min(a.Y, b.Y, c.Y), max(a.Y, b.Y, c.Y)
	loZ, hiZ := min(a.Z, b.Z, c.Z), max(a.Z, b.Z, c.Z)
	if loY > clearance || hiY < -clearance || loZ > clearance || hiZ < -clearance {
		return true
	}
	projected := [3]r3.Vec{{X: a.Y, Y: a.Z}, {X: b.Y, Y: b.Z}, {X: c.Y, Y: c.Z}}
	return loftcylinder.TriangleMinRadius(projected[:], [3]int{0, 1, 2}) > clearance
}

func buildRevolveSectorAnnularCut(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, caps revolveSectorBoreCaps, radius float64, wall *Face) (*Body, bool, error) {
	ring := joined.RevolveLoftJoin.CapRing
	n := len(ring.Angles)
	if len(joined.Tris) > revolveSectorBoreMaxAuditTriangles-4*n ||
		radius >= min(caps.lowRadius, caps.highRadius) {
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
			min(caps.lowRadius, caps.highRadius) {
		return nil, true, fmt.Errorf("%w: bore has no separate cap ring", ErrUnsupported)
	}
	clearance := proofbound.AbsSumUpper(radius, joined.MeshBound, boreBound)
	section := joined.RevolveLoftJoin.Certificate.Section
	for i, span := range section.Spans {
		if i == section.AxisSpan || i == caps.low || i == caps.high {
			continue
		}
		if min(span.Start.V, span.End.V) <= clearance {
			return nil, true, fmt.Errorf("%w: section meridian may enter the bore", ErrUnsupported)
		}
	}
	lowPos, highPos := make(map[int]int, n), make(map[int]int, n)
	used := map[int]bool{caps.lowAxis: true, caps.highAxis: true}
	for i := range n {
		for _, v := range [...]int{caps.lowOuter[i], caps.highOuter[i]} {
			if v < 0 || v >= len(joined.Verts) || used[v] {
				return nil, true, fmt.Errorf("%w: cap ring repeats a vertex", ErrUnsupported)
			}
			used[v] = true
		}
		lowPos[caps.lowOuter[i]], highPos[caps.highOuter[i]] = i, i
	}
	lowSeen, highSeen := make([]bool, n), make([]bool, n)
	mesh := &joinedMesh{Verts: append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetGroup(nil), joined.Groups...)}
	for i, tri := range joined.Tris {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		cell, fan, err := revolvemesh.SectorCapFan(tri, joined.Src[i], caps.lowAxis,
			caps.lowGroup, lowPos, n)
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
		cell, fan, err = revolvemesh.SectorCapFan(tri, joined.Src[i], caps.highAxis,
			caps.highGroup, highPos, n)
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
		if !revolveSectorTriangleClear(tri, joined.Verts, clearance) {
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
		low := r3.Vec{X: caps.lowX, Y: inner[i].U, Z: inner[i].V}
		high := r3.Vec{X: caps.highX, Y: inner[i].U, Z: inner[i].V}
		if i == 0 {
			innerLow[i], innerHigh[i] = caps.lowAxis, caps.highAxis
			mesh.Verts[caps.lowAxis], mesh.Verts[caps.highAxis] = low, high
			vertexBound[caps.lowAxis], vertexBound[caps.highAxis] = boreBound, boreBound
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
		mesh.Add([3]int{caps.lowOuter[i], innerLow[i], innerLow[j]}, caps.lowGroup)
		mesh.Add([3]int{caps.lowOuter[i], innerLow[j], caps.lowOuter[j]}, caps.lowGroup)
		mesh.Add([3]int{innerHigh[i], caps.highOuter[i], caps.highOuter[j]}, caps.highGroup)
		mesh.Add([3]int{innerHigh[i], caps.highOuter[j], innerHigh[j]}, caps.highGroup)
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
	patches := revolvemesh.SectorBorePatches(mesh.Src, oldCount, n, false, caps.lowGroup, caps.highGroup, -1, wallGroup)
	if err := loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, patches, revolveSectorBoreMaxAuditTriangles,
		revolveSectorBoreMaxAuditScans); err != nil {
		return nil, true, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(caps.highX), proofarith.FloatRat(caps.lowX)))
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
	body, err := buildFacetedBody(ctx, d, ref, result)
	return body, true, err
}
