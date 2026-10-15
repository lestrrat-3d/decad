package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftCylinderBoreRecord names the two certified annular caps of an unplaced
// through-bore result. Each outer and inner loop is CCW when viewed from +Z.
type loftCylinderBoreRecord struct {
	Blank *topo.Body
	// Teeth are exact Cut sources; ProofTeeth and AxialBridge retain the
	// private Union witnesses for later source-level admission checks.
	Teeth, ProofTeeth                   []*topo.Body
	AxialBridge                         []float64
	Outer, Inner                        [2][]int
	CapGroups                           [2]int
	WallGroup                           int
	BlankRadius, BoreRadius, BoreBound  float64
	InnerInset, OuterExcess, RootCollar float64
	Z0, Z1                              float64
}

// tryLoftCylinderBoreCut replaces the two planar cap disks of an authenticated
// loft/cylinder join with annuli and adds the bore's inward cylinder wall.
// The source join record makes the cap disks available without asking a new
// mesh boolean to resolve the earlier tooth/cylinder seams again.
func tryLoftCylinderBoreCut(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	if joined, ok := target.n.Payload.(facetedPayload); ok &&
		joined.LoftCylinderJoin != nil && joined.LoftCylinderJoin.Placement != nil {
		return tryPlacedLoftCylinderBoreCut(ctx, d, ref, target, tool)
	}
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.LoftCylinderJoin == nil || !joined.ContactAudited ||
		joined.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := joined.LoftCylinderJoin
	prism, ok := tool.n.Payload.(prismPayload)
	if !ok || prism.SurfaceResult || prism.Xform != r3.Identity() ||
		prism.SectionDelta != 0 || prism.Z0Delta != 0 || prism.Z1Delta != 0 ||
		prism.Frame.U() != (r3.Vec{X: 1}) || prism.Frame.V() != (r3.Vec{Y: 1}) ||
		prism.Frame.N() != (r3.Vec{Z: 1}) ||
		prism.Frame.Origin().X != 0 || prism.Frame.Origin().Y != 0 ||
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
		return nil, false, err
	}
	origin := prism.Frame.Origin()
	if !proofbound.FiniteVec(origin) ||
		!proofbound.FiniteVec(r3.Vec{X: prism.Z0, Y: prism.Z1, Z: radius}) ||
		radius <= 0 || radius >= record.Radius {
		return nil, false, nil
	}
	start := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(prism.Z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(prism.Z1))
	if start.Cmp(proofarith.FloatRat(record.Z0)) >= 0 ||
		end.Cmp(proofarith.FloatRat(record.Z1)) <= 0 {
		return nil, false, nil
	}
	var wall *Face
	for _, face := range tool.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role != prismrecord.OuterFirstSideRole {
				continue
			}
			if wall != nil {
				return nil, false, nil
			}
			if _, ok := face.n.Surface.(surfacegeom.Cylinder); !ok {
				return nil, false, nil
			}
			wall = face
		}
	}
	if wall == nil {
		return nil, false, nil
	}
	body, err := buildLoftCylinderBore(ctx, d, ref, joined, radius, wall)
	return body, true, err
}

func buildLoftCylinderBore(ctx context.Context, d *Document, ref producerID,
	joined facetedPayload, radius float64, wall *Face) (*Body, error) {
	record := joined.LoftCylinderJoin
	if len(joined.Tris) != len(joined.Src) || len(joined.Verts) != len(joined.VertexBound) {
		return nil, fmt.Errorf("%w: circular join has an incomplete mesh record", ErrUnsupported)
	}
	source := &joinedMesh{Verts: joined.Verts, Tris: joined.Tris, Src: joined.Src}
	for cap := range 2 {
		z := record.Z0
		if cap == 1 {
			z = record.Z1
		}
		if !loftCylinderAuditCap(ctx, source, record.CapOutlines[cap],
			record.CapGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join cap no longer matches its source ring", ErrUnsupported)
		}
	}
	if len(joined.Tris) < 4 || len(joined.Verts) == 0 ||
		record.CapGroups[0] == record.CapGroups[1] ||
		record.MinSideRadius <= 0 || proofbound.IsNonFinite(record.MinSideRadius) {
		return nil, fmt.Errorf("%w: circular join has no separated cap disks", ErrUnsupported)
	}
	const stations = 256
	angles := make([]float64, stations)
	for i := range angles {
		angles[i] = 2 * math.Pi * float64(i) / stations
	}
	innerStations, boreBound, err := circularbounds.BoreRing(ctx, radius, angles)
	if err != nil {
		return nil, err
	}
	if proofbound.IsNonFinite(boreBound) || boreBound >= radius ||
		proofbound.AbsSumUpper(radius, boreBound) >= record.Radius ||
		proofbound.AbsSumUpper(radius, boreBound, joined.MeshBound) >= record.MinSideRadius {
		return nil, fmt.Errorf("%w: circular bore reaches a joined side facet", ErrUnsupported)
	}
	if proofbound.AbsSumUpper(radius, boreBound, record.InnerInset) >= record.Radius {
		return nil, fmt.Errorf("%w: circular bore reaches the joined source seam annulus", ErrUnsupported)
	}
	mesh := &joinedMesh{
		Verts:  append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetGroup(nil), joined.Groups...),
	}
	for i, tri := range joined.Tris {
		if joined.Src[i] != record.CapGroups[0] && joined.Src[i] != record.CapGroups[1] {
			mesh.Add(tri, joined.Src[i])
		}
	}
	wallGroup := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, facetGroup{Origins: wall.originRefs(),
		Surface: wall.n.Surface, NormalBound: wall.n.NormalBound,
		Reversed: !wall.n.Reversed})
	vertexBound := append([]float64(nil), joined.VertexBound...)
	var inner [2][]int
	for cap := range 2 {
		inner[cap] = make([]int, stations)
		z := record.Z0
		if cap == 1 {
			z = record.Z1
		}
		for i, point := range innerStations {
			inner[cap][i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, r3.Vec{X: point.U, Y: point.V, Z: z})
			vertexBound = append(vertexBound, boreBound)
		}
		if cap == 0 && !loftcylinder.BoreRingSimple(ctx, mesh.Verts, inner[cap]) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore ring does not enclose its axis", ErrUnsupported)
		}
		outline := record.CapOutlines[cap]
		points := make([]triangulation.Point2, len(outline)+stations)
		outerLoop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.Verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			outerLoop[i] = i
		}
		holeLoop := make([]int, stations)
		for i := range stations {
			p := mesh.Verts[inner[cap][i]]
			points[len(outline)+i] = triangulation.Point2{U: p.X, V: p.Y}
			holeLoop[i] = len(outline) + stations - 1 - i
		}
		tris, err := triangulation.Triangulate(ctx, points, [][]int{outerLoop, holeLoop})
		if err != nil {
			return nil, triangulation.WrapLoftError(err)
		}
		for _, tri := range tris {
			var mapped [3]int
			for k, index := range tri {
				if index < len(outline) {
					mapped[k] = outline[index]
				} else {
					mapped[k] = inner[cap][index-len(outline)]
				}
			}
			if cap == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.Add(mapped, record.CapGroups[cap])
		}
	}
	for i := range stations {
		j := (i + 1) % stations
		mesh.Add([3]int{inner[0][i], inner[1][i], inner[1][j]}, wallGroup)
		mesh.Add([3]int{inner[0][i], inner[1][j], inner[0][j]}, wallGroup)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(record.Z1), proofarith.FloatRat(record.Z0)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return nil, fmt.Errorf("%w: circular bore has no positive axial span", ErrUnsupported)
	}
	// Cutting the same exact cylinder from the source and held joined solids
	// cannot increase their occupied-volume difference. The new polygonal bore
	// differs from that cylinder only inside its certified radial shell.
	shellArea := proofbound.ProductUpper(
		proofbound.ProductUpper(2, proofbound.TwoPiUpper()),
		proofbound.ProductUpper(radius, boreBound))
	volumeGap := proofbound.AbsSumUpper(joined.VolSymDiff,
		proofbound.ProductUpper(shellArea, height))
	bound := max(joined.MeshBound, boreBound)
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := bodyHandle(record.Blank).Area()
	if err != nil {
		return nil, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, heldArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: circular bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.DPair = facetedSourceDiameterUpper(result.Verts, result.MeshBound)
	if proofbound.IsNonFinite(result.DPair) {
		return nil, fmt.Errorf("%w: circular bore has no finite diameter proof", ErrUnsupported)
	}
	result.LoftCylinderJoin = nil
	result.LoftCylinderBore = &loftCylinderBoreRecord{
		Blank:       record.Blank,
		Teeth:       append([]*topo.Body(nil), record.Teeth...),
		ProofTeeth:  append([]*topo.Body(nil), record.ProofTeeth...),
		AxialBridge: append([]float64(nil), record.AxialBridge...),
		CapGroups:   record.CapGroups, WallGroup: wallGroup,
		BlankRadius: record.Radius, BoreRadius: radius, BoreBound: boreBound,
		Z0: record.Z0, Z1: record.Z1,
		InnerInset: record.InnerInset, OuterExcess: record.OuterExcess, RootCollar: record.RootCollar,
	}
	for cap := range 2 {
		result.LoftCylinderBore.Outer[cap] = append([]int(nil), record.CapOutlines[cap]...)
		result.LoftCylinderBore.Inner[cap] = append([]int(nil), inner[cap]...)
	}
	result.ContactAudited = true
	for cap := range 2 {
		z := record.Z0
		if cap == 1 {
			z = record.Z1
		}
		if !loftCylinderAuditAnnulus(ctx, mesh, result.LoftCylinderBore.Outer[cap],
			result.LoftCylinderBore.Inner[cap], record.CapGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore cap does not tile its annulus", ErrUnsupported)
		}
	}
	if err := loftCylinderTagCaps(&result, bodyHandle(record.Blank), record.CapGroups,
		[2]float64{record.Z0, record.Z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&result, record.CapGroups[1], record.AxialBridge)
	return buildFacetedBody(ctx, d, ref, result)
}
