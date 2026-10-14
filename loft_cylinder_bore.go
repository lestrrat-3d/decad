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
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// loftCylinderBoreRecord names the two certified annular caps of an unplaced
// through-bore result. Each outer and inner loop is CCW when viewed from +Z.
type loftCylinderBoreRecord struct {
	blank *Body
	// teeth are exact Cut sources; proofTeeth and axialBridge retain the
	// private Union witnesses for later source-level admission checks.
	teeth, proofTeeth                   []*Body
	axialBridge                         []float64
	outer, inner                        [2][]int
	capGroups                           [2]int
	wallGroup                           int
	blankRadius, boreRadius, boreBound  float64
	innerInset, outerExcess, rootCollar float64
	z0, z1                              float64
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
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.loftCylinderJoin == nil || !joined.contactAudited ||
		joined.xform != r3.Identity() {
		return nil, false, nil
	}
	record := joined.loftCylinderJoin
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
		radius <= 0 || radius >= record.radius {
		return nil, false, nil
	}
	start := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(prism.Z0))
	end := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(prism.Z1))
	if start.Cmp(proofarith.FloatRat(record.z0)) >= 0 ||
		end.Cmp(proofarith.FloatRat(record.z1)) <= 0 {
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
	record := joined.loftCylinderJoin
	if len(joined.tris) != len(joined.src) || len(joined.verts) != len(joined.vertexBound) {
		return nil, fmt.Errorf("%w: circular join has an incomplete mesh record", ErrUnsupported)
	}
	source := &joinedMesh{verts: joined.verts, tris: joined.tris, src: joined.src}
	for cap := range 2 {
		z := record.z0
		if cap == 1 {
			z = record.z1
		}
		if !loftCylinderAuditCap(ctx, source, record.capOutlines[cap],
			record.capGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join cap no longer matches its source ring", ErrUnsupported)
		}
	}
	if len(joined.tris) < 4 || len(joined.verts) == 0 ||
		record.capGroups[0] == record.capGroups[1] ||
		record.minSideRadius <= 0 || proofbound.IsNonFinite(record.minSideRadius) {
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
		proofbound.AbsSumUpper(radius, boreBound) >= record.radius ||
		proofbound.AbsSumUpper(radius, boreBound, joined.meshBound) >= record.minSideRadius {
		return nil, fmt.Errorf("%w: circular bore reaches a joined side facet", ErrUnsupported)
	}
	if proofbound.AbsSumUpper(radius, boreBound, record.innerInset) >= record.radius {
		return nil, fmt.Errorf("%w: circular bore reaches the joined source seam annulus", ErrUnsupported)
	}
	mesh := &joinedMesh{
		verts:  append([]r3.Vec(nil), joined.verts...),
		groups: append([]facetGroup(nil), joined.groups...),
	}
	for i, tri := range joined.tris {
		if joined.src[i] != record.capGroups[0] && joined.src[i] != record.capGroups[1] {
			mesh.add(tri, joined.src[i])
		}
	}
	wallGroup := len(mesh.groups)
	mesh.groups = append(mesh.groups, facetGroup{origins: wall.originRefs(),
		surface: wall.n.Surface, normalBound: wall.n.NormalBound,
		reversed: !wall.n.Reversed})
	vertexBound := append([]float64(nil), joined.vertexBound...)
	var inner [2][]int
	for cap := range 2 {
		inner[cap] = make([]int, stations)
		z := record.z0
		if cap == 1 {
			z = record.z1
		}
		for i, point := range innerStations {
			inner[cap][i] = len(mesh.verts)
			mesh.verts = append(mesh.verts, r3.Vec{X: point.U, Y: point.V, Z: z})
			vertexBound = append(vertexBound, boreBound)
		}
		if cap == 0 && !loftcylinder.BoreRingSimple(ctx, mesh.verts, inner[cap]) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore ring does not enclose its axis", ErrUnsupported)
		}
		outline := record.capOutlines[cap]
		points := make([]triangulation.Point2, len(outline)+stations)
		outerLoop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			outerLoop[i] = i
		}
		holeLoop := make([]int, stations)
		for i := range stations {
			p := mesh.verts[inner[cap][i]]
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
			mesh.add(mapped, record.capGroups[cap])
		}
	}
	for i := range stations {
		j := (i + 1) % stations
		mesh.add([3]int{inner[0][i], inner[1][i], inner[1][j]}, wallGroup)
		mesh.add([3]int{inner[0][i], inner[1][j], inner[0][j]}, wallGroup)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(record.z1), proofarith.FloatRat(record.z0)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return nil, fmt.Errorf("%w: circular bore has no positive axial span", ErrUnsupported)
	}
	// Cutting the same exact cylinder from the source and held joined solids
	// cannot increase their occupied-volume difference. The new polygonal bore
	// differs from that cylinder only inside its certified radial shell.
	shellArea := proofbound.ProductUpper(
		proofbound.ProductUpper(2, proofbound.TwoPiUpper()),
		proofbound.ProductUpper(radius, boreBound))
	volumeGap := proofbound.AbsSumUpper(joined.volSymDiff,
		proofbound.ProductUpper(shellArea, height))
	bound := max(joined.meshBound, boreBound)
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := record.blank.Area()
	if err != nil {
		return nil, err
	}
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.areaSlack, heldArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: circular bore has no finite measure proof", ErrUnsupported)
	}
	result := joined
	result.verts, result.tris, result.src, result.groups = mesh.verts, mesh.tris, mesh.src, mesh.groups
	result.vertexBound, result.meshBound = vertexBound, bound
	result.volSymDiff, result.areaSlack = volumeGap, areaSlack
	result.dPair = facetedSourceDiameterUpper(result.verts, result.meshBound)
	if proofbound.IsNonFinite(result.dPair) {
		return nil, fmt.Errorf("%w: circular bore has no finite diameter proof", ErrUnsupported)
	}
	result.loftCylinderJoin = nil
	result.loftCylinderBore = &loftCylinderBoreRecord{
		blank:       record.blank,
		teeth:       append([]*Body(nil), record.teeth...),
		proofTeeth:  append([]*Body(nil), record.proofTeeth...),
		axialBridge: append([]float64(nil), record.axialBridge...),
		capGroups:   record.capGroups, wallGroup: wallGroup,
		blankRadius: record.radius, boreRadius: radius, boreBound: boreBound,
		z0: record.z0, z1: record.z1,
		innerInset: record.innerInset, outerExcess: record.outerExcess, rootCollar: record.rootCollar,
	}
	for cap := range 2 {
		result.loftCylinderBore.outer[cap] = append([]int(nil), record.capOutlines[cap]...)
		result.loftCylinderBore.inner[cap] = append([]int(nil), inner[cap]...)
	}
	result.contactAudited = true
	for cap := range 2 {
		z := record.z0
		if cap == 1 {
			z = record.z1
		}
		if !loftCylinderAuditAnnulus(ctx, mesh, result.loftCylinderBore.outer[cap],
			result.loftCylinderBore.inner[cap], record.capGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore cap does not tile its annulus", ErrUnsupported)
		}
	}
	if err := loftCylinderTagCaps(&result, record.blank, record.capGroups,
		[2]float64{record.z0, record.z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&result, record.capGroups[1], record.axialBridge)
	return buildFacetedBody(ctx, d, ref, result)
}
