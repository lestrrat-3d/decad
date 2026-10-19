package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// boreStations is the number of inner ring stations a bore places per cap.
const boreStations = 256

// BoreTool returns the radius of a bore tool's profile: one full circle
// centered on its section origin, with no holes, surface result or section
// displacement. Its radius and both extent levels must be finite, and the
// radius must be positive and below blankRadius. A mismatch reports false
// with a nil error, so the generic Cut keeps the operands; only a radius
// that is not a length is an error. The tool's frame and placement are the
// caller's to check.
func BoreTool(pp prismrecord.Payload, blankRadius float64) (float64, bool, error) {
	if pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 {
		return 0, false, nil
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center != (sectionrecord.Point2{}) ||
		!((circle.TStart == 0 && circle.TEnd == 1) ||
			(circle.TStart == 1 && circle.TEnd == 0)) {
		return 0, false, nil
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return 0, false, err
	}
	if !proofbound.FiniteVec(r3.Vec{X: pp.Z0, Y: pp.Z1, Z: radius}) ||
		radius <= 0 || radius >= blankRadius {
		return 0, false, nil
	}
	return radius, true, nil
}

// BoreThrough reports whether a bore tool whose frame origin sits at axial
// level originZ and whose extent runs from z0 to z1 starts strictly below
// joinZ0 and ends strictly above joinZ1, in exact rational arithmetic.
func BoreThrough(originZ, z0, z1, joinZ0, joinZ1 float64) bool {
	start := new(big.Rat).Add(proofarith.FloatRat(originZ), proofarith.FloatRat(z0))
	end := new(big.Rat).Add(proofarith.FloatRat(originZ), proofarith.FloatRat(z1))
	return start.Cmp(proofarith.FloatRat(joinZ0)) < 0 && end.Cmp(proofarith.FloatRat(joinZ1)) > 0
}

// BoreCut replaces the two planar cap disks of the audited identity-placed
// loft/cylinder join joined with annuli and adds the inward cylinder wall of
// tool, a centered axial circular prism that passes through both caps. The
// join record makes the cap disks available without asking a new mesh
// boolean to resolve the earlier tooth/cylinder seams again. The result is
// built in env, and env.Ref names its cap roles. A join or tool outside this
// shape reports false with a nil error; an error after admission reports
// true.
func BoreCut(ctx context.Context, env topo.Env, joined facetedrecord.Payload,
	tool *topo.Body) (*topo.Body, bool, error) {
	record := joined.LoftCylinderJoin
	if record == nil || !joined.ContactAudited || joined.Xform != r3.Identity() {
		return nil, false, nil
	}
	prism, ok := tool.Payload.(prismrecord.Payload)
	if !ok || prism.Xform != r3.Identity() ||
		prism.Frame.U() != (r3.Vec{X: 1}) || prism.Frame.V() != (r3.Vec{Y: 1}) ||
		prism.Frame.N() != (r3.Vec{Z: 1}) ||
		prism.Frame.Origin().X != 0 || prism.Frame.Origin().Y != 0 {
		return nil, false, nil
	}
	radius, ok, err := BoreTool(prism, record.Radius)
	if err != nil || !ok {
		return nil, false, err
	}
	origin := prism.Frame.Origin()
	if !proofbound.FiniteVec(origin) || !BoreThrough(origin.Z, prism.Z0, prism.Z1, record.Z0, record.Z1) {
		return nil, false, nil
	}
	var wall *topo.Face
	for _, face := range topobuild.Faces(tool) {
		for _, origin := range face.Origins {
			if origin.Role != prismrecord.OuterFirstSideRole {
				continue
			}
			if wall != nil {
				return nil, false, nil
			}
			if _, ok := face.Surface.(surfacegeom.Cylinder); !ok {
				return nil, false, nil
			}
			wall = face
		}
	}
	if wall == nil {
		return nil, false, nil
	}
	body, err := buildBore(ctx, env, joined, radius, wall)
	return body, true, err
}

func buildBore(ctx context.Context, env topo.Env, joined facetedrecord.Payload,
	radius float64, wall *topo.Face) (*topo.Body, error) {
	record := joined.LoftCylinderJoin
	if len(joined.Tris) != len(joined.Src) || len(joined.Verts) != len(joined.VertexBound) {
		return nil, fmt.Errorf("%w: circular join has an incomplete mesh record", decaderr.ErrUnsupported)
	}
	for cap := range 2 {
		z := record.Z0
		if cap == 1 {
			z = record.Z1
		}
		if !AuditCap(ctx, joined.Verts, joined.Tris, joined.Src, record.CapOutlines[cap],
			record.CapGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join cap no longer matches its source ring", decaderr.ErrUnsupported)
		}
	}
	if len(joined.Tris) < 4 || len(joined.Verts) == 0 ||
		record.CapGroups[0] == record.CapGroups[1] ||
		record.MinSideRadius <= 0 || proofbound.IsNonFinite(record.MinSideRadius) {
		return nil, fmt.Errorf("%w: circular join has no separated cap disks", decaderr.ErrUnsupported)
	}
	angles := make([]float64, boreStations)
	for i := range angles {
		angles[i] = 2 * math.Pi * float64(i) / boreStations
	}
	innerStations, boreBound, err := circularbounds.BoreRing(ctx, radius, angles)
	if err != nil {
		return nil, err
	}
	if proofbound.IsNonFinite(boreBound) || boreBound >= radius ||
		proofbound.AbsSumUpper(radius, boreBound) >= record.Radius ||
		proofbound.AbsSumUpper(radius, boreBound, joined.MeshBound) >= record.MinSideRadius {
		return nil, fmt.Errorf("%w: circular bore reaches a joined side facet", decaderr.ErrUnsupported)
	}
	if proofbound.AbsSumUpper(radius, boreBound, record.InnerInset) >= record.Radius {
		return nil, fmt.Errorf("%w: circular bore reaches the joined source seam annulus", decaderr.ErrUnsupported)
	}
	mesh := &facetedrecord.JoinedMesh{
		Verts:  append([]r3.Vec(nil), joined.Verts...),
		Groups: append([]facetedrecord.Group(nil), joined.Groups...),
	}
	for i, tri := range joined.Tris {
		if joined.Src[i] != record.CapGroups[0] && joined.Src[i] != record.CapGroups[1] {
			mesh.Add(tri, joined.Src[i])
		}
	}
	wallGroup := len(mesh.Groups)
	mesh.Groups = append(mesh.Groups, facetedrecord.Group{
		Origins: append([]provenance.Ref(nil), wall.Origins...),
		Surface: wall.Surface, NormalBound: wall.NormalBound,
		Reversed: !wall.Reversed})
	vertexBound := append([]float64(nil), joined.VertexBound...)
	var inner [2][]int
	for cap := range 2 {
		inner[cap] = make([]int, boreStations)
		z := record.Z0
		if cap == 1 {
			z = record.Z1
		}
		for i, point := range innerStations {
			inner[cap][i] = len(mesh.Verts)
			mesh.Verts = append(mesh.Verts, r3.Vec{X: point.U, Y: point.V, Z: z})
			vertexBound = append(vertexBound, boreBound)
		}
		if cap == 0 && !BoreRingSimple(ctx, mesh.Verts, inner[cap]) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore ring does not enclose its axis", decaderr.ErrUnsupported)
		}
		outline := record.CapOutlines[cap]
		points := make([]triangulation.Point2, len(outline)+boreStations)
		outerLoop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.Verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			outerLoop[i] = i
		}
		holeLoop := make([]int, boreStations)
		for i := range boreStations {
			p := mesh.Verts[inner[cap][i]]
			points[len(outline)+i] = triangulation.Point2{U: p.X, V: p.Y}
			holeLoop[i] = len(outline) + boreStations - 1 - i
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
	for i := range boreStations {
		j := (i + 1) % boreStations
		mesh.Add([3]int{inner[0][i], inner[1][i], inner[1][j]}, wallGroup)
		mesh.Add([3]int{inner[0][i], inner[1][j], inner[0][j]}, wallGroup)
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris,
		loftmesh.MaxLoftAuditTriangles, AuditMaxScans); err != nil {
		return nil, err
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(record.Z1), proofarith.FloatRat(record.Z0)))
	if height <= 0 || proofbound.IsNonFinite(height) {
		return nil, fmt.Errorf("%w: circular bore has no positive axial span", decaderr.ErrUnsupported)
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
	blankArea := record.Blank.Area
	wallArea := proofbound.ProductUpper(
		proofbound.ProductUpper(proofbound.TwoPiUpper(), radius), height)
	areaSlack := proofbound.AbsSumUpper(joined.AreaSlack, heldArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), wallArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: circular bore has no finite measure proof", decaderr.ErrUnsupported)
	}
	result := joined
	result.Verts, result.Tris, result.Src, result.Groups = mesh.Verts, mesh.Tris, mesh.Src, mesh.Groups
	result.VertexBound, result.MeshBound = vertexBound, bound
	result.VolSymDiff, result.AreaSlack = volumeGap, areaSlack
	result.DPair = facetedrecord.SourceDiameterUpper(result.Verts, result.MeshBound)
	if proofbound.IsNonFinite(result.DPair) {
		return nil, fmt.Errorf("%w: circular bore has no finite diameter proof", decaderr.ErrUnsupported)
	}
	result.LoftCylinderJoin = nil
	result.LoftCylinderBore = &facetedrecord.LoftCylinderBoreRecord{
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
		if !AuditAnnulus(ctx, mesh.Verts, mesh.Tris, mesh.Src, result.LoftCylinderBore.Outer[cap],
			result.LoftCylinderBore.Inner[cap], record.CapGroups[cap], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular bore cap does not tile its annulus", decaderr.ErrUnsupported)
		}
	}
	if err := TagCaps(&result, record.Blank, record.CapGroups,
		[2]float64{record.Z0, record.Z1}, env.Ref); err != nil {
		return nil, err
	}
	TagBridgedTop(&result, record.CapGroups[1], record.AxialBridge)
	return facetedrecord.Build(ctx, env, result)
}
