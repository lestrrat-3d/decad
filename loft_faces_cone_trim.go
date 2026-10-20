package decad

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

func loftFacesCapRecordReady(record *loftFacesCapRecord) bool {
	if record == nil || record.Source == nil {
		return false
	}
	n := len(record.Maps)
	return n >= 2 && n <= maxLoftFacesSections && len(record.Bounds) == n &&
		len(record.StationGroup) == len(record.Source.SourceStation) && len(record.StationGroup) >= 3
}

func uniqueConeFace(body *Body) (*Face, bool) {
	var coneFace *Face
	for _, face := range body.Faces() {
		if _, ok := face.n.Surface.(surfacegeom.Cone); !ok {
			continue
		}
		if coneFace != nil {
			return nil, false
		}
		coneFace = face
	}
	return coneFace, coneFace != nil
}

func indexOfFace(faces []*Face, want *Face) int {
	for i, face := range faces {
		if face == want {
			return i
		}
	}
	return -1
}

// A source group can split into multiple result patches. The cap proof uses
// one connected cap only, so retain the record only for one surviving face.
func uniqueResultSourceFace(body *Body, source int) *Face {
	p, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil
	}
	faces := body.Faces()
	index, ok := facetedtopology.UniqueGroupFace(p.Src, p.FaceOf, len(faces), source)
	if !ok {
		return nil
	}
	return faces[index]
}

func recordLoftFacesConeToe(ctx context.Context, source, tool, result *Body) error {
	base, ok := source.n.Payload.(facetedPayload)
	if !ok || !base.LoftFacesPrimitive || !loftFacesCapRecordReady(base.LoftFacesCap) ||
		base.Xform != r3.Identity() {
		return nil
	}
	cone, ok := revolveplan.PointConeSource(&tool.n)
	if !ok || !pointcone.AboveFarCap(base.Verts, base.MeshBound, cone.Far) {
		return nil
	}
	box, err := revolveloftfaces.LfSourceBox(base.LoftFacesCap.Source)
	if err != nil {
		return err
	}
	if err := loftface.PositiveDenominator(base.LoftFacesCap.Final, box); err != nil {
		return err
	}
	if !facetedrecord.CapOutsideToeCone(base.LoftFacesCap.Final, box, cone) {
		return nil
	}
	capFace, matched, err := unchangedLoftCap(ctx, source, result, base, base.LoftFacesCap.Group)
	if err != nil {
		return err
	}
	if !matched {
		return nil
	}
	coneFace, ok := uniqueConeFace(tool)
	if !ok {
		return nil
	}
	toeCapFace := uniqueResultSourceFace(result, len(source.Faces())+indexOfFace(tool.Faces(), coneFace))
	if toeCapFace == nil {
		return nil
	}
	p, ok := result.n.Payload.(facetedPayload)
	if !ok {
		return nil
	}
	p.LoftFacesToe = &loftFacesToeRecord{Base: base, Cap: base.LoftFacesCap,
		CapFace: faceNode(capFace), ToeCapFace: faceNode(toeCapFace), Toe: cone, ToeTool: bodyNode(tool)}
	result.n.Payload = p
	return nil
}

// A Cut may keep only part of a cap. Require the exact held triangle multiset
// and one unchanged source patch before carrying the cap's true-map proof.
func unchangedLoftCap(ctx context.Context, source, result *Body, base facetedPayload, group int) (*Face, bool, error) {
	resultFaces := result.Faces()
	var held *facetedtopology.GroupedMesh
	if p, ok := result.n.Payload.(facetedPayload); ok {
		held = &facetedtopology.GroupedMesh{Verts: p.Verts, Tris: p.Tris, Src: p.Src, FaceOf: p.FaceOf}
	}
	index, ok, err := facetedtopology.UnchangedGroupPatch(ctx,
		facetedtopology.GroupedMesh{Verts: base.Verts, Tris: base.Tris, Src: base.Src, FaceOf: base.FaceOf},
		len(base.Groups), group, len(source.Faces()), held, len(resultFaces))
	if err != nil || !ok {
		return nil, false, err
	}
	return resultFaces[index], true, nil
}

func tryLoftFacesConeHeel(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpIntersect {
		return nil, false, nil
	}
	p, ok := target.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesToe == nil || !loftFacesCapRecordReady(p.LoftFacesToe.Cap) ||
		p.Xform != r3.Identity() ||
		p.LoftFacesToe.CapFace == nil || p.LoftFacesToe.CapFace.Body != bodyNode(target) {
		return nil, false, nil
	}
	cone, ok := revolveplan.PointConeSource(&tool.n)
	if !ok || cone.Far.Cmp(p.LoftFacesToe.Toe.Far) != 0 ||
		!pointcone.AboveFarCap(p.LoftFacesToe.Base.Verts, p.LoftFacesToe.Base.MeshBound, cone.Far) ||
		!orderedLoftFacesConeLimits(p.LoftFacesToe.Toe, cone, p.LoftFacesToe.Base) {
		return nil, false, nil
	}
	box, err := revolveloftfaces.LfSourceBox(p.LoftFacesToe.Cap.Source)
	if err != nil {
		return nil, true, asBooleanError(op, err)
	}
	if err := loftface.PositiveDenominator(p.LoftFacesToe.Cap.Final, box); err != nil {
		return nil, true, asBooleanError(op, err)
	}
	var plane *loftface.TangentPlane
	if !facetedrecord.CapOutsideHeelCone(p.LoftFacesToe.Cap.Final, box, cone) {
		tangent, err := loftCapTangentToHeelCone(p.LoftFacesToe.Cap, box, cone)
		if err != nil {
			return nil, true, asBooleanError(op, err)
		}
		if tangent == nil {
			return nil, false, nil
		}
		plane = tangent
	}
	coneFace, ok := uniqueConeFace(tool)
	if !ok {
		return nil, false, nil
	}
	capSource := -1
	for i, face := range target.Faces() {
		if faceNode(face) == p.LoftFacesToe.CapFace {
			capSource = i
			break
		}
	}
	if capSource < 0 {
		return nil, false, nil
	}
	toeCapSource := indexOfFace(target.Faces(), faceHandle(p.LoftFacesToe.ToeCapFace))
	if toeCapSource < 0 {
		return nil, false, nil
	}
	heelCapSource := len(target.Faces()) + indexOfFace(tool.Faces(), coneFace)
	// A larger chord may be needed when the toe Cut's local rim bound is
	// coarse. Retry only its staging refusal; every other gate keeps its
	// verdict. Each attempt proves its own tessellation and result bounds.
	var eval booleanEvaluation
	var meshes *tangentHeelMeshes
	for scale := 4.0; scale <= 32; scale *= 2 {
		if plane == nil {
			eval, err = evaluateBooleanMeshesWithConeCap(ctx, op, target, tool, pairMeshes{}, scale,
				p.LoftFacesToe.CapFace, faceNode(coneFace))
		} else {
			// The tangent cap touches the cone, so no pair is exempted. The
			// tooth enters the ordinary gate with an exact prism glued to its
			// end cap (section 11 of the design; "Two-cone continuation").
			meshes = &tangentHeelMeshes{target: target, toe: p.LoftFacesToe, plane: *plane}
			eval, err = evaluateBooleanMeshesWithConeCap(ctx, op, target, tool, meshes, scale, nil, nil)
		}
		if err == nil {
			break
		}
		var expected *meshbool.BooleanExpectedError
		if scale == 32 || !errors.As(err, &expected) ||
			expected.Kind != meshbool.BooleanExpectedStaging || expected.Operand != 0 {
			return nil, true, asBooleanError(op, err)
		}
	}
	if meshes != nil {
		if err := meshes.clearsResult(ctx, eval.payload.Verts); err != nil {
			return nil, true, asBooleanError(op, err)
		}
	}
	// The cap and wall have no held contact. The mesh result must exclude
	// the whole cap, agreeing with the exact proof that it is outside.
	if slices.Contains(eval.payload.Src, capSource) {
		return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact,
			fmt.Errorf("%w: the held cone retains a cap certified outside its true surface", ErrUnsupported)))
	}
	if plane != nil {
		// The tangent trim record needs both tool faces as single result
		// patches. Refuse before the body is built, not after.
		unique, err := facetedrecord.UniqueSourcePatches(ctx, eval.payload, eval.audit.Adj, toeCapSource, heelCapSource)
		if err != nil {
			return nil, true, asBooleanError(op, err)
		}
		if !unique {
			return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact,
				fmt.Errorf("%w: the tangent heel result has no single toe or heel tool face", ErrUnsupported)))
		}
	}
	body, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), eval.payload, eval.audit, measurementToInternal(eval.volume), eval.volumeRat))
	if err != nil {
		return nil, true, asBooleanError(op, err)
	}
	result, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return body, true, nil
	}
	if uniqueResultSourceFace(body, toeCapSource) == nil ||
		uniqueResultSourceFace(body, heelCapSource) == nil {
		return body, true, nil
	}
	result.LoftFacesTrim = &loftFacesTrimRecord{
		Base: p.LoftFacesToe.Base, Source: p.LoftFacesToe.Cap.Source,
		Maps:         append([]loftFaceMap(nil), p.LoftFacesToe.Cap.Maps...),
		Bounds:       append([]loftFaceCapBounds(nil), p.LoftFacesToe.Cap.Bounds...),
		StationGroup: append([]int(nil), p.LoftFacesToe.Cap.StationGroup...),
		Toe:          p.LoftFacesToe.Toe, Heel: cone,
		ToeTool: p.LoftFacesToe.ToeTool, HeelTool: bodyNode(tool),
		ToeCapGroup: toeCapSource, HeelCapGroup: heelCapSource,
		Tangent: p.LoftFacesToe.Cap.Tangent,
	}
	body.n.Payload = result
	return body, true, nil
}

func orderedLoftFacesConeLimits(toe, heel *pointConeLimit, base facetedPayload) bool {
	return pointcone.ToeHeelOrdered(toe.Apex, toe.Slope, heel.Apex, heel.Slope,
		base.Verts, base.MeshBound)
}

// loftCapTangentToHeelCone admits the cap of a cone-tangent LoftFaces end
// that touches the heel cone along one ruling. It returns the cap's exact
// plane, with the loft side, when all of these hold, and nil when the record
// is not a cone-tangent end for this cone:
//
//   - the record's tangent end was built for this cone's exact apex and slope
//     and reproduces the recorded final map;
//   - the final cap's pullback to the cone is exactly C*v^2 with C > 0 over a
//     source rectangle strictly on one side of u = 0
//     (loftface.ConeTangentAlongRuling);
//   - the cone lies on the loft side of the cap plane (G1,
//     loftface.ConeOnLoftSide);
//   - every earlier cap lies strictly on that side (G2, loftface.CapsOnSide).
//
// An error wrapping ErrUnsupported means the record is a tangent end for
// this cone but the loft and the cone are not provably on one side of the
// cap plane. The caller refuses; it never falls back to a skipped pair.
// See docs/loft-face-sections-design.md, "Two-cone continuation".
func loftCapTangentToHeelCone(record *loftFacesCapRecord, box [4]*big.Rat,
	cone *pointConeLimit) (*loftface.TangentPlane, error) {
	rec := record.Tangent
	if rec == nil || !rec.MatchesCone(cone.Apex, cone.Slope) ||
		!loftface.MapEqual(rec.Map(), record.Final) ||
		!loftface.ConeTangentAlongRuling(record.Final, box, cone.Apex, cone.Slope) {
		return nil, nil //nolint:nilnil // nil plane with nil error means the record is not a tangent end for this cone.
	}
	if len(record.Maps) < 2 {
		return nil, nil //nolint:nilnil // same: not a tangent end for this cone.
	}
	side, ok := loftface.ConeOnLoftSide(record.Final, record.Maps[len(record.Maps)-2], box,
		cone.Apex, cone.Slope, cone.Far)
	if !ok {
		return nil, fmt.Errorf("%w: the heel cone is not provably on the loft side of the tangent cap plane",
			ErrUnsupported)
	}
	if !loftface.CapsOnSide(record.Final, record.Maps[:len(record.Maps)-1], box, side) {
		return nil, fmt.Errorf("%w: an earlier cap is not provably on the loft side of the tangent cap plane",
			ErrUnsupported)
	}
	plane, ok := loftface.NewTangentPlane(record.Final, side)
	if !ok {
		return nil, fmt.Errorf("%w: the tangent cap has no exact plane", ErrUnsupported)
	}
	return &plane, nil
}

// tangentCapExtensionLength is the length in millimetres of the prism that
// tangentHeelMeshes glues to a tangent end cap. The exact equality
// (T ∪ E) ∩ K = T ∩ K does not depend on it, and Gate E5 decides the held
// clearance for whatever length is used.
const tangentCapExtensionLength = 1

// tangentHeelMeshes supplies the heel Intersect of a cone-tangent tooth with
// the tooth's mesh extended by an exact prism glued to its end cap
// (loftface.CapExtension). The cone is meshed as usual. Because the proxy is
// restating, the restated chain-depth gate applies to it, and no cap/cone
// face pair is exempted from the proximity gate. It records the meshes it
// hands out so Gate E5 can check them after the Boolean. Use one value per
// Boolean attempt.
type tangentHeelMeshes struct {
	target *Body
	toe    *loftFacesToeRecord
	plane  loftface.TangentPlane

	slab      loftface.Slab
	hasSlab   bool
	coneVerts []r3.Vec
	coneTris  [][3]int

	// pairTol, proxy and cone are the tolerance and the two meshes the Boolean
	// read. Tests rebuild the composed bounds from them.
	pairTol     float64
	proxy, cone *Mesh
}

func (m *tangentHeelMeshes) operandMesh(ctx context.Context, b *Body, pairTol float64) (*Mesh, bool, error) {
	mesh, restating, err := tessellateBooleanOperand(ctx, b, pairTol)
	if err != nil {
		return mesh, restating, err
	}
	if b != m.target {
		verts, tris := mesh.n.Vertices, mesh.n.Triangles
		m.coneVerts, m.coneTris, m.cone = verts, tris, mesh
		return mesh, restating, nil
	}
	proxy, slab, err := extendTangentCap(ctx, mesh, m.toe, m.plane)
	if err != nil {
		return nil, false, err
	}
	m.slab, m.hasSlab, m.pairTol, m.proxy = slab, true, pairTol, proxy
	return proxy, restating, nil
}

// clearsResult runs the held-only part of Gate E5 on the meshes the last
// Boolean attempt used, then the post-Boolean defence on the result vertices.
// Any failure wraps ErrUnsupported (or is a canceled context).
func (m *tangentHeelMeshes) clearsResult(ctx context.Context, result []r3.Vec) error {
	if !m.hasSlab || m.coneTris == nil {
		return fmt.Errorf("%w: the tangent heel Boolean recorded no operand meshes", ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := loftface.ExtensionClearsCone(m.slab, m.coneVerts, m.coneTris, m.plane, budget); err != nil {
		return err
	}
	depth, ok := loftface.ConeDepth(m.coneVerts, m.plane)
	if !ok {
		return fmt.Errorf("%w: the held heel cone has no usable vertex", ErrUnsupported)
	}
	if depth.Cmp(m.slab.Eps) < 0 {
		depth = m.slab.Eps
	}
	if loftface.ResultBeyondSlab(result, m.plane, depth) {
		return fmt.Errorf("%w: the tangent heel result has a vertex beyond the end-cap slab", ErrUnsupported)
	}
	return nil
}

// extendTangentCap returns a copy of held with the prism glued to the end
// cap, with the prism's clearance data. It refuses with ErrUnsupported when
// held is not a verified mesh, a ring vertex is not a station vertex of the
// toe record's last cap, or the extension fails its audit.
//
// The proxy carries the unextended operand's VolSymDiff and AreaSlack. That is
// valid only for the heel Boolean under Gate E5, which proves the prism
// changes no held contact and no retained piece, so every result facet is a
// piece of a non-extension facet of the operands. The proxy is never published.
func extendTangentCap(ctx context.Context, held *Mesh, toe *loftFacesToeRecord,
	plane loftface.TangentPlane) (*Mesh, loftface.Slab, error) {
	refuse := func(why string) (*Mesh, loftface.Slab, error) {
		return nil, loftface.Slab{}, fmt.Errorf("%w: the tangent tooth %s", ErrUnsupported, why)
	}
	vertices, triangles, sources, capFace := held.n.Vertices, held.n.Triangles, held.n.Source, toe.CapFace
	if !held.n.BoundaryOK || !held.n.SymDiffOK {
		return refuse("mesh is not verified")
	}
	link, err := revolveloftfaces.TangentToeStationLink(vertices, toe)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	beta, err := topobuild.VertexBounds(&held.n)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	var faces []*topo.Face
	ids := map[*topo.Face]int{}
	face := make([]int, len(sources))
	for i, f := range sources {
		id, ok := ids[f]
		if !ok {
			id = len(faces)
			ids[f] = id
			faces = append(faces, f)
		}
		face[i] = id
	}
	capID, ok := ids[capFace]
	if !ok {
		return refuse("mesh has no end cap facet")
	}
	ext, err := loftface.CapExtension(ctx, loftface.HeldMesh{Vertices: vertices, Triangles: triangles,
		Face: face, VertexBound: beta}, capID, plane, big.NewRat(tangentCapExtensionLength, 1), link)
	if err != nil {
		return nil, loftface.Slab{}, err
	}
	next := bornMesh(&Mesh{n: held.n})
	next.n.Vertices = ext.Vertices
	next.n.Triangles = ext.Triangles
	next.n.VertexBound = ext.VertexBound
	next.n.Source = make([]*topo.Face, len(ext.Face))
	for i, id := range ext.Face {
		next.n.Source[i] = faces[id]
	}
	next.n.FaceBound = maps.Clone(held.n.FaceBound)
	for id, bound := range ext.FaceBound {
		next.n.FaceBound[faces[id]] = bound
	}
	next.n.Bound = 0
	for _, bound := range next.n.FaceBound {
		next.n.Bound = max(next.n.Bound, bound)
	}
	next.n.CoordBound = proofbound.AbsSumUpper(held.n.CoordBound, ext.CoordCharge)
	if proofbound.IsNonFinite(next.n.Bound) || proofbound.IsNonFinite(next.n.CoordBound) {
		return refuse("cap extension has an unbounded bound")
	}
	return next, ext.Slab, nil
}
