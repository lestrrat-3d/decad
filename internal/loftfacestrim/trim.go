package loftfacestrim

import (
	"context"
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// MaxSections is the most sections a LoftFaces body takes.
const MaxSections = 32

// CapRecordReady reports whether record carries enough of a cap to certify it.
func CapRecordReady(record *facetedrecord.LoftFacesCapRecord) bool {
	if record == nil || record.Source == nil {
		return false
	}
	n := len(record.Maps)
	return n >= 2 && n <= MaxSections && len(record.Bounds) == n &&
		len(record.StationGroup) == len(record.Source.SourceStation) && len(record.StationGroup) >= 3
}

func uniqueConeFace(body *topo.Body) (*topo.Face, bool) {
	var coneFace *topo.Face
	for _, face := range topobuild.Faces(body) {
		if _, ok := face.Surface.(surfacegeom.Cone); !ok {
			continue
		}
		if coneFace != nil {
			return nil, false
		}
		coneFace = face
	}
	return coneFace, coneFace != nil
}

// IndexOfFace returns the position of want in faces, or -1.
func IndexOfFace(faces []*topo.Face, want *topo.Face) int {
	for i, face := range faces {
		if face == want {
			return i
		}
	}
	return -1
}

// RecordConeToe records the toe trim of a certified LoftFaces primitive cut
// by a cone on result, when every gate of the toe proof holds.
func RecordConeToe(ctx context.Context, source, tool, result *topo.Body) error {
	base, ok := source.Payload.(facetedrecord.Payload)
	if !ok || !base.LoftFacesPrimitive || !CapRecordReady(base.LoftFacesCap) ||
		base.Xform != r3.Identity() {
		return nil
	}
	cone, ok := revolveplan.PointConeSource(tool)
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
	toeCapFace := revolvecontact.UniqueResultSourceFace(result, len(topobuild.Faces(source))+IndexOfFace(topobuild.Faces(tool), coneFace))
	if toeCapFace == nil {
		return nil
	}
	p, ok := result.Payload.(facetedrecord.Payload)
	if !ok {
		return nil
	}
	p.LoftFacesToe = &facetedrecord.LoftFacesToeRecord{Base: base, Cap: base.LoftFacesCap,
		CapFace: capFace, ToeCapFace: toeCapFace, Toe: cone, ToeTool: tool}
	result.Payload = p
	return nil
}

// A Cut may keep only part of a cap. Require the exact held triangle multiset
// and one unchanged source patch before carrying the cap's true-map proof.
func unchangedLoftCap(ctx context.Context, source, result *topo.Body, base facetedrecord.Payload, group int) (*topo.Face, bool, error) {
	resultFaces := topobuild.Faces(result)
	var held *facetedtopology.GroupedMesh
	if p, ok := result.Payload.(facetedrecord.Payload); ok {
		held = &facetedtopology.GroupedMesh{Verts: p.Verts, Tris: p.Tris, Src: p.Src, FaceOf: p.FaceOf}
	}
	index, ok, err := facetedtopology.UnchangedGroupPatch(ctx,
		facetedtopology.GroupedMesh{Verts: base.Verts, Tris: base.Tris, Src: base.Src, FaceOf: base.FaceOf},
		len(base.Groups), group, len(topobuild.Faces(source)), held, len(resultFaces))
	if err != nil || !ok {
		return nil, false, err
	}
	return resultFaces[index], true, nil
}

// ConeHeelBoolean carries the toe trim of a cone-cut LoftFaces body through an
// Intersect with the heel cone, or declines. Every error it returns is the raw
// evaluator error; the caller classifies it for the operation.
func ConeHeelBoolean(ctx context.Context, op meshbool.OperationKind, env topo.Env,
	mesher booleaneval.OperandMesher, target, tool *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpIntersect {
		return nil, false, nil
	}
	p, ok := target.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesToe == nil || !CapRecordReady(p.LoftFacesToe.Cap) ||
		p.Xform != r3.Identity() ||
		p.LoftFacesToe.CapFace == nil || p.LoftFacesToe.CapFace.Body != target {
		return nil, false, nil
	}
	cone, ok := revolveplan.PointConeSource(tool)
	if !ok || cone.Far.Cmp(p.LoftFacesToe.Toe.Far) != 0 ||
		!pointcone.AboveFarCap(p.LoftFacesToe.Base.Verts, p.LoftFacesToe.Base.MeshBound, cone.Far) ||
		!orderedConeLimits(p.LoftFacesToe.Toe, cone, p.LoftFacesToe.Base) {
		return nil, false, nil
	}
	box, err := revolveloftfaces.LfSourceBox(p.LoftFacesToe.Cap.Source)
	if err != nil {
		return nil, true, err
	}
	if err := loftface.PositiveDenominator(p.LoftFacesToe.Cap.Final, box); err != nil {
		return nil, true, err
	}
	var plane *loftface.TangentPlane
	if !facetedrecord.CapOutsideHeelCone(p.LoftFacesToe.Cap.Final, box, cone) {
		tangent, err := facetedrecord.CapTangentToHeelCone(p.LoftFacesToe.Cap, box, cone)
		if err != nil {
			return nil, true, err
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
	for i, face := range topobuild.Faces(target) {
		if face == p.LoftFacesToe.CapFace {
			capSource = i
			break
		}
	}
	if capSource < 0 {
		return nil, false, nil
	}
	toeCapSource := IndexOfFace(topobuild.Faces(target), p.LoftFacesToe.ToeCapFace)
	if toeCapSource < 0 {
		return nil, false, nil
	}
	heelCapSource := len(topobuild.Faces(target)) + IndexOfFace(topobuild.Faces(tool), coneFace)
	// A larger chord may be needed when the toe Cut's local rim bound is
	// coarse. Retry only its staging refusal; every other gate keeps its
	// verdict. Each attempt proves its own tessellation and result bounds.
	var eval booleaneval.Evaluation
	var meshes *TangentHeelMeshes
	for scale := 4.0; scale <= 32; scale *= 2 {
		if plane == nil {
			eval, err = booleaneval.Evaluate(ctx, op, target, tool, mesher, scale,
				p.LoftFacesToe.CapFace, coneFace)
		} else {
			// The tangent cap touches the cone, so no pair is exempted. The
			// tooth enters the ordinary gate with an exact prism glued to its
			// end cap (section 11 of the design; "Two-cone continuation").
			meshes = &TangentHeelMeshes{Base: mesher, Target: target, Toe: p.LoftFacesToe, Plane: *plane}
			eval, err = booleaneval.Evaluate(ctx, op, target, tool, meshes, scale, nil, nil)
		}
		if err == nil {
			break
		}
		var expected *meshbool.BooleanExpectedError
		if scale == 32 || !errors.As(err, &expected) ||
			expected.Kind != meshbool.BooleanExpectedStaging || expected.Operand != 0 {
			return nil, true, err
		}
	}
	if meshes != nil {
		if err := meshes.ClearsResult(ctx, eval.Payload.Verts); err != nil {
			return nil, true, err
		}
	}
	// The cap and wall have no held contact. The mesh result must exclude
	// the whole cap, agreeing with the exact proof that it is outside.
	if slices.Contains(eval.Payload.Src, capSource) {
		return nil, true, meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact,
			fmt.Errorf("%w: the held cone retains a cap certified outside its true surface", decaderr.ErrUnsupported))
	}
	if plane != nil {
		// The tangent trim record needs both tool faces as single result
		// patches. Refuse before the body is built, not after.
		unique, err := facetedrecord.UniqueSourcePatches(ctx, eval.Payload, eval.Audit.Adj, toeCapSource, heelCapSource)
		if err != nil {
			return nil, true, err
		}
		if !unique {
			return nil, true, meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact,
				fmt.Errorf("%w: the tangent heel result has no single toe or heel tool face", decaderr.ErrUnsupported))
		}
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(eval.Volume),
		Exactness: measurement.ExactnessOf(eval.VolumeBound), Bound: units.CubicMillimeters(eval.VolumeBound)}
	body, err := facetedrecord.BuildWithProof(ctx, env, eval.Payload, eval.Audit, volume, eval.VolumeRat)
	if err != nil {
		return nil, true, err
	}
	result, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return body, true, nil
	}
	if revolvecontact.UniqueResultSourceFace(body, toeCapSource) == nil ||
		revolvecontact.UniqueResultSourceFace(body, heelCapSource) == nil {
		return body, true, nil
	}
	result.LoftFacesTrim = &facetedrecord.LoftFacesTrimRecord{
		Base: p.LoftFacesToe.Base, Source: p.LoftFacesToe.Cap.Source,
		Maps:         append([]loftface.Map(nil), p.LoftFacesToe.Cap.Maps...),
		Bounds:       append([]facetedrecord.LoftFaceCapBounds(nil), p.LoftFacesToe.Cap.Bounds...),
		StationGroup: append([]int(nil), p.LoftFacesToe.Cap.StationGroup...),
		Toe:          p.LoftFacesToe.Toe, Heel: cone,
		ToeTool: p.LoftFacesToe.ToeTool, HeelTool: tool,
		ToeCapGroup: toeCapSource, HeelCapGroup: heelCapSource,
		Tangent: p.LoftFacesToe.Cap.Tangent,
	}
	body.Payload = result
	return body, true, nil
}

// TangentHeelMeshes supplies the heel Intersect of a cone-tangent tooth with
// the tooth's mesh extended by an exact prism glued to its end cap
// (loftface.CapExtension). The cone is meshed as usual. Because the proxy is
// restating, the restated chain-depth gate applies to it, and no cap/cone
// face pair is exempted from the proximity gate. It records the meshes it
// hands out so Gate E5 can check them after the Boolean. Use one value per
// Boolean attempt.
type TangentHeelMeshes struct {
	// Base meshes each operand as the ordinary Boolean does.
	Base   booleaneval.OperandMesher
	Target *topo.Body
	Toe    *facetedrecord.LoftFacesToeRecord
	Plane  loftface.TangentPlane

	Slab      loftface.Slab
	hasSlab   bool
	ConeVerts []r3.Vec
	ConeTris  [][3]int

	// PairTol, Proxy and Cone are the tolerance and the two meshes the Boolean
	// read. Tests rebuild the composed bounds from them.
	PairTol     float64
	Proxy, Cone *topo.Mesh
}

func (m *TangentHeelMeshes) TessellateOperand(ctx context.Context, node *topo.Body, pairTol float64) (*topo.Mesh, bool, error) {
	mesh, restating, err := m.Base.TessellateOperand(ctx, node, pairTol)
	if err != nil {
		return nil, restating, err
	}
	if node != m.Target {
		m.ConeVerts, m.ConeTris, m.Cone = mesh.Vertices, mesh.Triangles, mesh
		return mesh, restating, nil
	}
	proxyNode, slab, err := revolveloftfaces.ExtendTangentCap(ctx, mesh, m.Toe, m.Plane)
	if err != nil {
		return nil, false, err
	}
	proxy := *proxyNode
	m.Slab, m.hasSlab, m.PairTol, m.Proxy = slab, true, pairTol, &proxy
	return &proxy, restating, nil
}

// ClearsResult runs the held-only part of Gate E5 on the meshes the last
// Boolean attempt used, then the post-Boolean defence on the result vertices.
// Any failure wraps decaderr.ErrUnsupported (or is a canceled context).
func (m *TangentHeelMeshes) ClearsResult(ctx context.Context, result []r3.Vec) error {
	if !m.hasSlab || m.ConeTris == nil {
		return fmt.Errorf("%w: the tangent heel Boolean recorded no operand meshes", decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := loftface.ExtensionClearsCone(m.Slab, m.ConeVerts, m.ConeTris, m.Plane, budget); err != nil {
		return err
	}
	depth, ok := loftface.ConeDepth(m.ConeVerts, m.Plane)
	if !ok {
		return fmt.Errorf("%w: the held heel cone has no usable vertex", decaderr.ErrUnsupported)
	}
	if depth.Cmp(m.Slab.Eps) < 0 {
		depth = m.Slab.Eps
	}
	if loftface.ResultBeyondSlab(result, m.Plane, depth) {
		return fmt.Errorf("%w: the tangent heel result has a vertex beyond the end-cap slab", decaderr.ErrUnsupported)
	}
	return nil
}

func orderedConeLimits(toe, heel *facetedrecord.PointConeLimit, base facetedrecord.Payload) bool {
	return pointcone.ToeHeelOrdered(toe.Apex, toe.Slope, heel.Apex, heel.Slope,
		base.Verts, base.MeshBound)
}
