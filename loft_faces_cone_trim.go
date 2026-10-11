package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The original LoftFaces proof authenticates this map over source.sourceBox.
// A later Boolean may use it only after proving that its far cap survived.
type loftFacesCapRecord struct {
	source       *pointSectionRecord
	maps         []loftFaceMap
	bounds       []loftFaceCapBounds
	stationGroup []int
	final        loftFaceMap
	group        int
}

// This record is issued to one unplaced Cut of the original LoftFaces mesh.
// Placement drops it, and the following Intersect does not copy it.
type loftFacesToeRecord struct {
	base       facetedPayload
	cap        *loftFacesCapRecord
	capFace    *Face
	toeCapFace *Face
	toe        *pointConeLimit
}

// loftFacesTrimRecord retains the authenticated source of one unplaced
// LoftFaces tooth after its original toe Cut and heel Intersect.
// The Boolean result remains faceted; this record only admits later
// source-specific proofs after those source identities are checked again.
type loftFacesTrimRecord struct {
	base                      facetedPayload
	source                    *pointSectionRecord
	maps                      []loftFaceMap
	bounds                    []loftFaceCapBounds
	stationGroup              []int
	toe, heel                 *pointConeLimit
	toeCapGroup, heelCapGroup int
}

func uniqueConeFace(body *Body) (*Face, bool) {
	var coneFace *Face
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(Cone); !ok {
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
	p, ok := body.payload.(facetedPayload)
	if !ok || source < 0 || len(p.faceOf) != len(p.src) {
		return nil
	}
	faces := body.Faces()
	var result *Face
	for i, group := range p.src {
		if group != source {
			continue
		}
		faceIndex := p.faceOf[i]
		if faceIndex < 0 || faceIndex >= len(faces) {
			return nil
		}
		if result != nil && result != faces[faceIndex] {
			return nil
		}
		result = faces[faceIndex]
	}
	return result
}

func recordLoftFacesConeToe(ctx context.Context, source, tool, result *Body) error {
	base, ok := source.payload.(facetedPayload)
	if !ok || !base.loftFacesPrimitive || base.loftFacesCap == nil || base.xform != r3.Identity() {
		return nil
	}
	cone, ok := exactPointCone(tool)
	if !ok || !pointConeSourceAboveFarCap(base, cone) {
		return nil
	}
	box, err := lfSourceBox(base.loftFacesCap.source)
	if err != nil {
		return err
	}
	if err := lfPositiveDenominator(base.loftFacesCap.final, box); err != nil {
		return err
	}
	if !loftCapOutsideToeCone(base.loftFacesCap.final, box, cone) {
		return nil
	}
	capFace, matched, err := unchangedLoftCap(ctx, source, result, base, base.loftFacesCap.group)
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
	p, ok := result.payload.(facetedPayload)
	if !ok {
		return nil
	}
	p.loftFacesToe = &loftFacesToeRecord{base: base, cap: base.loftFacesCap,
		capFace: capFace, toeCapFace: toeCapFace, toe: cone}
	result.payload = p
	return nil
}

// The source cap lies outside the toe cone on its whole certified rectangle.
// Linear-fractional coordinate extrema occur at its corners because g > 0.
func loftCapOutsideToeCone(m loftFaceMap, box [4]*big.Rat, cone *pointConeLimit) bool {
	corners := lfBoxCorners(box)
	x := lfScalarRange(m, loftFaceRatVec{lfOne(), lfZero(), lfZero()}, corners)
	y := lfScalarRange(m, loftFaceRatVec{lfZero(), lfOne(), lfZero()}, corners)
	z := lfScalarRange(m, loftFaceRatVec{lfZero(), lfZero(), lfOne()}, corners)
	if x.lo.Cmp(cone.far) <= 0 {
		return false
	}
	d := lfIntervalSub(loftFaceInterval{cone.apex, cone.apex}, x)
	radial := lfAdd(loftIntervalSquareMin(y), loftIntervalSquareMin(z))
	margin := lfSub(lfMul(lfMul(cone.slope, cone.slope), radial), loftIntervalSquareMax(d))
	return margin.Sign() > 0
}

func loftIntervalSquareMin(i loftFaceInterval) *big.Rat {
	if i.lo.Sign() <= 0 && i.hi.Sign() >= 0 {
		return lfZero()
	}
	a, b := lfMul(i.lo, i.lo), lfMul(i.hi, i.hi)
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

func loftIntervalSquareMax(i loftFaceInterval) *big.Rat {
	a, b := lfMul(i.lo, i.lo), lfMul(i.hi, i.hi)
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}

type loftCapTriangle [3]r3.Vec

func loftCapTriangleOf(v []r3.Vec, t [3]int) loftCapTriangle {
	out := loftCapTriangle{v[t[0]], v[t[1]], v[t[2]]}
	less := func(a, b r3.Vec) bool {
		if a.X != b.X {
			return a.X < b.X
		}
		if a.Y != b.Y {
			return a.Y < b.Y
		}
		return a.Z < b.Z
	}
	if less(out[1], out[0]) {
		out[0], out[1] = out[1], out[0]
	}
	if less(out[2], out[1]) {
		out[1], out[2] = out[2], out[1]
	}
	if less(out[1], out[0]) {
		out[0], out[1] = out[1], out[0]
	}
	return out
}

// A Cut may keep only part of a cap. Require the exact held triangle multiset
// and one unchanged source patch before carrying the cap's true-map proof.
func unchangedLoftCap(ctx context.Context, source, result *Body, base facetedPayload, group int) (*Face, bool, error) {
	if group < 0 || group >= len(base.groups) || len(base.faceOf) != len(base.tris) {
		return nil, false, nil
	}
	sourceFaces, resultFaces := source.Faces(), result.Faces()
	var sourceFace *Face
	want := map[loftCapTriangle]int{}
	for i, tri := range base.tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		if base.src[i] != group {
			continue
		}
		faceIndex := base.faceOf[i]
		if faceIndex < 0 || faceIndex >= len(sourceFaces) {
			return nil, false, nil
		}
		face := sourceFaces[faceIndex]
		if sourceFace != nil && sourceFace != face {
			return nil, false, nil
		}
		sourceFace = face
		want[loftCapTriangleOf(base.verts, tri)]++
	}
	if sourceFace == nil {
		return nil, false, nil
	}
	sourceIndex := -1
	for i, face := range sourceFaces {
		if face == sourceFace {
			sourceIndex = i
			break
		}
	}
	if sourceIndex < 0 {
		return nil, false, nil
	}
	p, ok := result.payload.(facetedPayload)
	if !ok {
		return nil, false, nil
	}
	var resultFace *Face
	count := 0
	for i, tri := range p.tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		if p.src[i] != sourceIndex {
			continue
		}
		faceIndex := p.faceOf[i]
		if faceIndex < 0 || faceIndex >= len(resultFaces) {
			return nil, false, nil
		}
		face := resultFaces[faceIndex]
		if resultFace != nil && resultFace != face {
			return nil, false, nil
		}
		resultFace = face
		key := loftCapTriangleOf(p.verts, tri)
		if want[key] == 0 {
			return nil, false, nil
		}
		want[key]--
		count++
	}
	for _, remaining := range want {
		if remaining != 0 {
			return nil, false, nil
		}
	}
	if count == 0 {
		return nil, false, nil
	}
	return resultFace, true, nil
}

func tryLoftFacesConeHeel(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpIntersect {
		return nil, false, nil
	}
	p, ok := target.payload.(facetedPayload)
	if !ok || p.loftFacesToe == nil || p.xform != r3.Identity() ||
		p.loftFacesToe.capFace == nil || p.loftFacesToe.capFace.body != target {
		return nil, false, nil
	}
	cone, ok := exactPointCone(tool)
	if !ok || cone.far.Cmp(p.loftFacesToe.toe.far) != 0 ||
		!pointConeSourceAboveFarCap(p.loftFacesToe.base, cone) ||
		!orderedLoftFacesConeLimits(p.loftFacesToe.toe, cone, p.loftFacesToe.base) {
		return nil, false, nil
	}
	box, err := lfSourceBox(p.loftFacesToe.cap.source)
	if err != nil {
		return nil, true, asBooleanError(op, err)
	}
	if err := lfPositiveDenominator(p.loftFacesToe.cap.final, box); err != nil {
		return nil, true, asBooleanError(op, err)
	}
	if !loftCapOutsideHeelCone(p.loftFacesToe.cap.final, box, cone) {
		return nil, false, nil
	}
	coneFace, ok := uniqueConeFace(tool)
	if !ok {
		return nil, false, nil
	}
	capSource := -1
	for i, face := range target.Faces() {
		if face == p.loftFacesToe.capFace {
			capSource = i
			break
		}
	}
	if capSource < 0 {
		return nil, false, nil
	}
	toeCapSource := indexOfFace(target.Faces(), p.loftFacesToe.toeCapFace)
	if toeCapSource < 0 {
		return nil, false, nil
	}
	heelCapSource := len(target.Faces()) + indexOfFace(tool.Faces(), coneFace)
	// A larger chord may be needed when the toe Cut's local rim bound is
	// coarse. Retry only its staging refusal; every other gate keeps its
	// verdict. Each attempt proves its own tessellation and result bounds.
	var eval booleanEvaluation
	for scale := 4.0; scale <= 32; scale *= 2 {
		eval, err = evaluateBooleanMeshesWithConeCap(ctx, op, target, tool, pairMeshes{}, scale,
			p.loftFacesToe.capFace, coneFace)
		if err == nil {
			break
		}
		var expected *meshbool.BooleanExpectedError
		if scale == 32 || !errors.As(err, &expected) ||
			expected.Kind != meshbool.BooleanExpectedStaging || expected.Operand != 0 {
			return nil, true, asBooleanError(op, err)
		}
	}
	// The cap and wall have no held contact. The mesh result must exclude
	// the whole cap, agreeing with the exact proof that it is outside.
	if slices.Contains(eval.payload.src, capSource) {
		return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedContact,
			fmt.Errorf("%w: the held cone retains a cap certified outside its true surface", ErrUnsupported)))
	}
	body, err := buildFacetedBodyWithProof(ctx, d, ref, eval.payload, eval.audit, eval.volume, eval.volumeRat)
	if err != nil {
		return nil, true, asBooleanError(op, err)
	}
	if len(p.loftFacesToe.cap.maps) == 9 {
		result, ok := body.payload.(facetedPayload)
		if !ok {
			return body, true, nil
		}
		if uniqueResultSourceFace(body, toeCapSource) == nil ||
			uniqueResultSourceFace(body, heelCapSource) == nil {
			return body, true, nil
		}
		result.loftFacesTrim = &loftFacesTrimRecord{
			base: p.loftFacesToe.base, source: p.loftFacesToe.cap.source,
			maps:         append([]loftFaceMap(nil), p.loftFacesToe.cap.maps...),
			bounds:       append([]loftFaceCapBounds(nil), p.loftFacesToe.cap.bounds...),
			stationGroup: append([]int(nil), p.loftFacesToe.cap.stationGroup...),
			toe:          p.loftFacesToe.toe, heel: cone,
			toeCapGroup: toeCapSource, heelCapGroup: heelCapSource,
		}
		body.payload = result
	}
	return body, true, nil
}

func orderedLoftFacesConeLimits(toe, heel *pointConeLimit, base facetedPayload) bool {
	if len(base.verts) == 0 || proofbound.IsNonFinite(base.meshBound) || base.meshBound < 0 ||
		toe.apex.Cmp(heel.apex) >= 0 || toe.slope.Sign() <= 0 || heel.slope.Sign() <= 0 {
		return false
	}
	minX, maxX := base.verts[0].X, base.verts[0].X
	for _, v := range base.verts {
		if !proofbound.FiniteVec(v) {
			return false
		}
		minX, maxX = min(minX, v.X), max(maxX, v.X)
	}
	lo := lfSub(lfF(minX), lfF(base.meshBound))
	hi := lfAdd(lfF(maxX), lfF(base.meshBound))
	// The heel radius exceeds the toe radius when this expression is
	// positive. It is affine in X, so both interval endpoints suffice.
	margin := func(x *big.Rat) *big.Rat {
		return lfSub(lfMul(toe.slope, lfSub(heel.apex, x)),
			lfMul(heel.slope, lfSub(toe.apex, x)))
	}
	return margin(lo).Sign() > 0 && margin(hi).Sign() > 0
}

// Q is the exact cone pullback after multiplying through the positive map
// denominator. This narrow arm requires Q=A*u²+C*v² with A,C>0 and u
// strictly one-sided throughout the authenticated source rectangle.
func loftCapOutsideHeelCone(m loftFaceMap, box [4]*big.Rat, cone *pointConeLimit) bool {
	if box[1].Sign() >= 0 && box[0].Sign() <= 0 {
		return false
	}
	g := [3]*big.Rat{m.g0, m.gu, m.gv}
	coordinate := func(axis int) [3]*big.Rat {
		return [3]*big.Rat{
			lfAdd(lfMul(m.constant[axis], g[0]), m.q0[axis]),
			lfAdd(lfMul(m.constant[axis], g[1]), m.qu[axis]),
			lfAdd(lfMul(m.constant[axis], g[2]), m.qv[axis]),
		}
	}
	x, y, z := coordinate(0), coordinate(1), coordinate(2)
	d := [3]*big.Rat{}
	for i := range d {
		d[i] = lfSub(lfMul(cone.apex, g[i]), x[i])
	}
	quad := func(v [3]*big.Rat) [6]*big.Rat {
		return [6]*big.Rat{lfMul(v[0], v[0]), lfMul(big.NewRat(2, 1), lfMul(v[0], v[1])),
			lfMul(big.NewRat(2, 1), lfMul(v[0], v[2])), lfMul(v[1], v[1]),
			lfMul(big.NewRat(2, 1), lfMul(v[1], v[2])), lfMul(v[2], v[2])}
	}
	y2, z2, d2 := quad(y), quad(z), quad(d)
	k2 := lfMul(cone.slope, cone.slope)
	var q [6]*big.Rat
	for i := range q {
		q[i] = lfSub(lfMul(k2, lfAdd(y2[i], z2[i])), d2[i])
	}
	return q[0].Sign() == 0 && q[1].Sign() == 0 && q[2].Sign() == 0 &&
		q[4].Sign() == 0 && q[3].Sign() > 0 && q[5].Sign() > 0
}

// The true cap and cone are disjoint by the source-map proof. Require the
// held facets to be disjoint too before skipping their proximity refusal.
func disjointConeCapFacets(ctx context.Context, a, b *meshbool.BoolMesh,
	capFacets, cone []int, memo *meshbool.ContactMemo) error {
	budget := proofbound.NewWorkBudget(ctx)
	for _, i := range capFacets {
		for _, j := range cone {
			if err := budget.Step(); err != nil {
				return err
			}
			if !meshbool.BoxesWithin(a.Boxes[i], b.Boxes[j], 0) {
				continue
			}
			contact, err := memo.Classify(i, j)
			if err != nil {
				return err
			}
			if contact.Kind != meshbool.ContactNone {
				return fmt.Errorf("%w: the certified cone cap has a held facet contact", ErrUnsupported)
			}
		}
	}
	return nil
}
