package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// spiralFirstUnionAdmission is scoped to one exact blank and its first
// unplaced cone-trimmed curved tooth. The proxy mesh is temporary: public
// operands and their retained exact source records do not change.
type spiralFirstUnionAdmission struct {
	source    *spiralFirstJoinSource
	short     [2][4]spiralsector.ShortCapFamily
	roles     map[[2]*Face]spiralNearPairKind
	allowed   map[[2]int]bool
	rootBoxes *[2]*spiralsector.RootFlankSourceBoxes
}

// spiralPrepareFirstUnion authenticates source geometry before modifying a
// temporary held operand. Any failed source gate leaves generic Union in
// charge; cancellation and evaluator errors still propagate.
func spiralPrepareFirstUnion(ctx context.Context, op meshbool.OperationKind,
	blank, tooth *Body, held *Mesh, sourceIDs []int) (*Mesh, *spiralFirstUnionAdmission, error) {
	if ctx == nil || blank == nil || tooth == nil || held == nil ||
		op != meshbool.OpUnion {
		return held, nil, nil
	}
	if _, ok := exactSpiralBlank(blank); !ok {
		return held, nil, nil
	}
	payload, ok := tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim == nil {
		return held, nil, nil
	}
	source, ok, err := certifySpiralFirstJoin(ctx, blank, tooth)
	if err != nil || !ok {
		return held, nil, err
	}
	roles, short, ok, err := certifySpiralNearFaceRoles(ctx, blank, tooth, source)
	if err != nil || !ok {
		return held, nil, err
	}
	proxy, ok, err := spiralOffsetToothMesh(ctx, blank, tooth, held, sourceIDs, source)
	if err != nil || !ok {
		return held, nil, err
	}
	return proxy, &spiralFirstUnionAdmission{source: source, short: short,
		roles: roles}, nil
}

// spiralFirstUnionPairs lists the only source-face switches supported by the
// curved first-join certificate. Full FeatureRef matches bind the source
// flanks to live faces; source group numbers alone are insufficient after the
// two cone trims.
func spiralFirstUnionPairs(blank, tooth *Body, admission *spiralFirstUnionAdmission,
	faceID map[*Face]int) (map[[2]int]bool, bool) {
	if blank == nil || tooth == nil || admission == nil || admission.source == nil {
		return nil, false
	}
	proof := admission.source
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return nil, false
	}
	allowed := map[[2]int]bool{
		{groups.blankToe, groups.toothRootToe}:   true,
		{groups.blankHeel, groups.toothRootHeel}: true,
	}
	for pair, kind := range admission.roles {
		if kind != spiralNearPairCrossing {
			continue
		}
		a, aOK := faceID[pair[0]]
		b, bOK := faceID[pair[1]]
		if !aOK || !bOK {
			return nil, false
		}
		allowed[[2]int{a, b}] = true
	}
	for end := range 2 {
		blankCap := groups.blankToe
		if end == 1 {
			blankCap = groups.blankHeel
		}
		for index, segment := range [...]int{0, 2, 3, 5} {
			family := admission.short[end][index]
			if family.Group != segment || family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return nil, false
			}
			origin, found := spiralOriginalSideRef(proof.Tooth, family.Band, segment)
			if !found {
				return nil, false
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{blankCap, group}] = true
		}
	}
	for band := 1; band < len(proof.Tooth.Maps)-1; band++ {
		for _, segment := range [...]int{0, 2} {
			origin, found := spiralOriginalSideRef(proof.Tooth, band, segment)
			if !found {
				return nil, false
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{groups.blankRoot, group}] = true
		}
	}
	return allowed, len(allowed) == 24
}

// spiralValidateFirstUnion runs after the ordinary stitched-solid audit. The
// source-pair gate permits only named near pairs, and this gate checks every
// resulting switched curve against its complete authenticated source seam.
// The unchanged mesh-Boolean and stitch volume proofs still apply.
func spiralValidateFirstUnion(ctx context.Context, blank, tooth *Body,
	blankHeld *meshbool.BoolMesh, result *meshbool.StitchedMesh,
	admission *spiralFirstUnionAdmission, radialExtra ...float64) (bool, error) {
	if ctx == nil || blank == nil || tooth == nil || blankHeld == nil ||
		result == nil || admission == nil || admission.source == nil {
		return false, nil
	}
	proof := admission.source
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok || proof.HeelSeam == nil {
		return false, nil
	}
	// The complete original Revolve cap faces are radial graphs. A bound on
	// their actual held facets then applies to every stitched subedge, with
	// the stitch displacement charged by the long-seam helpers.
	var radial [2]float64
	for end, edge := range [...]int{4, 2} {
		graph, err := spiralHeldBlankCellGraph(ctx, blank, blankHeld, proof.Blank, edge)
		if err != nil || !graph {
			return false, err
		}
		bound, accepted, err := spiralHeldBlankRadialAllowance(ctx,
			blank, blankHeld, proof.Blank, edge)
		if err != nil || !accepted {
			return false, err
		}
		radial[end] = bound
		if len(radialExtra) == 1 {
			radial[end] = proofbound.AbsSumUpper(radial[end], radialExtra[0])
		}
	}
	if _, ok, err := spiralHeldShortCapProof(ctx, result, blank, tooth,
		proof, admission.short); err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralsector.HeldHeelSeamProof(ctx, result,
		groups.blankHeel, groups.toothRootHeel, proof.HeelSeam, radial[1]); err != nil || !ok {
		return false, err
	}
	toeSeam, ok, err := spiralsector.CertifySpiralToeRootSeam(ctx, proof)
	if err != nil || !ok {
		return false, err
	}
	if _, ok, err := spiralsector.HeldToeSeamProof(ctx, result,
		groups.blankToe, groups.toothRootToe, toeSeam, radial[0]); err != nil || !ok {
		return false, err
	}
	for sourceSide, segment := range [...]int{0, 2} {
		origin, found := spiralOriginalSideRef(proof.Tooth,
			proof.Junctions[sourceSide][0].Band, segment)
		if !found {
			return false, nil
		}
		face := spiralUniqueFaceWithOrigin(tooth, origin)
		if face == nil {
			return false, nil
		}
		faceGroup := -1
		for i, live := range tooth.Faces() {
			if live == face {
				faceGroup = len(blank.Faces()) + i
			}
		}
		if faceGroup < 0 {
			return false, nil
		}
		chain, accepted, err := spiralsector.HeldHeelSwitch(ctx, result,
			groups.blankRoot, faceGroup)
		if err != nil || !accepted {
			return false, err
		}
		ordered := make([]r3.Vec, len(chain.Vertices))
		for i, vertex := range chain.Vertices {
			ordered[i] = result.Verts[vertex]
		}
		var prepared []*spiralsector.RootFlankSourceBoxes
		if admission.rootBoxes != nil {
			prepared = append(prepared, admission.rootBoxes[sourceSide])
		}
		if _, accepted, err := spiralsector.RootFlankWholeBound(ctx, proof,
			sourceSide, ordered, chain.MinBeta, prepared...); err != nil || !accepted {
			return false, err
		}
	}
	return ctx.Err() == nil, ctx.Err()
}

// spiralJoinAreaAllowance uses the whole operand boundary areas as a
// deliberately coarse ceiling on the true Union boundary area. This lets a
// source-specific radial patch proof state a smaller boundary displacement
// without pretending its facet areas follow the ordinary affine-corner
// construction. The Union boundary is a subset of the two operand boundaries.
func spiralJoinAreaAllowance(ctx context.Context, a, b *Body,
	held *meshbool.StitchedMesh) (float64, bool, error) {
	if ctx == nil || a == nil || b == nil || held == nil {
		return 0, false, nil
	}
	heldUpper, err := proofbound.PerturbedAreaUpperContext(ctx, held.Verts, held.Tris, 0)
	if err != nil {
		return 0, false, err
	}
	if proofbound.IsNonFinite(heldUpper) {
		return 0, false, nil
	}
	trueUpper := 0.0
	for _, body := range [...]*Body{a, b} {
		area, err := body.Area()
		if err != nil {
			return 0, false, err
		}
		value, err := area.Value.In(units.SquareMillimeter)
		if err != nil {
			return 0, false, err
		}
		bound, err := area.Bound.In(units.SquareMillimeter)
		if err != nil {
			return 0, false, err
		}
		if value < 0 || bound < 0 || proofbound.IsNonFinite(value) || proofbound.IsNonFinite(bound) {
			return 0, false, nil
		}
		trueUpper = proofbound.AbsSumUpper(trueUpper, value, bound)
	}
	allowance := proofbound.AbsSumUpper(heldUpper, trueUpper)
	return allowance, !proofbound.IsNonFinite(allowance), nil
}

const spiralFirstJoinCapOffset = 0.032 // millimetres; every actual move is charged below

// spiralOffsetToothMesh makes a temporary held operand for the authenticated
// first spiral join. The exact source solid remains the original tooth. Every
// moved vertex changes only Y; its incident side facets follow the same indexed
// mesh and are audited again before the caller can use the proxy in a Boolean.
//
// The old and new boundary surfaces agree outside the swept changed facets.
// Their solids can differ only inside those sweeps, each of which lies above
// the same exact XZ triangle with vertical thickness at most maxShift. This
// proves the symmetric-difference charge even if an intermediate surface
// during the linear Y motion crosses itself.
func spiralOffsetToothMesh(ctx context.Context, blank, tooth *Body, original *Mesh,
	src []int, proof *spiralFirstJoinSource) (*Mesh, bool, error) {
	if ctx == nil || blank == nil || tooth == nil || original == nil || proof == nil ||
		proof.Tooth == nil || !original.n.BoundaryOK || !original.n.SymDiffOK ||
		len(src) != len(original.n.Triangles) || len(original.n.Vertices) == 0 ||
		len(original.n.Vertices) > 1<<16 || len(original.n.Triangles) > 1<<17 ||
		len(original.n.VertexBound) != len(original.n.Vertices) ||
		len(original.n.Source) != len(original.n.Triangles) {
		return nil, false, nil
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim != proof.Tooth {
		return nil, false, nil
	}
	toeFace := spiralTrimFaceGroup(tooth, proof.Tooth.ToeCapGroup)
	heelFace := spiralTrimFaceGroup(tooth, proof.Tooth.HeelCapGroup)
	if toeFace < 0 || heelFace < 0 || toeFace == heelFace {
		return nil, false, nil
	}
	toeGroup, heelGroup := len(blank.Faces())+toeFace, len(blank.Faces())+heelFace
	move := make([]int8, len(original.n.Vertices))
	var capFacets [2]int
	for i, tri := range original.n.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		side := int8(0)
		switch src[i] {
		case toeGroup:
			side = -1
			capFacets[0]++
		case heelGroup:
			side = 1
			capFacets[1]++
		}
		for _, vertex := range tri {
			if vertex < 0 || vertex >= len(move) {
				return nil, false, nil
			}
			if side != 0 {
				if move[vertex] != 0 && move[vertex] != side {
					return nil, false, nil
				}
				move[vertex] = side
			}
		}
	}
	if capFacets[0] == 0 || capFacets[1] == 0 ||
		len(original.n.FaceBound) == 0 {
		return nil, false, nil
	}
	next := bornMesh(&Mesh{n: original.n})
	next.n.Vertices = append([]r3.Vec(nil), original.n.Vertices...)
	next.n.VertexBound = append([]float64(nil), original.n.VertexBound...)
	next.n.FaceBound = make(map[*topo.Face]float64, len(original.n.FaceBound))
	for face, bound := range original.n.FaceBound {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		next.n.FaceBound[face] = bound
	}
	maxShift := new(big.Rat)
	moved := 0
	for i, side := range move {
		if side == 0 {
			continue
		}
		old := original.n.Vertices[i]
		if !proofbound.FiniteVec(old) || old.Y <= 0 {
			return nil, false, nil
		}
		next.n.Vertices[i].Y += float64(side) * spiralFirstJoinCapOffset
		if !proofbound.FiniteVec(next.n.Vertices[i]) || next.n.Vertices[i].Y <= 0 {
			return nil, false, nil
		}
		actual := lfAbs(lfSub(lfF(next.n.Vertices[i].Y), lfF(old.Y)))
		if actual.Cmp(maxShift) > 0 {
			maxShift = actual
		}
		shiftUp := proofbound.RatFloatUp(actual)
		if proofbound.IsNonFinite(shiftUp) || proofbound.IsNonFinite(next.n.VertexBound[i]) ||
			next.n.VertexBound[i] < 0 {
			return nil, false, nil
		}
		next.n.VertexBound[i] = proofbound.AbsSumUpper(next.n.VertexBound[i], shiftUp)
		moved++
	}
	if moved == 0 || maxShift.Sign() == 0 {
		return nil, false, nil
	}
	maxShiftUp := proofbound.RatFloatUp(maxShift)
	if proofbound.IsNonFinite(maxShiftUp) {
		return nil, false, nil
	}
	areaXZ := new(big.Rat)
	changed := make([][3]int, 0)
	seenFace := make(map[*topo.Face]bool)
	for i, tri := range original.n.Triangles {
		if move[tri[0]] == 0 && move[tri[1]] == 0 && move[tri[2]] == 0 {
			continue
		}
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		changed = append(changed, tri)
		// The source certificate is per facet. Its old maximum may sit at an
		// unmoved corner, so charging only moved corners can understate the
		// new facet departure read from the maximum of its vertex bounds.
		largest := tri[0]
		for _, vertex := range tri {
			bound := original.n.VertexBound[vertex]
			if proofbound.IsNonFinite(bound) || bound < 0 {
				return nil, false, nil
			}
			if bound > original.n.VertexBound[largest] {
				largest = vertex
			}
		}
		required := proofbound.AbsSumUpper(original.n.VertexBound[largest], maxShiftUp)
		if proofbound.IsNonFinite(required) {
			return nil, false, nil
		}
		next.n.VertexBound[largest] = max(next.n.VertexBound[largest], required)
		face := original.n.Source[i]
		bound, present := original.n.FaceBound[face]
		if !present {
			return nil, false, nil
		}
		if !seenFace[face] {
			next.n.FaceBound[face] = proofbound.AbsSumUpper(bound, maxShiftUp)
			seenFace[face] = true
		}
		a, b, c := original.n.Vertices[tri[0]], original.n.Vertices[tri[1]], original.n.Vertices[tri[2]]
		cross := lfSub(lfMul(lfSub(lfF(b.X), lfF(a.X)), lfSub(lfF(c.Z), lfF(a.Z))),
			lfMul(lfSub(lfF(b.Z), lfF(a.Z)), lfSub(lfF(c.X), lfF(a.X))))
		areaXZ.Add(areaXZ, lfAbs(cross))
	}
	areaXZ.Quo(areaXZ, big.NewRat(2, 1))
	volumeCharge := proofbound.RatFloatUp(lfMul(maxShift, areaXZ))
	if proofbound.IsNonFinite(volumeCharge) {
		return nil, false, nil
	}
	next.n.VolSymDiff = proofbound.AbsSumUpper(original.n.VolSymDiff, volumeCharge)
	next.n.AreaSlack = proofbound.AbsSumUpper(original.n.AreaSlack,
		facetproof.MeshAreaUpper(original.n.Vertices, changed),
		facetproof.MeshAreaUpper(next.n.Vertices, changed))
	next.n.CoordBound = proofbound.AbsSumUpper(original.n.CoordBound, maxShiftUp)
	for _, bound := range next.n.FaceBound {
		next.n.Bound = max(next.n.Bound, bound)
	}
	if proofbound.IsNonFinite(next.n.VolSymDiff) || proofbound.IsNonFinite(next.n.AreaSlack) ||
		proofbound.IsNonFinite(next.n.Bound) || proofbound.IsNonFinite(next.n.CoordBound) {
		return nil, false, nil
	}
	if err := massmoment.AuditMesh(ctx, next.n.Vertices, next.n.Triangles, false); err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	return next, true, nil
}

// spiralHeldShortCapProof compares every complete short cap/flank or
// cap/connector switch with its authenticated source crossing. Global held
// group ids come from the original two operands' face order, not a gallery
// layout. Exact source-cell and held-edge boxes cover both curves.
func spiralHeldShortCapProof(ctx context.Context, held *meshbool.StitchedMesh,
	blank, tooth *Body, proof *spiralFirstJoinSource,
	short [2][4]spiralsector.ShortCapFamily) (float64, bool, error) {
	if ctx == nil || held == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Tooth == nil {
		return 0, false, nil
	}
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return 0, false, nil
	}
	roles := [...]int{0, 2, 3, 5}
	maxDistance := 0.0
	// Every switch below reads the same unchanged held mesh, so its edge
	// table is built once, at the first switch, and reused within this call.
	var edges map[spiralsector.HeldEdge]spiralsector.HeldEdgeOwners
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		blankCap := groups.blankToe
		if end == 1 {
			blankCap = groups.blankHeel
		}
		for index, role := range roles {
			family := short[end][index]
			if family.Group != role || len(family.Pieces) != 1 ||
				family.Band < 0 || family.Band >= len(proof.Tooth.Maps)-1 ||
				family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return 0, false, nil
			}
			origin, found := spiralOriginalSideRef(proof.Tooth, family.Band, role)
			if !found {
				return 0, false, nil
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			if face == nil {
				return 0, false, nil
			}
			toothFace := -1
			for i, live := range tooth.Faces() {
				if live == face {
					toothFace = i
				}
			}
			if toothFace < 0 {
				return 0, false, nil
			}
			if edges == nil {
				built, ok, err := spiralsector.HeldEdges(ctx, held)
				if err != nil || !ok {
					return 0, false, err
				}
				edges = built
			}
			chain, ok, err := spiralsector.HeldSwitchFrom(ctx, held, edges,
				blankCap, len(blank.Faces())+toothFace, false)
			if err != nil || !ok {
				return 0, false, err
			}
			ordered := make([]r3.Vec, len(chain.Vertices))
			for i, vertex := range chain.Vertices {
				ordered[i] = held.Verts[vertex]
			}
			distance, accepted, err := spiralsector.ShortHeldBound(ctx,
				proof, family, ordered, chain.MinBeta)
			if err != nil || !accepted {
				return distance, false, err
			}
			maxDistance = max(maxDistance, distance)
		}
	}
	return maxDistance, true, nil
}
