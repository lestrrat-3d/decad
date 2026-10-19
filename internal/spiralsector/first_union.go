package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// FirstUnionAdmission is scoped to one exact blank and its first
// unplaced cone-trimmed curved tooth. The proxy mesh is temporary: public
// operands and their retained exact source records do not change.
type FirstUnionAdmission struct {
	Source    *spiralFirstJoinSource
	Short     [2][4]ShortCapFamily
	Roles     map[[2]*topo.Face]meshbool.NearPairRole
	Allowed   map[[2]int]bool
	RootBoxes *[2]*RootFlankSourceBoxes
}

// PrepareFirstUnion authenticates source geometry before modifying a
// temporary held operand. Any failed source gate leaves generic Union in
// charge; cancellation and evaluator errors still propagate. It returns the
// temporary held tooth node and its admission, or two nils when the pair is
// not this class. sections are the blank's candidate Revolve root sections.
func PrepareFirstUnion(ctx context.Context, op meshbool.OperationKind,
	sections []facetedrecord.RevolveSectionCertificate, blank, tooth *topo.Body,
	held *topo.Mesh, sourceIDs []int) (*topo.Mesh, *FirstUnionAdmission, error) {
	if ctx == nil || blank == nil || tooth == nil || held == nil ||
		op != meshbool.OpUnion {
		return nil, nil, nil
	}
	if _, ok := ExactBlank(sections); !ok {
		return nil, nil, nil
	}
	payload, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim == nil {
		return nil, nil, nil
	}
	source, ok, err := CertifyFirstJoin(ctx, sections, tooth)
	if err != nil || !ok {
		return nil, nil, err
	}
	roles, short, ok, err := CertifyNearFaceRoles(ctx, blank, tooth, source)
	if err != nil || !ok {
		return nil, nil, err
	}
	proxy, ok, err := OffsetToothMesh(ctx, blank, tooth, held, sourceIDs, source)
	if err != nil || !ok {
		return nil, nil, err
	}
	return proxy, &FirstUnionAdmission{Source: source, Short: short, Roles: roles}, nil
}

// faceGroupOf returns the held group of face in the joint blank-then-tooth
// face order, or -1 when tooth does not hold it.
func faceGroupOf(blank, tooth *topo.Body, face *topo.Face) int {
	for i, live := range topobuild.Faces(tooth) {
		if live == face {
			return len(topobuild.Faces(blank)) + i
		}
	}
	return -1
}

// FirstUnionPairs lists the only source-face switches supported by the
// curved first-join certificate. Full FeatureRef matches bind the source
// flanks to live faces; source group numbers alone are insufficient after the
// two cone trims. faceID maps each operand face to its held group.
func FirstUnionPairs(blank, tooth *topo.Body, admission *FirstUnionAdmission,
	faceID map[*topo.Face]int) (map[[2]int]bool, bool) {
	if blank == nil || tooth == nil || admission == nil || admission.Source == nil {
		return nil, false
	}
	proof := admission.Source
	groups, ok := JoinGroups(blank, tooth, proof)
	if !ok {
		return nil, false
	}
	allowed := map[[2]int]bool{
		{groups.BlankToe, groups.ToothRootToe}:   true,
		{groups.BlankHeel, groups.ToothRootHeel}: true,
	}
	for pair, kind := range admission.Roles {
		if kind != meshbool.NearPairCrossing {
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
		blankCap := groups.BlankToe
		if end == 1 {
			blankCap = groups.BlankHeel
		}
		for index, segment := range [...]int{0, 2, 3, 5} {
			family := admission.Short[end][index]
			if family.Group != segment || family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return nil, false
			}
			origin, found := OriginalSideRef(proof.Tooth, family.Band, segment)
			if !found {
				return nil, false
			}
			face := UniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{blankCap, group}] = true
		}
	}
	for band := 1; band < len(proof.Tooth.Maps)-1; band++ {
		for _, segment := range [...]int{0, 2} {
			origin, found := OriginalSideRef(proof.Tooth, band, segment)
			if !found {
				return nil, false
			}
			face := UniqueFaceWithOrigin(tooth, origin)
			group, present := faceID[face]
			if face == nil || !present {
				return nil, false
			}
			allowed[[2]int{groups.BlankRoot, group}] = true
		}
	}
	return allowed, len(allowed) == 24
}

// ValidateFirstUnion runs after the ordinary stitched-solid audit. The
// source-pair gate permits only named near pairs, and this gate checks every
// resulting switched curve against its complete authenticated source seam.
// The unchanged mesh-Boolean and stitch volume proofs still apply.
func ValidateFirstUnion(ctx context.Context, blank, tooth *topo.Body,
	blankHeld *meshbool.BoolMesh, result *meshbool.StitchedMesh,
	admission *FirstUnionAdmission, radialExtra ...float64) (bool, error) {
	if ctx == nil || blank == nil || tooth == nil || blankHeld == nil ||
		result == nil || admission == nil || admission.Source == nil {
		return false, nil
	}
	proof := admission.Source
	groups, ok := JoinGroups(blank, tooth, proof)
	if !ok || proof.HeelSeam == nil {
		return false, nil
	}
	// The complete original Revolve cap faces are radial graphs. A bound on
	// their actual held facets then applies to every stitched subedge, with
	// the stitch displacement charged by the long-seam helpers.
	var radial [2]float64
	for end, edge := range [...]int{4, 2} {
		graph, err := HeldBlankCellGraph(ctx, blank, blankHeld, proof.Blank, edge)
		if err != nil || !graph {
			return false, err
		}
		bound, accepted, err := HeldBlankRadialAllowance(ctx, blank, blankHeld, proof.Blank, edge)
		if err != nil || !accepted {
			return false, err
		}
		radial[end] = bound
		if len(radialExtra) == 1 {
			radial[end] = proofbound.AbsSumUpper(radial[end], radialExtra[0])
		}
	}
	if _, ok, err := heldShortCapProof(ctx, result, blank, tooth,
		proof, admission.Short); err != nil || !ok {
		return false, err
	}
	if _, ok, err := HeldHeelSeamProof(ctx, result,
		groups.BlankHeel, groups.ToothRootHeel, proof.HeelSeam, radial[1]); err != nil || !ok {
		return false, err
	}
	toeSeam, ok, err := CertifySpiralToeRootSeam(ctx, proof)
	if err != nil || !ok {
		return false, err
	}
	if _, ok, err := HeldToeSeamProof(ctx, result,
		groups.BlankToe, groups.ToothRootToe, toeSeam, radial[0]); err != nil || !ok {
		return false, err
	}
	for sourceSide, segment := range [...]int{0, 2} {
		origin, found := OriginalSideRef(proof.Tooth,
			proof.Junctions[sourceSide][0].Band, segment)
		if !found {
			return false, nil
		}
		face := UniqueFaceWithOrigin(tooth, origin)
		if face == nil {
			return false, nil
		}
		faceGroup := faceGroupOf(blank, tooth, face)
		if faceGroup < 0 {
			return false, nil
		}
		chain, accepted, err := HeldHeelSwitch(ctx, result, groups.BlankRoot, faceGroup)
		if err != nil || !accepted {
			return false, err
		}
		ordered := make([]r3.Vec, len(chain.Vertices))
		for i, vertex := range chain.Vertices {
			ordered[i] = result.Verts[vertex]
		}
		var prepared []*RootFlankSourceBoxes
		if admission.RootBoxes != nil {
			prepared = append(prepared, admission.RootBoxes[sourceSide])
		}
		if _, accepted, err := RootFlankWholeBound(ctx, proof,
			sourceSide, ordered, chain.MinBeta, prepared...); err != nil || !accepted {
			return false, err
		}
	}
	return ctx.Err() == nil, ctx.Err()
}

// JoinAreaAllowance uses the whole operand boundary areas as a
// deliberately coarse ceiling on the true Union boundary area. This lets a
// source-specific radial patch proof state a smaller boundary displacement
// without pretending its facet areas follow the ordinary affine-corner
// construction. The Union boundary is a subset of the two operand boundaries.
func JoinAreaAllowance(ctx context.Context, a, b *topo.Body,
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
	for _, body := range [...]*topo.Body{a, b} {
		value, err := body.Area.Value.In(units.SquareMillimeter)
		if err != nil {
			return 0, false, err
		}
		bound, err := body.Area.Bound.In(units.SquareMillimeter)
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

const firstJoinCapOffset = 0.032 // millimetres; every actual move is charged below

// OffsetToothMesh makes a temporary held operand for the authenticated
// first spiral join. The exact source solid remains the original tooth. Every
// moved vertex changes only Y; its incident side facets follow the same indexed
// mesh and are audited again before the caller can use the proxy in a Boolean.
// It returns a new mesh node that shares original's triangles and source faces
// and owns its vertices, vertex bounds and face bounds; the caller gives it a
// handle.
//
// The old and new boundary surfaces agree outside the swept changed facets.
// Their solids can differ only inside those sweeps, each of which lies above
// the same exact XZ triangle with vertical thickness at most maxShift. This
// proves the symmetric-difference charge even if an intermediate surface
// during the linear Y motion crosses itself.
func OffsetToothMesh(ctx context.Context, blank, tooth *topo.Body, original *topo.Mesh,
	src []int, proof *spiralFirstJoinSource) (*topo.Mesh, bool, error) {
	if ctx == nil || blank == nil || tooth == nil || original == nil || proof == nil ||
		proof.Tooth == nil || !original.BoundaryOK || !original.SymDiffOK ||
		len(src) != len(original.Triangles) || len(original.Vertices) == 0 ||
		len(original.Vertices) > 1<<16 || len(original.Triangles) > 1<<17 ||
		len(original.VertexBound) != len(original.Vertices) ||
		len(original.Source) != len(original.Triangles) {
		return nil, false, nil
	}
	p, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesTrim != proof.Tooth {
		return nil, false, nil
	}
	toeFace := TrimFaceGroup(tooth, proof.Tooth.ToeCapGroup)
	heelFace := TrimFaceGroup(tooth, proof.Tooth.HeelCapGroup)
	if toeFace < 0 || heelFace < 0 || toeFace == heelFace {
		return nil, false, nil
	}
	blankFaces := len(topobuild.Faces(blank))
	toeGroup, heelGroup := blankFaces+toeFace, blankFaces+heelFace
	move := make([]int8, len(original.Vertices))
	var capFacets [2]int
	for i, tri := range original.Triangles {
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
		len(original.FaceBound) == 0 {
		return nil, false, nil
	}
	next := new(topo.Mesh)
	*next = *original
	next.Vertices = append([]r3.Vec(nil), original.Vertices...)
	next.VertexBound = append([]float64(nil), original.VertexBound...)
	next.FaceBound = make(map[*topo.Face]float64, len(original.FaceBound))
	for face, bound := range original.FaceBound {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		next.FaceBound[face] = bound
	}
	maxShift := new(big.Rat)
	moved := 0
	for i, side := range move {
		if side == 0 {
			continue
		}
		old := original.Vertices[i]
		if !proofbound.FiniteVec(old) || old.Y <= 0 {
			return nil, false, nil
		}
		next.Vertices[i].Y += float64(side) * firstJoinCapOffset
		if !proofbound.FiniteVec(next.Vertices[i]) || next.Vertices[i].Y <= 0 {
			return nil, false, nil
		}
		actual := seamRatAbs(seamRatSub(seamRatF(next.Vertices[i].Y), seamRatF(old.Y)))
		if actual.Cmp(maxShift) > 0 {
			maxShift = actual
		}
		shiftUp := proofbound.RatFloatUp(actual)
		if proofbound.IsNonFinite(shiftUp) || proofbound.IsNonFinite(next.VertexBound[i]) ||
			next.VertexBound[i] < 0 {
			return nil, false, nil
		}
		next.VertexBound[i] = proofbound.AbsSumUpper(next.VertexBound[i], shiftUp)
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
	for i, tri := range original.Triangles {
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
			bound := original.VertexBound[vertex]
			if proofbound.IsNonFinite(bound) || bound < 0 {
				return nil, false, nil
			}
			if bound > original.VertexBound[largest] {
				largest = vertex
			}
		}
		required := proofbound.AbsSumUpper(original.VertexBound[largest], maxShiftUp)
		if proofbound.IsNonFinite(required) {
			return nil, false, nil
		}
		next.VertexBound[largest] = max(next.VertexBound[largest], required)
		face := original.Source[i]
		bound, present := original.FaceBound[face]
		if !present {
			return nil, false, nil
		}
		if !seenFace[face] {
			next.FaceBound[face] = proofbound.AbsSumUpper(bound, maxShiftUp)
			seenFace[face] = true
		}
		a, b, c := original.Vertices[tri[0]], original.Vertices[tri[1]], original.Vertices[tri[2]]
		cross := seamRatSub(seamRatMul(seamRatSub(seamRatF(b.X), seamRatF(a.X)), seamRatSub(seamRatF(c.Z), seamRatF(a.Z))),
			seamRatMul(seamRatSub(seamRatF(b.Z), seamRatF(a.Z)), seamRatSub(seamRatF(c.X), seamRatF(a.X))))
		areaXZ.Add(areaXZ, seamRatAbs(cross))
	}
	areaXZ.Quo(areaXZ, big.NewRat(2, 1))
	volumeCharge := proofbound.RatFloatUp(seamRatMul(maxShift, areaXZ))
	if proofbound.IsNonFinite(volumeCharge) {
		return nil, false, nil
	}
	next.VolSymDiff = proofbound.AbsSumUpper(original.VolSymDiff, volumeCharge)
	next.AreaSlack = proofbound.AbsSumUpper(original.AreaSlack,
		facetproof.MeshAreaUpper(original.Vertices, changed),
		facetproof.MeshAreaUpper(next.Vertices, changed))
	next.CoordBound = proofbound.AbsSumUpper(original.CoordBound, maxShiftUp)
	for _, bound := range next.FaceBound {
		next.Bound = max(next.Bound, bound)
	}
	if proofbound.IsNonFinite(next.VolSymDiff) || proofbound.IsNonFinite(next.AreaSlack) ||
		proofbound.IsNonFinite(next.Bound) || proofbound.IsNonFinite(next.CoordBound) {
		return nil, false, nil
	}
	if err := massmoment.AuditMesh(ctx, next.Vertices, next.Triangles, false); err != nil {
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

// heldShortCapProof compares every complete short cap/flank or
// cap/connector switch with its authenticated source crossing. Global held
// group ids come from the original two operands' face order, not a gallery
// layout. Exact source-cell and held-edge boxes cover both curves.
func heldShortCapProof(ctx context.Context, held *meshbool.StitchedMesh,
	blank, tooth *topo.Body, proof *spiralFirstJoinSource,
	short [2][4]ShortCapFamily) (float64, bool, error) {
	if ctx == nil || held == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Tooth == nil {
		return 0, false, nil
	}
	groups, ok := JoinGroups(blank, tooth, proof)
	if !ok {
		return 0, false, nil
	}
	roles := [...]int{0, 2, 3, 5}
	maxDistance := 0.0
	// Every switch below reads the same unchanged held mesh, so its edge
	// table is built once, at the first switch, and reused within this call.
	var edges map[HeldEdge]HeldEdgeOwners
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		blankCap := groups.BlankToe
		if end == 1 {
			blankCap = groups.BlankHeel
		}
		for index, role := range roles {
			family := short[end][index]
			if family.Group != role || len(family.Pieces) != 1 ||
				family.Band < 0 || family.Band >= len(proof.Tooth.Maps)-1 ||
				family.Band != proof.Junctions[0][end].Band ||
				family.Band != proof.Junctions[1][end].Band {
				return 0, false, nil
			}
			origin, found := OriginalSideRef(proof.Tooth, family.Band, role)
			if !found {
				return 0, false, nil
			}
			face := UniqueFaceWithOrigin(tooth, origin)
			if face == nil {
				return 0, false, nil
			}
			toothFace := faceGroupOf(blank, tooth, face)
			if toothFace < 0 {
				return 0, false, nil
			}
			if edges == nil {
				built, ok, err := HeldEdges(ctx, held)
				if err != nil || !ok {
					return 0, false, err
				}
				edges = built
			}
			chain, ok, err := HeldSwitchFrom(ctx, held, edges, blankCap, toothFace, false)
			if err != nil || !ok {
				return 0, false, err
			}
			ordered := make([]r3.Vec, len(chain.Vertices))
			for i, vertex := range chain.Vertices {
				ordered[i] = held.Verts[vertex]
			}
			distance, accepted, err := ShortHeldBound(ctx, proof, family, ordered, chain.MinBeta)
			if err != nil || !accepted {
				return distance, false, err
			}
			maxDistance = max(maxDistance, distance)
		}
	}
	return maxDistance, true, nil
}
