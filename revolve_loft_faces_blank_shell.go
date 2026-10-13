package decad

import (
	"context"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A source-aligned blank shell is a closed construction piece. Its face bound
// charges angular chording and vertex movement; it is not a Union result.
type revolveLoftFacesBlankSeamShell struct {
	mesh         joinedMesh
	vertexBounds []float64
	seam         []int
	chordBound   float64
	faceBound    float64
}

type revolveLoftFacesRay struct {
	y, z      *big.Rat
	seamIndex int
}

// This builds every authenticated Revolve wall and end cap on one angular
// grid. One whole-arc source contact splits its finite meridian side. Source
// cells select the ring; held output edges never supply a crossing location.
func revolveLoftFacesBlankShellAtArc(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	sectionIndex, cellIndex int,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.tooth == nil || manifest == nil ||
		sectionIndex < 0 || sectionIndex >= len(proof.section.spans) ||
		sectionIndex >= len(manifest.sideOutcome) ||
		manifest.sideOutcome[sectionIndex] != revolveLoftFacesSideSourceCandidate ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false, nil
	}
	payload, payloadOK := proof.tooth.payload.(facetedPayload)
	if !payloadOK || payload.loftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) {
		return nil, false, nil
	}
	census, currentOK, err := revolveLoftFacesCensus(ctx, proof)
	if err != nil || !currentOK {
		return nil, false, err
	}
	current := census.manifest
	if !revolveLoftFacesSameShellManifest(manifest, current) {
		return nil, false, nil
	}
	manifest = current
	section := proof.section.spans[sectionIndex]
	if section.face == nil || section.start.U == section.end.U ||
		section.start.V <= 0 || section.end.V <= 0 {
		return nil, false, nil
	}
	var side *revolveLoftFacesNonverticalSide
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			if side != nil {
				return nil, false, nil
			}
			side = &manifest.nonvertical[i]
		}
	}
	if side == nil || side.face != section.face ||
		!slices.Equal(side.origins, section.origins) ||
		cellIndex < 0 || cellIndex >= len(side.cells) {
		return nil, false, nil
	}
	cell := side.cells[cellIndex]
	mapped := cell.mapped
	if mapped.side != revolveLoftFacesJointContact ||
		mapped.jointKind != revolveLoftFacesWholeArcContact || mapped.jointArc == nil ||
		mapped.tLo == nil || mapped.tHi == nil || mapped.tLoOpen || mapped.tHiOpen ||
		mapped.segment < 0 || mapped.segment >= len(proof.trim.source.source.entities) ||
		proof.trim.source.source.entities[mapped.segment] != mapped.jointArc ||
		mapped.band < 0 || mapped.band+1 >= len(proof.trim.maps) {
		return nil, false, nil
	}
	if mapped.band >= len(proof.sourceRefs) ||
		mapped.segment >= len(proof.sourceRefs[mapped.band]) ||
		cell.ref != proof.sourceRefs[mapped.band][mapped.segment] {
		return nil, false, nil
	}
	liveFace, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, cell.ref)
	if !unique || liveFace == nil || liveFace != cell.face {
		return nil, false, nil
	}
	arcCells, certified, err := revolveLoftFacesArcSpanCells(ctx, proof.trim,
		section, mapped.segment)
	if err != nil || !certified {
		return nil, false, err
	}
	matched := false
	for _, source := range arcCells {
		matched = matched || revolveLoftFacesSameArcContact(source, mapped)
	}
	if !matched {
		return nil, false, nil
	}
	stations, ok := revolveLoftFacesCompleteArcRun(proof.trim, mapped.segment)
	if !ok {
		return nil, false, nil
	}
	seam, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(ctx, proof.blank, proof.tooth,
		sectionIndex, mapped.segment, mapped.band,
		loftFaceInterval{lo: mapped.tLo, hi: mapped.tHi}, stations)
	if err != nil || !ok {
		return nil, false, err
	}
	x := seam[0].exact[0]
	for _, vertex := range seam[1:] {
		if vertex.exact[0].Cmp(x) != 0 {
			return nil, false, nil
		}
	}
	m, b := revolveLoftFacesMeridian(section)
	radius := lfAdd(lfMul(m, x), b)
	if radius.Sign() <= 0 {
		return nil, false, nil
	}
	maxRadius := 0.0
	for _, span := range proof.section.spans {
		maxRadius = max(maxRadius, span.start.V, span.end.V)
	}
	if proof.section.axisSpan < 0 || !proofbound.FiniteVec(r3.Vec{X: maxRadius}) ||
		maxRadius <= 0 || maxRadius > 0.04*1024*1024 {
		return nil, false, nil
	}
	// A quarter-circle rational parameter advances by at most 2/n radians.
	// Its maximum sagitta is at most radius/(2*n*n).
	n := max(16, int(math.Ceil(math.Sqrt(maxRadius/0.04))))
	if n > 1024 {
		return nil, false, nil
	}
	rays := revolveLoftFacesShellRays(n, seam)
	if len(rays) < 4 || len(rays) > 4*n+len(seam) ||
		len(rays)*len(proof.section.spans) > 1<<16 {
		return nil, false, nil
	}
	out := &revolveLoftFacesBlankSeamShell{chordBound: maxRadius / (2 * float64(n*n)),
		seam: make([]int, len(seam))}
	for i := range out.seam {
		out.seam[i] = -1
	}
	group := make([]int, len(proof.section.spans))
	for i, span := range proof.section.spans {
		group[i] = -1
		if span.face == nil {
			if i != proof.section.axisSpan {
				return nil, false, nil
			}
			continue
		}
		if i >= len(manifest.sideOutcome) {
			return nil, false, nil
		}
		current, err := revolveLoftFacesShellSideCurrent(ctx, proof, manifest, i, span)
		if err != nil || !current {
			return nil, false, err
		}
		group[i] = len(out.mesh.groups)
		out.mesh.groups = append(out.mesh.groups,
			facetGroup{origins: append([]FeatureRef(nil), span.origins...), planar: span.face.isPlanar()})
	}
	appendPoint := func(point r3.Vec, bound float64) int {
		index := len(out.mesh.verts)
		out.mesh.verts = append(out.mesh.verts, point)
		out.vertexBounds = append(out.vertexBounds, bound)
		return index
	}
	makeRing := func(x, radius *big.Rat, middle bool) ([]int, bool) {
		if radius.Sign() < 0 {
			return nil, false
		}
		if radius.Sign() == 0 {
			point, bound, valid := revolveLoftFacesHeldPoint(loftFaceRatVec{x, lfZero(), lfZero()})
			if !valid {
				return nil, false
			}
			return []int{appendPoint(point, bound)}, true
		}
		ring := make([]int, len(rays))
		for j, ray := range rays {
			if middle && ray.seamIndex >= 0 {
				vertex := seam[ray.seamIndex]
				ring[j] = appendPoint(vertex.point, vertex.blankBound)
				out.seam[ray.seamIndex] = ring[j]
				continue
			}
			point, bound, valid := revolveLoftFacesMeridianRayPoint(
				[2]*big.Rat{ray.y, ray.z}, x, radius)
			if !valid {
				return nil, false
			}
			ring[j] = appendPoint(point, bound)
		}
		return ring, true
	}
	rings := make([][]int, len(proof.section.spans))
	for i, span := range proof.section.spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		var valid bool
		rings[i], valid = makeRing(lfF(span.start.U), lfF(span.start.V), false)
		if !valid {
			return nil, false, nil
		}
	}
	middle, valid := makeRing(x, radius, true)
	if !valid {
		return nil, false, nil
	}
	addBand := func(start, end []int, source int) bool {
		if len(start) != len(rays) && len(start) != 1 ||
			len(end) != len(rays) && len(end) != 1 || source < 0 {
			return false
		}
		for j := range rays {
			next := (j + 1) % len(rays)
			switch {
			case len(start) == 1 && len(end) == 1:
				return false
			case len(start) == 1:
				out.mesh.add([3]int{start[0], end[j], end[next]}, source)
			case len(end) == 1:
				out.mesh.add([3]int{start[j], end[0], start[next]}, source)
			default:
				out.mesh.add([3]int{start[j], end[j], end[next]}, source)
				out.mesh.add([3]int{start[j], end[next], start[next]}, source)
			}
		}
		return true
	}
	for i := range proof.section.spans {
		if group[i] < 0 {
			continue
		}
		start, end := rings[i], rings[(i+1)%len(rings)]
		if i == sectionIndex {
			if !addBand(start, middle, group[i]) || !addBand(middle, end, group[i]) {
				return nil, false, nil
			}
			continue
		}
		if !addBand(start, end, group[i]) {
			return nil, false, nil
		}
	}
	for _, index := range out.seam {
		if index < 0 {
			return nil, false, nil
		}
	}
	maxVertexBound := 0.0
	for _, bound := range out.vertexBounds {
		if proofbound.IsNonFinite(bound) || bound < 0 {
			return nil, false, nil
		}
		maxVertexBound = max(maxVertexBound, bound)
	}
	// The ideal meridian quads have zero twist because every meridian edge
	// is straight and all rings use the same rays. The end caps are planar.
	// Angular chord sagitta and vertex movement bound each complete face.
	out.faceBound = proofbound.AbsSumUpper(out.chordBound, maxVertexBound)
	if proofbound.IsNonFinite(out.faceBound) {
		return nil, false, nil
	}
	if err := orientJoinedMesh(&out.mesh); err != nil {
		return nil, false, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		out.mesh.verts, out.mesh.tris, len(out.mesh.tris), 1<<20); err != nil {
		return nil, false, err
	}
	if !revolveLoftFacesBlankShellEulerTwo(&out.mesh) {
		return nil, false, nil
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, out.mesh.verts, out.mesh.tris)
	if err != nil {
		return nil, false, err
	}
	if audit == nil || len(audit.Members) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, false, nil
	}
	return out, true, nil
}

func revolveLoftFacesBlankShellEulerTwo(mesh *joinedMesh) bool {
	if mesh == nil || len(mesh.verts) == 0 || len(mesh.tris) == 0 {
		return false
	}
	edges := make(map[[2]int]struct{}, 3*len(mesh.tris)/2)
	used := make([]bool, len(mesh.verts))
	for _, tri := range mesh.tris {
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(mesh.verts) || b >= len(mesh.verts) || a == b {
				return false
			}
			used[a], used[b] = true, true
			edges[[2]int{min(a, b), max(a, b)}] = struct{}{}
		}
	}
	for _, referenced := range used {
		if !referenced {
			return false
		}
	}
	return len(mesh.verts)-len(edges)+len(mesh.tris) == 2
}

func revolveLoftFacesSameArcContact(a, b revolveLoftFacesMappedCell) bool {
	return a.segment == b.segment && a.span == b.span && a.band == b.band &&
		a.side == revolveLoftFacesJointContact && b.side == a.side &&
		a.jointKind == revolveLoftFacesWholeArcContact && b.jointKind == a.jointKind &&
		a.jointArc == b.jointArc && a.jointPoint == b.jointPoint &&
		a.sLo != nil && b.sLo != nil && a.sLo.Cmp(b.sLo) == 0 &&
		a.sHi != nil && b.sHi != nil && a.sHi.Cmp(b.sHi) == 0 &&
		a.tLo != nil && b.tLo != nil && a.tLo.Cmp(b.tLo) == 0 &&
		a.tHi != nil && b.tHi != nil && a.tHi.Cmp(b.tHi) == 0 &&
		a.tLoOpen == b.tLoOpen && a.tHiOpen == b.tHiOpen
}

// A candidate must carry its retained source cells; a marked outcome alone
// cannot turn an empty nonvertical face into certified contact.
func revolveLoftFacesShellSideCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	index int, span revolveSectionSpan) (bool, error) {
	switch manifest.sideOutcome[index] {
	case revolveLoftFacesSideSourceCandidate:
		if span.start.U != span.end.U {
			for _, side := range manifest.nonvertical {
				if side.sectionIndex == index {
					return side.face == span.face && len(side.cells) > 0 &&
						slices.Equal(side.origins, span.origins), nil
				}
			}
			return false, nil
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveLoftFacesVerticalContact ||
			side == nil || side.face != span.face || len(side.cells) == 0 {
			return false, err
		}
		for _, entry := range manifest.vertical {
			if entry.sectionIndex == index {
				return entry.outcome == revolveLoftFacesVerticalContact &&
					entry.side != nil && entry.side.face == span.face &&
					len(entry.side.cells) > 0, nil
			}
		}
	case revolveLoftFacesSideMiss:
		if span.start.U != span.end.U {
			return false, nil
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveLoftFacesVerticalMiss || side != nil {
			return false, err
		}
		for _, entry := range manifest.vertical {
			if entry.sectionIndex == index {
				return entry.outcome == revolveLoftFacesVerticalMiss && entry.side == nil, nil
			}
		}
	}
	return false, nil
}

func revolveLoftFacesShellRays(n int, seam []revolveLoftFacesSeamVertex) []revolveLoftFacesRay {
	rays := make([]revolveLoftFacesRay, 0, 4*n+len(seam))
	for quadrant := range 4 {
		for i := range n {
			q := big.NewRat(int64(i), int64(n))
			q2 := lfMul(q, q)
			denominator := lfAdd(lfOne(), q2)
			c := lfDiv(lfSub(lfOne(), q2), denominator)
			s := lfDiv(lfMul(big.NewRat(2, 1), q), denominator)
			ray := revolveLoftFacesRay{seamIndex: -1}
			switch quadrant {
			case 0:
				ray.y, ray.z = c, s
			case 1:
				ray.y, ray.z = lfSub(lfZero(), s), c
			case 2:
				ray.y, ray.z = lfSub(lfZero(), c), lfSub(lfZero(), s)
			case 3:
				ray.y, ray.z = s, lfSub(lfZero(), c)
			}
			rays = append(rays, ray)
		}
	}
	for i, point := range seam {
		rays = append(rays, revolveLoftFacesRay{y: point.exact[1], z: point.exact[2], seamIndex: i})
	}
	sort.Slice(rays, func(i, j int) bool { return revolveLoftFacesRayBefore(rays[i], rays[j]) })
	unique := rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !revolveLoftFacesSameRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		if ray.seamIndex >= 0 {
			if unique[len(unique)-1].seamIndex >= 0 {
				return nil
			}
			unique[len(unique)-1] = ray
		}
	}
	return unique
}

func revolveLoftFacesRayBefore(a, b revolveLoftFacesRay) bool {
	upper := func(ray revolveLoftFacesRay) bool {
		return ray.z.Sign() > 0 || ray.z.Sign() == 0 && ray.y.Sign() >= 0
	}
	if upper(a) != upper(b) {
		return upper(a)
	}
	return lfSub(lfMul(a.y, b.z), lfMul(a.z, b.y)).Sign() > 0
}

func revolveLoftFacesSameRay(a, b revolveLoftFacesRay) bool {
	return lfSub(lfMul(a.y, b.z), lfMul(a.z, b.y)).Sign() == 0 &&
		lfAdd(lfMul(a.y, b.y), lfMul(a.z, b.z)).Sign() > 0
}

func revolveLoftFacesMeridianRayPoint(ray [2]*big.Rat, x, radius *big.Rat) (r3.Vec, float64, bool) {
	if ray[0] == nil || ray[1] == nil || x == nil || radius == nil || radius.Sign() <= 0 {
		return r3.Vec{}, 0, false
	}
	r2 := lfAdd(lfMul(ray[0], ray[0]), lfMul(ray[1], ray[1]))
	stored, _ := r2.Float64()
	if proofbound.IsNonFinite(stored) || stored <= 0 {
		return r3.Vec{}, 0, false
	}
	denominator := math.Sqrt(stored)
	if proofbound.IsNonFinite(denominator) || denominator <= 0 {
		return r3.Vec{}, 0, false
	}
	scale := lfDiv(radius, lfF(denominator))
	exact := loftFaceRatVec{x, lfMul(ray[0], scale), lfMul(ray[1], scale)}
	point, round, ok := revolveLoftFacesHeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	actual := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	residual := proofbound.RatFloatUp(lfDiv(
		lfAbs(lfSub(actual, lfMul(radius, radius))), radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}
