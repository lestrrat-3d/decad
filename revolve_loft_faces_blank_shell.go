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
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
)

// A source-aligned blank shell is a closed construction piece. Its face bound
// charges angular chording and vertex movement; it is not a Union result.
type revolveLoftFacesBlankSeamShell struct {
	mesh          joinedMesh
	vertexBounds  []float64
	seam          []int
	seamIDs       []int
	seams         [][]int
	seamRunIDs    [][]int
	triangleCells []revolveLoftFacesShellTriangleCell
	sectionRings  [][][]int
	rays          []revolveLoftFacesRay
	rayCount      int
	capRings      []revolveLoftFacesShellCapRing
	chordBound    float64
	faceBound     float64
}

type revolveLoftFacesShellTriangleCell struct {
	section, band, ray, half int
}

type revolveLoftFacesShellSeamSelector struct {
	sectionIndex, cellIndex int
}

type revolveLoftFacesShellCapRing struct {
	sectionIndex     int
	arc              *revolveLoftFacesToolCapOccupiedArc
	vertices         []int
	endpoints        [2]int
	occupied         []int
	endpointProof    [2]revolveLoftFacesCapSeamVertex
	ties             *revolveLoftFacesHeelArcSideTies
	endpointGraphIDs [2]int
	endpointCellIDs  [2]int
}

type revolveLoftFacesShellCut struct {
	sectionIndex int
	x, radius    *big.Rat
	arc          *revolveLoftFacesToolCapOccupiedArc
	seam         bool
	seamRun      int
	vertices     []int
	endpointIDs  [2]int
	ties         *revolveLoftFacesHeelArcSideTies
}

type revolveLoftFacesRay struct {
	y, z      *big.Rat
	seamIndex int
	seamRefs  []revolveLoftFacesRaySeamRef
	capIDs    []int
}

type revolveLoftFacesRaySeamRef struct {
	run, station int
}

// This builds every authenticated Revolve wall and end cap on one angular
// grid. One whole-arc source contact splits its finite meridian side. Source
// cells select the ring; held output edges never supply a crossing location.
func revolveLoftFacesBlankShellAtArc(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	sectionIndex, cellIndex int,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	return revolveLoftFacesBlankShellAtArcWithCuts(ctx, proof, manifest,
		sectionIndex, cellIndex, nil)
}

func revolveLoftFacesBlankShellAtArcWithCuts(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	sectionIndex, cellIndex int, capCuts []revolveLoftFacesShellCut,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	return revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof, manifest,
		[]revolveLoftFacesShellSeamSelector{{sectionIndex: sectionIndex, cellIndex: cellIndex}}, capCuts)
}

func revolveLoftFacesBlankShellAtRunsWithCuts(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	selectors []revolveLoftFacesShellSeamSelector, capCuts []revolveLoftFacesShellCut,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.tooth == nil || manifest == nil ||
		len(selectors) == 0 ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false, nil
	}
	payload, payloadOK := proof.tooth.n.Payload.(facetedPayload)
	if !payloadOK || payload.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) {
		return nil, false, nil
	}
	census, currentOK, err := revolveLoftFacesCensusIn(ctx, proof)
	if err != nil || !currentOK {
		return nil, false, err
	}
	current := census.manifest
	if !revolveLoftFacesSameShellManifest(manifest, current) {
		return nil, false, nil
	}
	manifest = current
	seams := make([][]revolveLoftFacesSeamVertex, len(selectors))
	seamCuts := make([]revolveLoftFacesShellCut, len(selectors))
	seenSelectors := make(map[revolveLoftFacesShellSeamSelector]struct{}, len(selectors))
	stationCount := 0
	for run, selector := range selectors {
		if _, duplicate := seenSelectors[selector]; duplicate {
			return nil, false, nil
		}
		seenSelectors[selector] = struct{}{}
		cut, vertices, valid, cutErr := revolveLoftFacesShellSeamCutOf(ctx, proof, manifest, selector)
		if cutErr != nil || !valid {
			return nil, false, cutErr
		}
		cut.seamRun = run
		seamCuts[run], seams[run] = cut, vertices
		stationCount += len(vertices)
	}
	maxRadius := 0.0
	for _, span := range proof.section.Spans {
		maxRadius = max(maxRadius, span.Start.V, span.End.V)
	}
	if proof.section.AxisSpan < 0 || !proofbound.FiniteVec(r3.Vec{X: maxRadius}) ||
		maxRadius <= 0 || maxRadius > 0.04*1024*1024 {
		return nil, false, nil
	}
	// A quarter-circle rational parameter advances by at most 2/n radians.
	// Its maximum sagitta is at most radius/(2*n*n).
	n := max(16, int(math.Ceil(math.Sqrt(maxRadius/0.04))))
	if n > 1024 {
		return nil, false, nil
	}
	capVertices := make([]revolveLoftFacesCapSeamVertex, 0, 2*len(capCuts))
	for i := range capCuts {
		vertices, built, err := revolveLoftFacesCapArcSeamVerticesIn(ctx, proof, census, capCuts[i].arc)
		if err != nil || !built {
			return nil, false, err
		}
		ties, tied, err := revolveLoftFacesHeelArcSideTiesIn(ctx, proof, census, capCuts[i].arc)
		if err != nil || !tied {
			return nil, false, err
		}
		capCuts[i].endpointIDs = [2]int{len(capVertices), len(capVertices) + 1}
		capCuts[i].ties = ties
		capVertices = append(capVertices, vertices[:]...)
	}
	rays := revolveLoftFacesShellRaysForRuns(n, seams, capVertices)
	if len(rays) < 4 || len(rays) > 4*n+stationCount+len(capVertices) ||
		len(rays)*len(proof.section.Spans) > 1<<16 {
		return nil, false, nil
	}
	if len(seams) == 1 {
		for i := range rays {
			if len(rays[i].seamRefs) == 1 {
				rays[i].seamIndex = rays[i].seamRefs[0].station
			}
		}
		if len(capVertices) != 0 {
			orderRays := make([]revolveloftfaces.RayOrderRay, len(rays))
			for i, ray := range rays {
				orderRays[i] = revolveloftfaces.RayOrderRay{
					Y: ray.y, Z: ray.z, SeamIndex: ray.seamIndex, CapCount: len(ray.capIDs),
				}
				if len(ray.capIDs) != 0 {
					orderRays[i].CapID = ray.capIDs[0]
				}
			}
			orderSeam := make([]revolveloftfaces.RayOrderSeam, len(seams[0]))
			for i, point := range seams[0] {
				orderSeam[i] = revolveloftfaces.RayOrderSeam{Point: point.point, LoftBound: point.loftBound}
			}
			capBoxes := make([]revolveloftfaces.RayBox, len(capVertices))
			for i, point := range capVertices {
				capBoxes[i] = revolveloftfaces.RayBox{
					Y: point.boundary.box[1], Z: point.boundary.box[2],
				}
			}
			if !revolveloftfaces.CapRayOrderBounded(orderRays, orderSeam, capBoxes) {
				return nil, false, nil
			}
		}
	} else if !revolveLoftFacesAllRayOrderBounded(rays, seams, capVertices) {
		return nil, false, nil
	}
	positiveYAxis := -1
	for index, ray := range rays {
		if ray.y.Sign() > 0 && ray.z.Sign() == 0 {
			if positiveYAxis >= 0 {
				return nil, false, nil
			}
			positiveYAxis = index
		}
	}
	if positiveYAxis < 0 {
		return nil, false, nil
	}
	out := &revolveLoftFacesBlankSeamShell{chordBound: maxRadius / (2 * float64(n*n)),
		seams: make([][]int, len(seams)), rayCount: len(rays), rays: rays}
	for run := range seams {
		out.seams[run] = make([]int, len(seams[run]))
		for i := range out.seams[run] {
			out.seams[run][i] = -1
		}
	}
	out.seam = out.seams[0]
	group := make([]int, len(proof.section.Spans))
	for i, span := range proof.section.Spans {
		group[i] = -1
		if span.Face == nil {
			if i != proof.section.AxisSpan {
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
		group[i] = len(out.mesh.Groups)
		out.mesh.Groups = append(out.mesh.Groups,
			facetGroup{Origins: append([]provenance.Ref(nil), span.Origins...), Planar: faceHandle(span.Face).isPlanar()})
	}
	appendPoint := func(point r3.Vec, bound float64) int {
		index := len(out.mesh.Verts)
		out.mesh.Verts = append(out.mesh.Verts, point)
		out.vertexBounds = append(out.vertexBounds, bound)
		return index
	}
	makeRing := func(x, radius *big.Rat, cut *revolveLoftFacesShellCut) ([]int, bool) {
		if radius.Sign() < 0 {
			return nil, false
		}
		if radius.Sign() == 0 {
			point, bound, valid := revolveloftfaces.HeldPoint(loftFaceRatVec{x, lfZero(), lfZero()})
			if !valid {
				return nil, false
			}
			return []int{appendPoint(point, bound)}, true
		}
		ring := make([]int, len(rays))
		for j, ray := range rays {
			if cut != nil && cut.seam {
				matched := false
				for _, ref := range ray.seamRefs {
					if ref.run != cut.seamRun {
						continue
					}
					vertex := seams[ref.run][ref.station]
					ring[j] = appendPoint(vertex.point, vertex.blankBound)
					out.seams[ref.run][ref.station] = ring[j]
					matched = true
					break
				}
				if matched {
					continue
				}
			}
			if cut != nil && cut.arc != nil {
				found := -1
				for endpoint, id := range cut.endpointIDs {
					if slices.Contains(ray.capIDs, id) {
						found = endpoint
					}
				}
				if found >= 0 {
					vertex := capVertices[cut.endpointIDs[found]]
					ring[j] = appendPoint(vertex.point, vertex.blankBound)
					continue
				}
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
	rings := make([][]int, len(proof.section.Spans))
	for i, span := range proof.section.Spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		var valid bool
		rings[i], valid = makeRing(lfF(span.Start.U), lfF(span.Start.V), nil)
		if !valid {
			return nil, false, nil
		}
	}
	cuts := make([][]revolveLoftFacesShellCut, len(proof.section.Spans))
	for _, cut := range seamCuts {
		cuts[cut.sectionIndex] = append(cuts[cut.sectionIndex], cut)
	}
	for _, cut := range capCuts {
		if cut.arc == nil || cut.sectionIndex < 0 ||
			cut.sectionIndex >= len(proof.section.Spans) ||
			cut.x == nil || cut.radius == nil || cut.seam ||
			cut.arc.circle.x == nil || cut.arc.circle.radius == nil ||
			cut.arc.circle.span != cut.sectionIndex ||
			cut.arc.circle.x.Cmp(cut.x) != 0 ||
			cut.arc.circle.radius.Cmp(cut.radius) != 0 {
			return nil, false, nil
		}
		current, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx, proof, census, cut.arc)
		if err != nil || !current {
			return nil, false, err
		}
		cuts[cut.sectionIndex] = append(cuts[cut.sectionIndex], cut)
	}
	if (len(proof.section.Spans)+len(capCuts)+len(seamCuts))*len(rays) > 1<<16 {
		return nil, false, nil
	}
	for i := range cuts {
		span := proof.section.Spans[i]
		parameter := func(cut revolveLoftFacesShellCut) (*big.Rat, bool) {
			startX, endX := lfF(span.Start.U), lfF(span.End.U)
			startR, endR := lfF(span.Start.V), lfF(span.End.V)
			if startX.Cmp(endX) != 0 {
				fraction := lfDiv(lfSub(cut.x, startX), lfSub(endX, startX))
				return fraction, lfAdd(startR, lfMul(fraction, lfSub(endR, startR))).Cmp(cut.radius) == 0
			}
			if cut.x.Cmp(startX) != 0 || startR.Cmp(endR) == 0 {
				return nil, false
			}
			return lfDiv(lfSub(cut.radius, startR), lfSub(endR, startR)), true
		}
		for _, cut := range cuts[i] {
			fraction, valid := parameter(cut)
			if !valid || fraction.Sign() <= 0 || fraction.Cmp(lfOne()) >= 0 {
				return nil, false, nil
			}
		}
		sort.Slice(cuts[i], func(a, b int) bool {
			first, _ := parameter(cuts[i][a])
			second, _ := parameter(cuts[i][b])
			return first.Cmp(second) < 0
		})
		for j := range cuts[i] {
			if j > 0 {
				previous, _ := parameter(cuts[i][j-1])
				current, _ := parameter(cuts[i][j])
				if previous.Cmp(current) >= 0 {
					return nil, false, nil
				}
			}
			var valid bool
			cuts[i][j].vertices, valid = makeRing(cuts[i][j].x,
				cuts[i][j].radius, &cuts[i][j])
			if !valid {
				return nil, false, nil
			}
			if cuts[i][j].arc != nil {
				endpoints := [2]int{-1, -1}
				for rayIndex, ray := range rays {
					for side, id := range cuts[i][j].endpointIDs {
						if slices.Contains(ray.capIDs, id) {
							if endpoints[side] >= 0 {
								return nil, false, nil
							}
							endpoints[side] = cuts[i][j].vertices[rayIndex]
						}
					}
				}
				if endpoints[0] < 0 || endpoints[1] < 0 || endpoints[0] == endpoints[1] {
					return nil, false, nil
				}
				occupied, pathOK := revolveLoftFacesOccupiedRingPath(
					cuts[i][j].vertices, endpoints, positiveYAxis,
					cuts[i][j].arc.throughPositiveY)
				if !pathOK {
					return nil, false, nil
				}
				out.capRings = append(out.capRings, revolveLoftFacesShellCapRing{
					sectionIndex: i, arc: cuts[i][j].arc,
					vertices: cuts[i][j].vertices, endpoints: endpoints,
					occupied: occupied,
					endpointProof: [2]revolveLoftFacesCapSeamVertex{
						capVertices[cuts[i][j].endpointIDs[0]],
						capVertices[cuts[i][j].endpointIDs[1]]},
					ties: cuts[i][j].ties})
			}
		}
	}
	out.sectionRings = make([][][]int, len(cuts))
	for i := range cuts {
		out.sectionRings[i] = append(out.sectionRings[i], rings[i])
		for _, cut := range cuts[i] {
			out.sectionRings[i] = append(out.sectionRings[i], cut.vertices)
		}
		out.sectionRings[i] = append(out.sectionRings[i], rings[(i+1)%len(rings)])
	}
	addBand := func(start, end []int, source, section, band int) bool {
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
				out.mesh.Add([3]int{start[0], end[j], end[next]}, source)
				out.triangleCells = append(out.triangleCells,
					revolveLoftFacesShellTriangleCell{section: section, band: band, ray: j})
			case len(end) == 1:
				out.mesh.Add([3]int{start[j], end[0], start[next]}, source)
				out.triangleCells = append(out.triangleCells,
					revolveLoftFacesShellTriangleCell{section: section, band: band, ray: j})
			default:
				out.mesh.Add([3]int{start[j], end[j], end[next]}, source)
				out.mesh.Add([3]int{start[j], end[next], start[next]}, source)
				out.triangleCells = append(out.triangleCells,
					revolveLoftFacesShellTriangleCell{section: section, band: band, ray: j},
					revolveLoftFacesShellTriangleCell{section: section, band: band, ray: j, half: 1})
			}
		}
		return true
	}
	for i := range proof.section.Spans {
		if group[i] < 0 {
			continue
		}
		start := rings[i]
		for band, cut := range cuts[i] {
			if !addBand(start, cut.vertices, group[i], i, band) {
				return nil, false, nil
			}
			start = cut.vertices
		}
		if !addBand(start, rings[(i+1)%len(rings)], group[i], i, len(cuts[i])) {
			return nil, false, nil
		}
	}
	for _, run := range out.seams {
		for _, index := range run {
			if index < 0 {
				return nil, false, nil
			}
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
		out.mesh.Verts, out.mesh.Tris, len(out.mesh.Tris), 1<<20); err != nil {
		return nil, false, err
	}
	if !revolveLoftFacesBlankShellEulerTwo(&out.mesh) {
		return nil, false, nil
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, out.mesh.Verts, out.mesh.Tris)
	if err != nil {
		return nil, false, err
	}
	if audit == nil || len(audit.Members) != 1 || audit.CompVol[0].Sign() <= 0 {
		return nil, false, nil
	}
	return out, true, nil
}

func revolveLoftFacesBlankShellEulerTwo(mesh *joinedMesh) bool {
	if mesh == nil || len(mesh.Verts) == 0 || len(mesh.Tris) == 0 {
		return false
	}
	edges := make(map[[2]int]struct{}, 3*len(mesh.Tris)/2)
	used := make([]bool, len(mesh.Verts))
	for _, tri := range mesh.Tris {
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(mesh.Verts) || b >= len(mesh.Verts) || a == b {
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
	return len(mesh.Verts)-len(edges)+len(mesh.Tris) == 2
}

func revolveLoftFacesSameArcContact(a, b revolveloftfaces.RevolveLoftFacesMappedCell) bool {
	return a.Segment == b.Segment && a.Span == b.Span && a.Band == b.Band &&
		a.Side == revolveloftfaces.RevolveLoftFacesJointContact && b.Side == a.Side &&
		a.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact && b.JointKind == a.JointKind &&
		a.JointArc == b.JointArc && a.JointPoint == b.JointPoint &&
		a.SLo != nil && b.SLo != nil && a.SLo.Cmp(b.SLo) == 0 &&
		a.SHi != nil && b.SHi != nil && a.SHi.Cmp(b.SHi) == 0 &&
		a.TLo != nil && b.TLo != nil && a.TLo.Cmp(b.TLo) == 0 &&
		a.THi != nil && b.THi != nil && a.THi.Cmp(b.THi) == 0 &&
		a.TLoOpen == b.TLoOpen && a.THiOpen == b.THiOpen
}

// A candidate must carry its retained source cells; a marked outcome alone
// cannot turn an empty nonvertical face into certified contact.
func revolveLoftFacesShellSideCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	index int, span revolveSectionSpan) (bool, error) {
	switch manifest.sideOutcome[index] {
	case revolveLoftFacesSideSourceCandidate:
		if span.Start.U != span.End.U {
			for _, side := range manifest.nonvertical {
				if side.sectionIndex == index {
					return faceNode(side.face) == span.Face && len(side.cells) > 0 &&
						slices.Equal(side.origins, span.Origins), nil
				}
			}
			return false, nil
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveLoftFacesVerticalContact ||
			side == nil || faceNode(side.face) != span.Face || len(side.cells) == 0 {
			return false, err
		}
		for _, entry := range manifest.vertical {
			if entry.sectionIndex == index {
				return entry.outcome == revolveLoftFacesVerticalContact &&
					entry.side != nil && faceNode(entry.side.face) == span.Face &&
					len(entry.side.cells) > 0, nil
			}
		}
	case revolveLoftFacesSideMiss:
		if span.Start.U != span.End.U {
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

func revolveLoftFacesShellRays(n int, seam []revolveLoftFacesSeamVertex,
	capVertices []revolveLoftFacesCapSeamVertex) []revolveLoftFacesRay {
	rays := make([]revolveLoftFacesRay, 0, 4*n+len(seam)+len(capVertices))
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
	for i, point := range capVertices {
		if point.point.Y == 0 && point.point.Z == 0 {
			return nil
		}
		rays = append(rays, revolveLoftFacesRay{
			y: lfF(point.point.Y), z: lfF(point.point.Z), seamIndex: -1,
			capIDs: []int{i}})
	}
	sort.Slice(rays, func(i, j int) bool {
		return revolveloftfaces.RayBefore(rays[i].y, rays[i].z, rays[j].y, rays[j].z)
	})
	unique := rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !revolveLoftFacesSameRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		// A cap endpoint needs its own ray. A collision with a grid, seam,
		// or second cap ray would erase its certified angular interval.
		if len(ray.capIDs) != 0 || len(unique[len(unique)-1].capIDs) != 0 {
			return nil
		}
		if ray.seamIndex >= 0 {
			if unique[len(unique)-1].seamIndex >= 0 {
				return nil
			}
			unique[len(unique)-1].seamIndex = ray.seamIndex
		}
	}
	return unique
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
	point, round, ok := revolveloftfaces.HeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	actual := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
	residual := proofbound.RatFloatUp(lfDiv(
		lfAbs(lfSub(actual, lfMul(radius, radius))), radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}

// Each selected source cell must reprove its whole-arc contact before its
// exact meridian coordinate can add a cut to the shared blank shell.
func revolveLoftFacesShellSeamCutOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, manifest *revolveLoftFacesContactManifest,
	selector revolveLoftFacesShellSeamSelector,
) (revolveLoftFacesShellCut, []revolveLoftFacesSeamVertex, bool, error) {
	var empty revolveLoftFacesShellCut
	sectionIndex, cellIndex := selector.sectionIndex, selector.cellIndex
	if sectionIndex < 0 || sectionIndex >= len(proof.section.Spans) ||
		sectionIndex >= len(manifest.sideOutcome) ||
		manifest.sideOutcome[sectionIndex] != revolveLoftFacesSideSourceCandidate {
		return empty, nil, false, nil
	}
	section := proof.section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 {
		return empty, nil, false, nil
	}
	var side *revolveLoftFacesNonverticalSide
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			if side != nil {
				return empty, nil, false, nil
			}
			side = &manifest.nonvertical[i]
		}
	}
	if side == nil || faceNode(side.face) != section.Face ||
		!slices.Equal(side.origins, section.Origins) ||
		cellIndex < 0 || cellIndex >= len(side.cells) {
		return empty, nil, false, nil
	}
	cell := side.cells[cellIndex]
	mapped := cell.mapped
	if mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
		mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact || mapped.JointArc == nil ||
		mapped.TLo == nil || mapped.THi == nil || mapped.TLoOpen || mapped.THiOpen ||
		mapped.Segment < 0 || mapped.Segment >= len(proof.trim.Source.Source.Entities) ||
		proof.trim.Source.Source.Entities[mapped.Segment] != mapped.JointArc ||
		mapped.Band < 0 || mapped.Band+1 >= len(proof.trim.Maps) {
		return empty, nil, false, nil
	}
	if mapped.Band >= len(proof.sourceRefs) ||
		mapped.Segment >= len(proof.sourceRefs[mapped.Band]) ||
		cell.ref != proof.sourceRefs[mapped.Band][mapped.Segment] {
		return empty, nil, false, nil
	}
	liveFace, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, cell.ref)
	if !unique || liveFace == nil || liveFace != cell.face {
		return empty, nil, false, nil
	}
	arcCells, certified, err := revolveloftfaces.RevolveLoftFacesArcSpanCells(ctx, proof.trim, section, mapped.Segment)
	if err != nil || !certified {
		return empty, nil, false, err
	}
	matched := false
	for _, source := range arcCells {
		matched = matched || revolveLoftFacesSameArcContact(source, mapped)
	}
	if !matched {
		return empty, nil, false, nil
	}
	stations, complete := revolveLoftFacesCompleteArcRun(proof.trim, mapped.Segment)
	if !complete {
		return empty, nil, false, nil
	}
	seam, _, _, built, err := revolveLoftFacesCertifiedArcPatches(ctx, proof.blank, proof.tooth,
		sectionIndex, mapped.Segment, mapped.Band,
		loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, stations)
	if err != nil || !built {
		return empty, nil, false, err
	}
	x := seam[0].exact[0]
	for _, vertex := range seam[1:] {
		if vertex.exact[0].Cmp(x) != 0 {
			return empty, nil, false, nil
		}
	}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	radius := lfAdd(lfMul(m, x), b)
	if radius.Sign() <= 0 {
		return empty, nil, false, nil
	}
	return revolveLoftFacesShellCut{sectionIndex: sectionIndex, x: x,
		radius: radius, seam: true}, seam, true, nil
}

// One angular ray may carry station anchors from several different meridian
// cuts. Their event IDs remain separate even when their exact rays coincide.
func revolveLoftFacesShellRaysForRuns(n int, seams [][]revolveLoftFacesSeamVertex,
	capVertices []revolveLoftFacesCapSeamVertex) []revolveLoftFacesRay {
	rays := revolveLoftFacesShellRays(n, nil, capVertices)
	if rays == nil {
		return nil
	}
	for run, seam := range seams {
		for station, point := range seam {
			rays = append(rays, revolveLoftFacesRay{y: point.exact[1], z: point.exact[2],
				seamIndex: -1, seamRefs: []revolveLoftFacesRaySeamRef{{run: run, station: station}}})
		}
	}
	sort.Slice(rays, func(i, j int) bool {
		return revolveloftfaces.RayBefore(rays[i].y, rays[i].z, rays[j].y, rays[j].z)
	})
	unique := rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !revolveLoftFacesSameRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		previous := &unique[len(unique)-1]
		if len(ray.capIDs) != 0 || len(previous.capIDs) != 0 {
			return nil
		}
		for _, ref := range ray.seamRefs {
			for _, existing := range previous.seamRefs {
				if existing.run == ref.run {
					return nil
				}
			}
			previous.seamRefs = append(previous.seamRefs, ref)
		}
	}
	return unique
}

// Every source anchor must stay in the same cyclic angular order as the
// shared ring grid. A cap endpoint and a seam station cannot share a ray.
func revolveLoftFacesAllRayOrderBounded(rays []revolveLoftFacesRay,
	seams [][]revolveLoftFacesSeamVertex, capVertices []revolveLoftFacesCapSeamVertex) bool {
	if len(rays) < 4 {
		return false
	}
	boxes := make([][]revolveloftfaces.RayBox, len(rays))
	seenCaps := make([]bool, len(capVertices))
	seenStations := make([][]bool, len(seams))
	for run := range seams {
		seenStations[run] = make([]bool, len(seams[run]))
	}
	for index, ray := range rays {
		if ray.y == nil || ray.z == nil || len(ray.capIDs) > 1 ||
			len(ray.capIDs) > 0 && len(ray.seamRefs) > 0 {
			return false
		}
		if len(ray.capIDs) == 0 && len(ray.seamRefs) == 0 {
			boxes[index] = []revolveloftfaces.RayBox{{
				Y: loftFaceInterval{Lo: ray.y, Hi: ray.y},
				Z: loftFaceInterval{Lo: ray.z, Hi: ray.z}}}
		}
		for _, id := range ray.capIDs {
			if id < 0 || id >= len(capVertices) || seenCaps[id] {
				return false
			}
			seenCaps[id] = true
			boundary := capVertices[id].boundary
			boxes[index] = append(boxes[index], revolveloftfaces.RayBox{
				Y: boundary.box[1], Z: boundary.box[2]})
		}
		for _, ref := range ray.seamRefs {
			if ref.run < 0 || ref.run >= len(seams) ||
				ref.station < 0 || ref.station >= len(seams[ref.run]) ||
				seenStations[ref.run][ref.station] {
				return false
			}
			seenStations[ref.run][ref.station] = true
			point := seams[ref.run][ref.station]
			if proofbound.IsNonFinite(point.loftBound) || point.loftBound < 0 {
				return false
			}
			bound := lfF(point.loftBound)
			y, z := lfF(point.point.Y), lfF(point.point.Z)
			boxes[index] = append(boxes[index], revolveloftfaces.RayBox{
				Y: loftFaceInterval{Lo: lfSub(y, bound), Hi: lfAdd(y, bound)},
				Z: loftFaceInterval{Lo: lfSub(z, bound), Hi: lfAdd(z, bound)}})
		}
		for _, box := range boxes[index] {
			if !revolveloftfaces.RayBoxValid(box) {
				return false
			}
		}
	}
	for _, seen := range seenCaps {
		if !seen {
			return false
		}
	}
	for _, run := range seenStations {
		for _, seen := range run {
			if !seen {
				return false
			}
		}
	}
	for index := range rays {
		next := (index + 1) % len(rays)
		for _, first := range boxes[index] {
			for _, second := range boxes[next] {
				if revolveloftfaces.RayCrossRange(first, second).Lo.Sign() <= 0 {
					return false
				}
			}
		}
	}
	return true
}

// This graph consumer places every certified whole-arc run and occupied heel
// arc on one closed Revolve shell. It makes no retained-face decision.
func revolveLoftFacesBlankShellWithAllRuns(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]revolveLoftFacesShellSeamSelector, 0, len(graph.runs))
	for _, run := range graph.runs {
		if run.cellID < 0 || run.cellID >= len(graph.cells) {
			return nil, nil, false, nil
		}
		cell := graph.cells[run.cellID]
		if cell.source == nil || cell.pairID < 0 ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, revolveLoftFacesShellSeamSelector{
			sectionIndex: cell.sectionIndex, cellIndex: cell.cellIndex})
	}
	capCuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof,
		graph.manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.faceBound >= 0.1 ||
		len(shell.seams) != len(graph.runs) ||
		len(shell.triangleCells) != len(shell.mesh.Tris) {
		return nil, nil, false, err
	}
	shell.seamRunIDs = make([][]int, len(graph.runs))
	for i := range graph.runs {
		view := *shell
		view.seam = shell.seams[i]
		bound, bindErr := graph.bindBlankShell(ctx, proof, &graph.runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.seamRunIDs[i] = view.seamIDs
	}
	shell.seamIDs = shell.seamRunIDs[0]
	return shell, graph, true, nil
}
