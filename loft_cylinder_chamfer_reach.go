package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This measures only the part of the inset rim in a radial uncertainty band
// around the source root. Complete cap source coverage is checked separately.
func loftCylinderChamferCapRadialReach(source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) (float64, error) {
	if source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		proofbound.IsNonFinite(outerRadius) || capIndex < 0 || capIndex > 1 ||
		outerRadius <= source.blankRadius {
		return 0, fmt.Errorf("%w: cap radial band has invalid limits", ErrUnsupported)
	}
	outer := proofarith.FloatRat(outerRadius)
	outer2 := new(big.Rat).Mul(outer, outer)
	for _, tooth := range source.teeth {
		if tooth == nil {
			return 0, fmt.Errorf("%w: cap radial band lacks a source tooth", ErrUnsupported)
		}
		lp, ok := tooth.n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction {
			return 0, fmt.Errorf("%w: cap slit lacks an authenticated tooth", ErrUnsupported)
		}
		profile := lp.Profile0
		if capIndex == 1 {
			profile = lp.Profile1
		}
		if len(profile.Outer.Segments) <= 5 {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		arc, ok := profile.Outer.Segments[5].(arcSeg)
		if !ok {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		p := loftcylinder.SourcePoint(lp.Xform.Basis().EX, arc.Start)
		if q := loftcylinder.SourceDot(p, p); q.Cmp(outer2) > 0 {
			return 0, fmt.Errorf("%w: cap radial band misses a source root", ErrUnsupported)
		}
	}
	return loftcylinder.ChamferCapRadialRun(verts, ring, source.blankRadius, source.boreRadius, outerRadius)
}

// Every true tooth cap contour outside the centered root arc is matched to a
// retained held chord. Union copied these chords, in order, into its audited
// outer cap loop. A later inset whose exact clearance from that whole loop
// exceeds the Loft departure cannot be crossed by any such source contour.
func loftCylinderChamferExteriorLineage(fp facetedPayload,
	source loftCylinderChamferSource, capIndex int) error {
	if capIndex < 0 || capIndex > 1 || len(source.outer[capIndex]) < 3 ||
		fp.MeshBound < 0 || proofbound.IsNonFinite(fp.MeshBound) {
		return fmt.Errorf("%w: cap has no exterior contour correspondence", ErrUnsupported)
	}
	outer := source.outer[capIndex]
	edges := make(map[[2]r3.Vec]struct{}, len(outer))
	for i, index := range outer {
		next := outer[(i+1)%len(outer)]
		if index < 0 || index >= len(fp.Verts) || next < 0 || next >= len(fp.Verts) {
			return fmt.Errorf("%w: cap outer loop has invalid indices", ErrUnsupported)
		}
		edges[[2]r3.Vec{fp.Verts[index], fp.Verts[next]}] = struct{}{}
	}
	for _, body := range source.teeth {
		if body == nil {
			return fmt.Errorf("%w: cap lacks an original tooth", ErrUnsupported)
		}
		lp, ok := body.n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction || lp.Proof.FacetDeparture < 0 ||
			lp.Proof.FacetDeparture > fp.MeshBound || len(lp.Verts)%2 != 0 ||
			len(lp.Segment) != 1 {
			return fmt.Errorf("%w: cap tooth has no bounded source contour", ErrUnsupported)
		}
		n := len(lp.Verts) / 2
		if len(lp.Segment[0]) != n {
			return fmt.Errorf("%w: cap tooth has no station-to-curve map", ErrUnsupported)
		}
		root := -1
		for j, segment := range lp.Segment[0] {
			if segment == 5 {
				root = j
				break
			}
			if segment < 0 || segment > 4 {
				return fmt.Errorf("%w: cap tooth exterior has an unexpected source curve", ErrUnsupported)
			}
		}
		if root < 5 || root >= n-1 {
			return fmt.Errorf("%w: cap tooth has no separate root arc", ErrUnsupported)
		}
		first := capIndex * n
		for j := range root {
			if _, ok := edges[[2]r3.Vec{lp.Verts[first+j], lp.Verts[first+j+1]}]; !ok {
				return fmt.Errorf("%w: a source tooth exterior chord is absent from the held cap", ErrUnsupported)
			}
		}
	}
	return nil
}

func loftCylinderChamferCapToothOf(body *Body, capIndex int, z float64) (loftcylinder.ChamferCapTooth, error) {
	if body == nil {
		return loftcylinder.ChamferCapTooth{}, fmt.Errorf("%w: cap lacks a source tooth", ErrUnsupported)
	}
	lp, ok := body.n.Payload.(loftPayload)
	if !ok || !lp.AuthenticatedReconstruction || len(lp.Verts)%2 != 0 ||
		lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) {
		return loftcylinder.ChamferCapTooth{}, fmt.Errorf("%w: cap tooth has no matched Loft contour", ErrUnsupported)
	}
	return loftcylinder.ChamferCapToothOf(lp.Verts, lp.Proof.FacetDeparture, capIndex, z)
}

// Each source tooth's held cap chords and exact cap curve have the same
// authenticated parameters and a two-sided FacetDeparture bound. Strict
// chord clearance keeps a complete inset edge away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied.
// This checks the inset rim only. The cap interior and root band are separate.
func loftCylinderChamferExteriorRimInside(ctx context.Context, source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) error {
	if len(ring) < 3 || len(ring) > 4096 || capIndex < 0 || capIndex > 1 ||
		len(source.teeth) == 0 || len(source.teeth) > 64 ||
		source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		outerRadius <= source.blankRadius || proofbound.IsNonFinite(outerRadius) {
		return fmt.Errorf("%w: cap exterior source has invalid limits", ErrUnsupported)
	}
	z := source.z0
	if capIndex == 1 {
		z = source.z1
	}
	teeth := make([]loftcylinder.ChamferCapTooth, len(source.teeth))
	for i, body := range source.teeth {
		tooth, err := loftCylinderChamferCapToothOf(body, capIndex, z)
		if err != nil {
			return err
		}
		teeth[i] = tooth
	}
	return loftcylinder.ChamferExteriorRimInside(ctx, teeth, verts, ring, source.blankRadius, outerRadius)
}

// A planar cap tiles one simple outer loop and, for a bore, one nested hole.
// Every other triangle meets its plane only at a loop vertex or full loop
// edge. Thus all cap pairs have exactly the indexed contact of that tiling;
// the exact pair audit only needs the non-cap triangles. This avoids the
// quadratic box scan among the many coplanar cap triangles.
func loftCylinderChamferCrossingAudit(ctx context.Context, mesh *joinedMesh,
	source loftCylinderChamferSource, outer [2][]int, maxTriangles int) error {
	if len(mesh.Tris) > maxTriangles || len(mesh.Src) != len(mesh.Tris) ||
		source.z0 >= source.z1 || source.capGroups[0] == source.capGroups[1] {
		return fmt.Errorf("%w: cap chamfer has no bounded shell partition", ErrUnsupported)
	}
	var vertices [2]map[int]struct{}
	var edges [2]map[[2]int]struct{}
	for cap := range 2 {
		z := source.z0
		if cap == 1 {
			z = source.z1
		}
		if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, outer[cap]) {
			return fmt.Errorf("%w: cap %d has no simple outer loop", ErrUnsupported, cap)
		}
		if len(source.inner[cap]) > 0 {
			if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, source.inner[cap]) {
				return fmt.Errorf("%w: cap %d has no simple bore loop", ErrUnsupported, cap)
			}
			nested, err := loftcylinder.ChamferRingNested(ctx, mesh.Verts, outer[cap], source.inner[cap])
			if err != nil {
				return err
			}
			if !nested || !loftCylinderAuditAnnulus(ctx, mesh, outer[cap], source.inner[cap],
				source.capGroups[cap], z, cap == 0) {
				return fmt.Errorf("%w: cap %d does not tile a nested annulus", ErrUnsupported, cap)
			}
		} else if !loftCylinderAuditCap(ctx, mesh, outer[cap], source.capGroups[cap], z, cap == 0) {
			return fmt.Errorf("%w: cap %d does not tile its loop", ErrUnsupported, cap)
		}
		vertices[cap] = make(map[int]struct{}, len(outer[cap])+len(source.inner[cap]))
		edges[cap] = make(map[[2]int]struct{}, len(outer[cap])+len(source.inner[cap]))
		for _, ring := range [][]int{outer[cap], source.inner[cap]} {
			for i, a := range ring {
				b := ring[(i+1)%len(ring)]
				vertices[cap][a] = struct{}{}
				edges[cap][loftcylinder.ChamferEdge(a, b)] = struct{}{}
			}
		}
	}
	wall := make([][3]int, 0, len(mesh.Tris))
	for i, tri := range mesh.Tris {
		if i%256 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if mesh.Src[i] == source.capGroups[0] || mesh.Src[i] == source.capGroups[1] {
			continue
		}
		wall = append(wall, tri)
		for cap := range 2 {
			z := source.z0
			if cap == 1 {
				z = source.z1
			}
			var on []int
			for _, index := range tri {
				if index < 0 || index >= len(mesh.Verts) ||
					mesh.Verts[index].Z < source.z0 || mesh.Verts[index].Z > source.z1 {
					return fmt.Errorf("%w: cap chamfer wall leaves its axial slab", ErrUnsupported)
				}
				if mesh.Verts[index].Z == z {
					if _, ok := vertices[cap][index]; !ok {
						return fmt.Errorf("%w: cap chamfer wall meets a cap away from its loop", ErrUnsupported)
					}
					on = append(on, index)
				}
			}
			if len(on) == 3 {
				return fmt.Errorf("%w: cap chamfer wall lies in a cap plane", ErrUnsupported)
			}
			if len(on) == 2 {
				if _, ok := edges[cap][loftcylinder.ChamferEdge(on[0], on[1])]; !ok {
					return fmt.Errorf("%w: cap chamfer wall has no shared cap edge", ErrUnsupported)
				}
			}
		}
	}
	maxScans := uint64(min(64_000_000, 2_048*len(wall)))
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewPollWorkBudget(ctx), mesh.Verts, wall,
		maxTriangles, maxScans); err != nil {
		return fmt.Errorf("cap chamfer wall crossing audit (%d facets, %d pair scans): %w",
			len(wall), maxScans, err)
	}
	return nil
}

// The source-ring audit supplies a true-result boundary point on every
// split edge. Exact distance to that ring then bounds every band facet.
func loftCylinderChamferNearBandReach(ctx context.Context, mesh *joinedMesh,
	fp facetedPayload, source loftCylinderChamferSource, ideal []proofarith.Xpt, bands [2][2]int,
	clipBound, publishedBound float64, multipleRings bool, reflectedShift []*big.Rat) error {
	reach := math.Nextafter(publishedBound-clipBound, math.Inf(-1))
	if reach <= 0 || proofbound.IsNonFinite(reach) {
		return fmt.Errorf("%w: cap band has no boundary allowance", ErrUnsupported)
	}
	reachRat := proofarith.FloatRat(reach)
	reach2 := new(big.Rat).Mul(reachRat, reachRat)
	fractions := []*big.Rat{big.NewRat(11, 20)}
	if multipleRings {
		fractions = []*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4)}
	}
	teeth := make(loftCylinderRingTeeth)
	for cap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		capLevel := proofarith.FloatRat(source.z0)
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.z1)
		}
		var near [][3]proofarith.Xpt
		var edges []loftmesh.ChamferSplitEdge
		for _, fraction := range fractions {
			var sourceEdges [][2]proofarith.Xpt
			for i := band[0]; i < band[1]; i++ {
				parts, edge, err := loftmesh.ChamferNearPieces(mesh.Tris[i], ideal, capLevel, fraction)
				if err != nil {
					return err
				}
				if !multipleRings {
					near = append(near, parts...)
					far, err := loftmesh.ChamferFarTriangles(mesh.Tris[i], ideal, capLevel, fraction)
					if err != nil {
						return err
					}
					near = append(near, far...)
				}
				edges = append(edges, edge)
				sourceEdges = append(sourceEdges, [2]proofarith.Xpt{edge.A, edge.B})
			}
			// The direct and the reflected check read the same edges.
			records := new(loftcylinder.RingEdgeRecords)
			if _, err := loftCylinderSourceRingInside(ctx, fp, sourceEdges, teeth, records); err != nil {
				return err
			}
			if len(reflectedShift) != 0 {
				shifted := source
				shifted.ringLevelShift = reflectedShift
				if _, err := loftCylinderSourceRingInsideInputs(ctx, shifted, sourceEdges, teeth, records); err != nil {
					return fmt.Errorf("reflected cap split ring: %w", err)
				}
			}
		}
		if multipleRings {
			for i := band[0]; i < band[1]; i++ {
				tri := mesh.Tris[i]
				near = append(near, [3]proofarith.Xpt{ideal[tri[0]], ideal[tri[1]], ideal[tri[2]]})
			}
		}
		grid, err := loftmesh.ChamferEdgeGridOf(edges, reachRat)
		if err != nil {
			return err
		}
		maxSteps := min(8_000_000, 512*len(edges))
		depth := 10
		if multipleRings {
			maxSteps = min(8_000_000, 2_048*len(edges))
			depth = 12
		}
		steps := 0
		for i, tri := range near {
			if !loftmesh.ChamferNearCovered(ctx, tri, grid, reach2, depth, &steps, maxSteps) {
				var diagnostic [4]float64
				points := [4]proofarith.Xpt{tri[0], tri[1], tri[2],
					meshbool.XCentroid(tri[0], tri[1], tri[2])}
				for k, p := range points {
					diagnostic[k] = math.Inf(1)
					for _, edge := range edges {
						diagnostic[k] = min(diagnostic[k],
							meshbool.SegSegDist3(p.Vec(), p.Vec(), edge.A.Vec(), edge.B.Vec()))
					}
				}
				return fmt.Errorf("%w: near cap band %d facet %d has no certified split-ring reach after %d checks: distances=%v bound=%.17g",
					ErrUnsupported, cap, i, steps, diagnostic, publishedBound)
			}
		}
	}
	return nil
}

// middleSidePrep is the read-only input of the retained middle-side proof:
// the validated payload, the tooth faces and the old shell's audit data. The
// grid is the one mutable part; a worker uses a Fork of it.
type middleSidePrep struct {
	fp     facetedPayload
	middle [2]int
	grid   *loftmesh.ChamferBoxGrid
	proof  loftcylinder.ChamferMiddleInput
}

// middleSidePrepHook, when set by a test, sees each prepared proof before its
// facets run.
var middleSidePrepHook func(*middleSidePrep)

func newMiddleSidePrep(fp facetedPayload, mesh *joinedMesh, source loftCylinderChamferSource,
	ideal []proofarith.Xpt, middle [2]int, clipBound float64) (*middleSidePrep, error) {
	if middle[0] < 0 || middle[1] > len(mesh.Tris) || middle[0] >= middle[1] ||
		len(fp.Src) != len(fp.Tris) || proofbound.IsNonFinite(clipBound) {
		return nil, fmt.Errorf("%w: cap chamfer has no retained middle side", ErrUnsupported)
	}
	faces, err := loftCylinderChamferToothFaces(source)
	if err != nil {
		return nil, err
	}
	old := loftmesh.NewLoftAuditData(fp.Verts, fp.Tris)
	oldBoxes := make([][2]r3.Vec, len(fp.Tris))
	for i, tri := range fp.Tris {
		oldBoxes[i] = meshbool.TriBox(fp.Verts, tri)
	}
	oldGrid, err := loftmesh.ChamferBoxGridOf(oldBoxes, source.z0, source.z1)
	if err != nil {
		return nil, err
	}
	// A whole old boundary component cannot hide inside a middle collar:
	// every old vertex is on a cap plane, outside every strict middle prism.
	for _, p := range fp.Verts {
		if p.Z != source.z0 && p.Z != source.z1 {
			return nil, fmt.Errorf("%w: joined source has an interior-height vertex", ErrUnsupported)
		}
	}
	prep := &middleSidePrep{
		fp: fp, middle: middle, grid: oldGrid,
		proof: loftcylinder.ChamferMiddleInput{
			Mesh: mesh, Ideal: ideal, TriCount: len(fp.Tris), ClipBound: clipBound,
			Faces: faces, Old: old, OldBoxes: oldBoxes,
			Z0: source.z0, Z1: source.z1,
			BoreRadius: source.boreRadius, BlankRadius: source.blankRadius,
		},
	}
	if middleSidePrepHook != nil {
		middleSidePrepHook(prep)
	}
	return prep, nil
}

// A retained middle facet is an exact subset of an old held wall. An inward
// horizontal collar ending inside the source solid gives every point of that
// facet a source-cut boundary point within the collar's length.
func loftCylinderChamferMiddleSideReach(ctx context.Context, fp facetedPayload,
	mesh *joinedMesh, source loftCylinderChamferSource, ideal []proofarith.Xpt,
	middle [2]int, clipBound float64) error {
	prep, err := newMiddleSidePrep(fp, mesh, source, ideal, middle, clipBound)
	if err != nil {
		return err
	}
	return prep.serialFrom(ctx, middle[0])
}

// serialFrom runs the facets first..middle[1] in order on the real grid.
func (prep *middleSidePrep) serialFrom(ctx context.Context, first int) error {
	st := loftcylinder.NewChamferMiddleState(prep.grid,
		meshbool.NewParityMesh(prep.fp.Verts, prep.fp.Tris))
	for i := first; i < prep.middle[1]; i++ {
		if i%32 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if err := loftcylinder.ChamferMiddleFacet(ctx, prep.proof, i, st); err != nil {
			return err
		}
	}
	return nil
}

// loftCylinderChamferReachInput is everything the reach proofs read: the
// finished mesh, the source records and the published allowance.
type loftCylinderChamferReachInput struct {
	fp             facetedPayload
	mesh           *joinedMesh
	source         loftCylinderChamferSource
	ideal          []proofarith.Xpt
	bands          [2][2]int
	middle         [2]int
	clipBound      float64
	allowance      float64
	multipleRings  bool
	reflectedShift []*big.Rat
}

// loftCylinderChamferReachHook, when set by a test, sees each reach input.
var loftCylinderChamferReachHook func(*loftCylinderChamferReachInput)

func (in *loftCylinderChamferReachInput) nearBand(ctx context.Context) error {
	return loftCylinderChamferNearBandReach(ctx, in.mesh, in.fp, in.source, in.ideal, in.bands,
		in.clipBound, in.allowance, in.multipleRings, in.reflectedShift)
}

// loftCylinderChamferReach runs the near-band proof, the bore inheritance
// check and the middle-side proof, and returns the error a serial run of the
// three in that order returns. With workers < 2 it is that serial run.
// Otherwise the near band and the middle facets share one flat fan-out: each
// facet runs on a forked grid, and the visit log of each is replayed on the
// real grid in facet order. From the first facet whose replay the real grid
// would refuse, the facets run serially.
func loftCylinderChamferReach(ctx context.Context, in *loftCylinderChamferReachInput, workers int) error {
	if loftCylinderChamferReachHook != nil {
		loftCylinderChamferReachHook(in)
	}
	if workers < 2 {
		if err := in.nearBand(ctx); err != nil {
			return err
		}
		if err := loftCylinderChamferBoreInheritance(in.fp, in.mesh, in.source); err != nil {
			return err
		}
		return loftCylinderChamferMiddleSideReach(ctx, in.fp, in.mesh, in.source, in.ideal,
			in.middle, in.clipBound)
	}
	inheritErr := loftCylinderChamferBoreInheritance(in.fp, in.mesh, in.source)
	var prep *middleSidePrep
	var prepErr error
	if inheritErr == nil {
		prep, prepErr = newMiddleSidePrep(in.fp, in.mesh, in.source, in.ideal, in.middle, in.clipBound)
	}
	if inheritErr != nil || prepErr != nil {
		if err := in.nearBand(ctx); err != nil {
			return err
		}
		if inheritErr != nil {
			return inheritErr
		}
		return prepErr
	}
	facets := in.middle[1] - in.middle[0]
	// Each item has its own context derived from the caller's values only;
	// the polling caller alone cancels them.
	base := context.WithoutCancel(ctx)
	privates := make([]context.Context, 1+facets)
	cancels := make([]context.CancelFunc, 1+facets)
	for k := range privates {
		privates[k], cancels[k] = context.WithCancel(base) //nolint:fatcontext
	}
	cancelAll := func() {
		for _, cancel := range cancels {
			cancel()
		}
	}
	defer cancelAll()
	states := make(chan *loftcylinder.ChamferMiddleState, workers)
	for range workers {
		states <- loftcylinder.NewChamferMiddleState(prep.grid.Fork(),
			meshbool.NewParityMesh(in.fp.Verts, in.fp.Tris))
	}
	logs := make([][]loftmesh.ChamferGridQuery, facets)
	poll := func() error {
		if err := ctx.Err(); err != nil {
			cancelAll()
			return err
		}
		return nil
	}
	first, err := orderedwork.Each(1+facets, workers, 1, poll, func(k int) error { //nolint:contextcheck // private contexts, see above
		if k == 0 {
			return in.nearBand(privates[0])
		}
		st := <-states
		defer func() { states <- st }()
		err := loftcylinder.ChamferMiddleFacet(privates[k], prep.proof, in.middle[0]+k-1, st)
		logs[k-1] = st.Grid.TakeLog()
		return err
	})
	if first <= 0 {
		// A poll failure, or the near-band error, which a serial run reaches
		// before any facet.
		return err
	}
	// Facets 0..last-1 (the failing one included) are the ones the serial run
	// reaches; replay them on the real grid.
	last := facets
	if first < 1+facets {
		last = first
	}
	for t := range last {
		if !prep.grid.Replay(logs[t]) {
			return prep.serialFrom(ctx, in.middle[0]+t)
		}
	}
	return err
}

type loftCylinderRingTooth struct {
	loft   loftPayload
	circle loftcylinder.SourceCircle
	bound2 *big.Rat
	// At a certified open circle height, root walls are strictly inside the
	// blank and cannot bound an edge wholly outside its radius.
	exterior []loftcylinder.RingWall
	// The fitted and tip cells exclude both source connector walls and the
	// hidden root wall. A seam segment checks connectors exactly instead.
	fitted []loftcylinder.RingWall
}

func loftCylinderRingToothOf(body *Body, radius float64) (loftCylinderRingTooth, error) {
	if body == nil {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth source is absent", ErrUnsupported)
	}
	lp, ok := body.n.Payload.(loftPayload)
	if !ok {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no Loft source", ErrUnsupported)
	}
	circle, err := loftcylinder.SourceCircleOf(lp, radius)
	if err != nil {
		return loftCylinderRingTooth{}, err
	}
	bound := lp.Proof.FacetDeparture
	if bound < 0 || proofbound.IsNonFinite(bound) {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no finite wall bound", ErrUnsupported)
	}
	exterior, fitted, ok := loftcylinder.RingToothWalls(lp, bound)
	if !ok {
		return loftCylinderRingTooth{}, fmt.Errorf("%w: ring tooth has no wall cell map", ErrUnsupported)
	}
	return loftCylinderRingTooth{
		loft: lp, circle: circle,
		bound2:   new(big.Rat).Mul(proofarith.FloatRat(bound), proofarith.FloatRat(bound)),
		exterior: exterior, fitted: fitted,
	}, nil
}

// loftCylinderRingToothKey names the inputs of loftCylinderRingToothOf.
type loftCylinderRingToothKey struct {
	body   *Body
	radius float64
}

// loftCylinderRingTeeth holds the tooth records one near-band proof has
// built. A record is a function of its body's payload and the radius
// alone, and every reader only reads it. Only successfully built records are
// kept, so a call that fails after building some teeth can leave those
// records in the map. The near-band proof returns that error and drops the
// map. A nil map keeps nothing.
type loftCylinderRingTeeth map[loftCylinderRingToothKey]loftCylinderRingTooth

// loftCylinderSourceRingInside proves that an unordered exact split ring lies
// inside the original blank-and-teeth union at one strict interior height.
// Each edge avoids every true wall; one point inside the exact blank fixes
// occupancy of the connected cycle. Root walls lie inside the blank below
// each tooth's certified collar. Circle-crossing edges use the source circle
// sector and exact connector slices; all other exposed walls use the Loft's
// two-sided, parameter-matched wall displacement. A bore is excluded by its
// exact radius. The caller separately proves its cutter band's full surface,
// both cap planes, and either root collar.
func loftCylinderSourceRingInside(ctx context.Context, fp facetedPayload,
	edges [][2]proofarith.Xpt, teeth loftCylinderRingTeeth,
	records *loftcylinder.RingEdgeRecords) (loftcylinder.RingOccupancy, error) {
	if !fp.ContactAudited || fp.Xform != r3.Identity() {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no audited source body", ErrUnsupported)
	}
	source, ok := loftCylinderChamferSourceOf(fp)
	if !ok {
		return loftcylinder.RingOccupancy{},
			fmt.Errorf("%w: split ring has no retained loft/cylinder source", ErrUnsupported)
	}
	bridgeBound, topSourceLevel, err := loftcylinder.ChamferBridgeSources(&loftcylinder.ChamferBridgeSourcesInput{
		ProofTeeth: bodyNodes(source.teeth), OriginalTeeth: bodyNodes(source.originalTeeth),
		AxialBridge: source.axialBridge, Z0: source.z0, Z1: source.z1, BridgeBound: source.bridgeBound,
	})
	if err != nil {
		return loftcylinder.RingOccupancy{}, err
	}
	source.bridgeBound, source.topSourceLevel = bridgeBound, topSourceLevel
	return loftCylinderSourceRingInsideInputs(ctx, source, edges, teeth, records)
}

func loftCylinderSourceRingInsideInputs(ctx context.Context, source loftCylinderChamferSource,
	edges [][2]proofarith.Xpt, teeth loftCylinderRingTeeth,
	records *loftcylinder.RingEdgeRecords) (loftcylinder.RingOccupancy, error) {
	var result loftcylinder.RingOccupancy
	refuse := func(reason string) (loftcylinder.RingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if len(source.teeth) == 0 || source.blankRadius <= 0 || source.boreRadius < 0 ||
		source.boreRadius >= source.blankRadius || proofbound.IsNonFinite(source.blankRadius) ||
		proofbound.IsNonFinite(source.boreRadius) ||
		len(source.ringLevelShift) != 0 && len(source.ringLevelShift) != len(source.teeth) || ctx.Err() != nil {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		return refuse("ring lacks a certified blank and tooth source")
	}
	level, valid := loftcylinder.RingCycle(edges)
	if !valid || level.Cmp(proofarith.FloatRat(source.z0)) <= 0 ||
		level.Cmp(source.topSourceLevel) >= 0 {
		return refuse("ring is not one strict interior cycle")
	}
	toothRecords := make([]loftcylinder.RingToothLevel, len(source.teeth))
	for i, body := range source.teeth {
		key := loftCylinderRingToothKey{body: body, radius: source.blankRadius}
		tooth, cached := teeth[key]
		if !cached {
			var err error
			tooth, err = loftCylinderRingToothOf(body, source.blankRadius)
			if err != nil {
				return result, err
			}
			if teeth != nil {
				teeth[key] = tooth
			}
		}
		if tooth.circle.Z0.Cmp(proofarith.FloatRat(source.z0)) != 0 ||
			tooth.circle.Z1.Cmp(proofarith.FloatRat(source.z1)) != 0 {
			return refuse("ring tooth has different blank cap levels")
		}
		original, ok := source.originalTeeth[i].n.Payload.(loftPayload)
		if !ok {
			return refuse("ring has no authenticated original tooth")
		}
		scale := proofarith.FloatRat(original.Xform.Basis().EZ.Z)
		mappedLevel := new(big.Rat).Set(level)
		if len(source.ringLevelShift) != 0 {
			shift := source.ringLevelShift[i]
			if shift == nil || shift.Sign() < 0 {
				return refuse("ring has no finite physical tooth level")
			}
			mappedLevel.Sub(mappedLevel, shift)
		}
		mapped := new(big.Rat).Quo(mappedLevel, scale)
		if mapped.Cmp(tooth.circle.SafeBottom) <= 0 || mapped.Cmp(tooth.circle.SafeTop) >= 0 {
			return refuse("ring meets a source cap or root collar")
		}
		toothRecords[i] = loftcylinder.RingToothLevel{Circle: tooth.circle, Level: mapped,
			Bound2: tooth.bound2, Exterior: tooth.exterior, Fitted: tooth.fitted}
	}
	if records == nil {
		records = new(loftcylinder.RingEdgeRecords)
	}
	return loftcylinder.RingEdgesInsideShared(ctx, edges, toothRecords, source.blankRadius, source.boreRadius, records)
}
