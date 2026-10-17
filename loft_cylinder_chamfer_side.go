package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// middleSidePrep is the read-only input of the retained middle-side proof:
// the validated payload, the tooth faces and the old shell's audit data. The
// grid is the one mutable part; a worker uses a Fork of it.
type middleSidePrep struct {
	fp        facetedPayload
	mesh      *joinedMesh
	source    loftCylinderChamferSource
	ideal     []proofarith.Xpt
	middle    [2]int
	clipBound float64
	faces     map[provenance.Ref][]loftCylinderChamferToothFace
	old       *loftmesh.LoftAuditData
	oldBoxes  [][2]r3.Vec
	grid      *loftmesh.ChamferBoxGrid
}

// middleState is the mutable state one facet at a time may use: a grid, the
// old shell's parity mesh and the tooth caches. The caches only memoise pure
// functions of the read-only input, or, for the held-root memos, answers those
// functions gave, so any state gives the same answers.
type middleState struct {
	grid         *loftmesh.ChamferBoxGrid
	parity       *meshbool.ParityMesh
	toothParity  map[*Body]*meshbool.ParityMesh
	toothCircles map[*Body]loftcylinder.SourceCircle
	toothRoots   map[*Body]*loftmesh.ChamferHeldRootMemo
}

func newMiddleState(grid *loftmesh.ChamferBoxGrid, parity *meshbool.ParityMesh) *middleState {
	return &middleState{
		grid: grid, parity: parity,
		toothParity:  make(map[*Body]*meshbool.ParityMesh),
		toothCircles: make(map[*Body]loftcylinder.SourceCircle),
		toothRoots:   make(map[*Body]*loftmesh.ChamferHeldRootMemo),
	}
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
		fp: fp, mesh: mesh, source: source, ideal: ideal, middle: middle, clipBound: clipBound,
		faces: faces, old: old, oldBoxes: oldBoxes, grid: oldGrid,
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
	st := newMiddleState(prep.grid, meshbool.NewParityMesh(prep.fp.Verts, prep.fp.Tris))
	for i := first; i < prep.middle[1]; i++ {
		if i%32 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if err := prep.middleFacet(ctx, i, st); err != nil {
			return err
		}
	}
	return nil
}

func (prep *middleSidePrep) middleFacet(ctx context.Context, i int, st *middleState) error {
	mesh, source, ideal := prep.mesh, prep.source, prep.ideal
	group := mesh.Src[i]
	if group < 0 || group >= len(mesh.Groups) || len(mesh.Groups[group].Origins) == 0 {
		return fmt.Errorf("%w: retained middle facet lost its source group", ErrUnsupported)
	}
	origin := mesh.Groups[group].Origins[0]
	var base [3]proofarith.Xpt
	for j, index := range mesh.Tris[i] {
		if index < 0 || index >= len(ideal) {
			return fmt.Errorf("%w: retained middle facet has an invalid vertex", ErrUnsupported)
		}
		base[j] = ideal[index]
		z := meshbool.RatCoordOf(base[j], 2)
		if z.Cmp(proofarith.FloatRat(source.z0)) <= 0 ||
			z.Cmp(proofarith.FloatRat(source.z1)) >= 0 {
			return fmt.Errorf("%w: retained middle facet reaches a source cap", ErrUnsupported)
		}
	}
	last := "no inward source witness"
	var corners [6]loftmesh.ChamferCorner
	baseRounded := false
	for _, distance := range [...]float64{0.08, 0.06, 0.04, 0.02, 0.01} {
		q, shift, err := loftmesh.ChamferMiddleOffset(base, distance, prep.clipBound)
		if err != nil {
			return err
		}
		if !loftmesh.ChamferMiddleBeyondBore(q, source.boreRadius) {
			last = "inward patch reaches the source bore"
			continue
		}
		for k := range q {
			corners[3+k] = loftmesh.ChamferCornerOf(q[k])
		}
		qBox, qBoxed := loftmesh.ChamferCornersBox(corners[3], corners[4], corners[5])
		if !loftmesh.ChamferMiddleBlank(q, source.blankRadius) {
			candidates := prep.faces[origin]
			if len(candidates) == 0 {
				last = "source face has no authenticated tooth"
				continue
			}
			certified := false
			for _, face := range candidates {
				parity, ok := st.toothParity[face.body]
				if !ok {
					parity = meshbool.NewParityMesh(face.loft.Verts, face.loft.Tris)
					st.toothParity[face.body] = parity
				}
				if err := loftCylinderChamferMiddleToothSource(ctx, q, qBox, qBoxed, face,
					parity, source.blankRadius, st.toothCircles, st.toothRoots); err != nil {
					last = err.Error()
					continue
				}
				certified = true
				break
			}
			if !certified {
				continue
			}
		}
		if !baseRounded {
			for k := range base {
				corners[k] = loftmesh.ChamferCornerOf(base[k])
			}
			baseRounded = true
		}
		if err := loftmesh.ChamferMiddlePrism(ctx, len(prep.fp.Tris), prep.old, prep.oldBoxes, st.grid,
			st.parity, base, q, shift, corners); err != nil {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			last = err.Error()
			continue
		}
		return nil
	}
	return fmt.Errorf("%w: retained middle facet %d (origin %v) has no certified inward collar: %s",
		ErrUnsupported, i, origin, last)
}

// A whole inward patch is in the true Loft tooth when it misses every moving
// wall by its matched departure and one held-tooth point is inside. If only
// hidden root walls meet it, use the exact blank disk for that part instead.
func loftCylinderChamferMiddleToothSource(ctx context.Context, q [3]proofarith.Xpt,
	qBox [2]r3.Vec, boxed bool,
	face loftCylinderChamferToothFace, parity *meshbool.ParityMesh,
	blankRadius float64, circles map[*Body]loftcylinder.SourceCircle,
	roots map[*Body]*loftmesh.ChamferHeldRootMemo) error {
	lp := face.loft
	if lp.Walls <= 0 || lp.Walls > len(lp.Tris) || face.root < 3 ||
		face.scale == nil || face.scale.Sign() <= 0 ||
		2*face.root >= lp.Walls || face.bound < 0 || proofbound.IsNonFinite(face.bound) {
		return fmt.Errorf("%w: inward patch lacks a bounded Loft wall", ErrUnsupported)
	}
	fullClear := true
	for j := range lp.Walls {
		if j%64 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		tri := lp.Tris[j]
		if boxed && loftmesh.ChamferBoxGapAbove(qBox, meshbool.TriBox(lp.Verts, tri), face.bound) {
			continue
		}
		other := [3]proofarith.Xpt{
			proofarith.XptOf(lp.Verts[tri[0]]),
			proofarith.XptOf(lp.Verts[tri[1]]),
			proofarith.XptOf(lp.Verts[tri[2]]),
		}
		if loftmesh.ChamferTriangleGapSplit(q, other, face.bound, 2) ||
			meshbool.DisjointTriDistanceAbove(q, other, face.bound) {
			continue
		}
		if j < 2*face.root {
			return fmt.Errorf("%w: inward patch approaches exterior tooth facet %d", ErrUnsupported, j)
		}
		fullClear = false
	}
	if fullClear {
		witness := meshbool.XCentroid(q[0], q[1], q[2])
		inside, on, err := meshbool.MeshParityAllContext(ctx, witness, parity)
		if err != nil {
			return err
		}
		if inside && !on {
			return nil
		}
	}
	// The original root arc is hidden by the blank over this middle slab.
	// Both matched held and true root patches must remain in the convex disk;
	// their entire same-height homotopy then stays in that disk too.
	circle, ok := circles[face.body]
	if !ok {
		var err error
		circle, err = loftcylinder.SourceCircleOf(lp, blankRadius)
		if err != nil {
			return err
		}
		circles[face.body] = circle
	}
	zLo, zHi := meshbool.RatCoordOf(q[0], 2), meshbool.RatCoordOf(q[0], 2)
	for _, p := range q[1:] {
		z := meshbool.RatCoordOf(p, 2)
		if z.Cmp(zLo) < 0 {
			zLo = z
		}
		if z.Cmp(zHi) > 0 {
			zHi = z
		}
	}
	mappedLo := new(big.Rat).Quo(zLo, face.scale)
	mappedHi := new(big.Rat).Quo(zHi, face.scale)
	memo := roots[face.body]
	if memo == nil {
		memo = new(loftmesh.ChamferHeldRootMemo)
		roots[face.body] = memo
	}
	if mappedLo.Cmp(circle.SafeBottom) <= 0 || mappedHi.Cmp(circle.SafeTop) >= 0 ||
		!memo.Inside(lp.Verts, lp.Tris, lp.Walls, face.root, mappedLo, mappedHi, blankRadius) {
		return fmt.Errorf("%w: inward patch has no hidden root-wall interval", ErrUnsupported)
	}
	r := proofarith.FloatRat(blankRadius)
	r2 := new(big.Rat).Mul(r, r)
	outside := 0
	for _, p := range q {
		x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
		diff := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
		if diff.Cmp(r2) <= 0 {
			continue
		}
		outside++
		inside, on, err := meshbool.MeshParityAllContext(ctx, p, parity)
		if err != nil {
			return err
		}
		if !inside || on {
			return fmt.Errorf("%w: inward patch exterior vertex leaves its held tooth", ErrUnsupported)
		}
	}
	if outside == 0 {
		return fmt.Errorf("%w: inward patch root split has no exterior witness", ErrUnsupported)
	}
	// Each connected part of a triangle outside a centered convex disk reaches
	// an outside vertex: from x choose a vertex v with x·v >= |x|², and the
	// segment x--v stays outside. No source wall meets that part, so its
	// inside vertex fixes occupancy. The remaining patch is inside the blank.
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
	states := make(chan *middleState, workers)
	for range workers {
		states <- newMiddleState(prep.grid.Fork(), meshbool.NewParityMesh(in.fp.Verts, in.fp.Tris))
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
		err := prep.middleFacet(privates[k], in.middle[0]+k-1, st)
		logs[k-1] = st.grid.TakeLog()
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
