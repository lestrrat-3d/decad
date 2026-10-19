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
	"github.com/lestrrat-3d/r3"
)

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
